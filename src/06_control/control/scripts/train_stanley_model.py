#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
train_stanley_model.py

Stanley 동적 파라미터(k_e, k_v, base_lookahead) 자율 적응 추정을 위한 PyTorch 신경망 모델 및 옵션 오프라인 학습기.

입력 (6차원 차량 상태 오차):
  [v, yaw_rate, steer_cmd, e_ct, e_yaw, curve_gain]

출력 (3차원 동적 Stanley 파라미터):
  [k_e_scale, k_v_scale, lookahead_scale]
"""

import os
import argparse
import numpy as np
import pandas as pd
import torch
import torch.nn as nn
import torch.optim as optim

class NormalizedStanleyParamMLP(nn.Module):
    def __init__(self, mean, std, input_dim=6, hidden_dim=64, output_dim=4):
        super(NormalizedStanleyParamMLP, self).__init__()
        
        self.register_buffer('mean', torch.tensor(mean, dtype=torch.float32))
        self.register_buffer('std',  torch.tensor(std,  dtype=torch.float32) + 1e-6)
        
        self.net = nn.Sequential(
            nn.Linear(input_dim, hidden_dim),
            nn.LayerNorm(hidden_dim),
            nn.SiLU(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.LayerNorm(hidden_dim),
            nn.SiLU(),
            nn.Linear(hidden_dim, output_dim),
            nn.Sigmoid()  # 0.0 ~ 1.0 범위 출력
        )

        # 파라미터별 안전 물리 범위
        # k_e: [0.15, 2.5]
        # k_v: [0.5, 6.0]
        # lookahead: [1.5, 7.0] m
        # lateral_offset: [0.00, 0.50] m (양수 전용 보정 오프셋)
        self.register_buffer('min_val', torch.tensor([0.15, 0.5, 1.5, 0.00], dtype=torch.float32))
        self.register_buffer('max_val', torch.tensor([2.5,  6.0, 7.0, 0.50], dtype=torch.float32))

    def forward(self, x):
        # 정규화
        x_norm = (x - self.mean) / self.std
        sig = self.net(x_norm)
        # 물리적 스케일로 바운딩
        out = self.min_val + (self.max_val - self.min_val) * sig
        return out


def train_offline(csv_path, output_model_path, epochs=100, batch_size=64, lr=1e-3):
    if not os.path.exists(csv_path):
        print(f"[ERROR] 데이터셋 파일이 존재하지 않습니다: {csv_path}")
        return

    df = pd.read_csv(csv_path)
    required_cols = ['v', 'yaw_rate', 'steer', 'e_ct', 'e_yaw', 'curve_gain']
    for c in required_cols:
        if c not in df.columns:
            print(f"[ERROR] 누락된 열: {c}")
            return

    X = df[required_cols].values.astype(np.float32)
    mean = np.mean(X, axis=0)
    std = np.std(X, axis=0)

    # 기본 타겟 스케일 생성 (오차에 비례하는 k_e, k_v, lookahead, lateral_offset 라벨 생성)
    e_ct = np.abs(X[:, 3])
    e_yaw = np.abs(X[:, 4])
    v = X[:, 0]
    curve = X[:, 5]

    # k_e 라벨: 오차가 클수록 크게 (0.15 ~ 2.5)
    target_k_e = np.clip(0.8 + 1.2 * e_ct + 0.5 * e_yaw, 0.15, 2.5)
    # k_v 라벨: 속도가 빠를수록 감쇄 댐핑 높게 (0.5 ~ 6.0)
    target_k_v = np.clip(1.0 + 0.2 * v, 0.5, 6.0)
    # lookahead 라벨: 속도와 곡률에 맞춘 전방 시야 (1.5 ~ 7.0m)
    target_lh  = np.clip(2.5 + 0.15 * v, 1.5, 7.0)
    # lateral_offset 라벨: 횡오차 및 원심력 보정 (0.00 ~ 0.50m)
    target_offset = np.clip(0.6 * e_ct + 0.3 * curve, 0.00, 0.50)

    Y = np.stack([target_k_e, target_k_v, target_lh, target_offset], axis=1).astype(np.float32)

    model = NormalizedStanleyParamMLP(mean, std, input_dim=6, hidden_dim=64, output_dim=4)
    optimizer = optim.AdamW(model.parameters(), lr=lr, weight_decay=1e-4)
    criterion = nn.MSELoss()

    dataset = torch.utils.data.TensorDataset(torch.from_numpy(X), torch.from_numpy(Y))
    loader = torch.utils.data.DataLoader(dataset, batch_size=batch_size, shuffle=True)

    model.train()
    print(f"[TRAIN] Stanley 파라미터 신경망 학습 시작 (데이터 샘플: {len(X)}개)")
    for epoch in range(1, epochs + 1):
        total_loss = 0.0
        for bx, by in loader:
            optimizer.zero_grad()
            pred = model(bx)
            loss = criterion(pred, by)
            loss.backward()
            optimizer.step()
            total_loss += loss.item() * len(bx)
        avg_loss = total_loss / len(X)
        if epoch % 10 == 0 or epoch == epochs:
            print(f"Epoch [{epoch}/{epochs}] Loss: {avg_loss:.6f}")

    model.eval()
    os.makedirs(os.path.dirname(output_model_path), exist_ok=True)
    
    # TorchScript 저장
    example_input = torch.randn(1, 6, dtype=torch.float32)
    traced_model = torch.jit.trace(model, example_input)
    traced_model.save(output_model_path)
    print(f"[SAVE] 학습 완료된 Stanley 모델이 저장되었습니다 -> {output_model_path}")


if __name__ == '__main__':
    try:
        import rospkg
        base_dir = rospkg.RosPack().get_path('control')
    except Exception:
        from pathlib import Path
        base_dir = str(Path(__file__).resolve().parents[1])

    parser = argparse.ArgumentParser(description="Stanley Dynamic Param Model Trainer")
    parser.add_argument('--csv', type=str, default=os.path.join(base_dir, 'data', 'stanley_dataset.csv'))
    parser.add_argument('--out', type=str, default=os.path.join(base_dir, 'model', 'stanley_param.pt'))
    parser.add_argument('--epochs', type=int, default=100)
    args = parser.parse_args()

    train_offline(args.csv, args.out, epochs=args.epochs)
