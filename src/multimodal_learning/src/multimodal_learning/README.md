# V17 fixed-spatial 30 m planner

V17 removes the final current-speed/temporal-path contradiction. The path
heads predict geometry at fixed route-progress stations `3, 6, 10, 15, 22,
30 m`; they do not predict where the vehicle will be after a fixed time.

- Input: Camera 3 history + LiDAR BEV history + one 30 m Goal Point
- Path output: DRIVE 6x2 and AVOID 6x2 at fixed spatial stations
- Speed output: six absolute speeds at the same reached spatial stations
- State output: DRIVE/STOP/AVOID probabilities
- Local Route: offline label construction only, never model input
- Current Speed: not required for path geometry

The external state machine still selects candidates, forces STOP speed to
zero, and passes the selected path through smoothing and 0.1 m resampling.

## 2 TB SSD 연결 방법

V17 보관본은 2 TB SSD의 두 번째 파티션에 있다.

```text
device: /dev/sda2
filesystem: ext4
UUID: b4bc958b-d82f-4611-96bd-67d55146f5bf
SSD project: /media/libok/b4bc958b-d82f-4611-96bd-67d55146f5bf/home/libok/morai_project
```

SSD를 연결한 뒤 장치와 마운트 상태를 확인한다.

```bash
lsblk -o NAME,SIZE,FSTYPE,UUID,MOUNTPOINTS /dev/sda
```

`/dev/sda2`에 마운트 경로가 표시되지 않으면 다음과 같이 마운트한다.

```bash
udisksctl mount -b /dev/sda2
```

마운트가 완료되면 V17 코드와 체크포인트를 확인한다.

```bash
V17_SSD_ROOT=/media/libok/b4bc958b-d82f-4611-96bd-67d55146f5bf/home/libok/morai_project
ls "$V17_SSD_ROOT/multimodal_planner_v17_spatial30"
ls "$V17_SSD_ROOT/training_outputs/multimodal_planner_v17_spatial30_morai_ft_v002_b4"
```

현재 `/home/libok/morai_project`의 작업본과 SSD 보관본은 서로 다른
디렉터리다. 기존 작업본을 덮어쓰지 않고 짧은 경로로 접근하려면 별도
심볼릭 링크를 한 번만 만든다.

```bash
ln -s /media/libok/b4bc958b-d82f-4611-96bd-67d55146f5bf/home/libok/morai_project \
  /home/libok/morai_project/v17_ssd_archive
```

이후에는 다음 경로로 SSD 보관본에 접근할 수 있다.

```text
/home/libok/morai_project/v17_ssd_archive/multimodal_planner_v17_spatial30
/home/libok/morai_project/v17_ssd_archive/training_outputs/multimodal_planner_v17_spatial30_morai_ft_v002_b4/best.pt
```

이미 `v17_ssd_archive`가 존재하면 `ln -s`를 다시 실행하지 않는다. SSD를
분리하기 전에는 SSD 경로를 사용하는 학습·추론·터미널을 모두 종료하고
다음 명령으로 안전하게 마운트를 해제한다.

```bash
udisksctl unmount -b /dev/sda2
```
