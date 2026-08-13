#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CompressedImage

import tensorrt as trt
import pycuda.driver as cuda
# 1. [수정] 멀티쓰레드 충돌을 막기 위해 autoinit을 제거합니다.
# import pycuda.autoinit (삭제)

class TRTDepthNode:
    def __init__(self):
        rospy.init_node('depth_node', anonymous=True)
        self.bridge = CvBridge()
        
        # 2. [수정] CUDA Context를 수동으로 초기화합니다.
        cuda.init()
        self.cuda_device = cuda.Device(0)         # 0번 GPU 선택
        self.cuda_context = self.cuda_device.make_context() # Context 생성
        
        # 1. TensorRT 엔진 로드 및 초기화
        self.engine_path = "/home/acca/acca_ws/src/depth_perception/models/da3_fp16.engine"
        self.logger = trt.Logger(trt.Logger.WARNING)
        self.runtime = trt.Runtime(self.logger)
        
        rospy.loginfo("TensorRT 엔진 로드 중...")
        with open(self.engine_path, "rb") as f:
            self.engine = self.runtime.deserialize_cuda_engine(f.read())
        self.context = self.engine.create_execution_context()
        rospy.loginfo("TensorRT 엔진 로드 완료!")

        # 2. GPU 메모리 할당 (Buffer allocation)
        self.inputs, self.outputs, self.bindings, self.stream = self.allocate_buffers(self.engine)

        # 3. [수정] 초기화가 끝났으므로 일단 Context를 반환(pop) 해둡니다. 
        # (이후 콜백 쓰레드에서 가져다 쓸 수 있도록)
        self.cuda_context.pop()

        # 3. ROS Publisher & Subscriber 설정
        self.image_sub = rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.image_callback, queue_size=1)
        self.depth_pub = rospy.Publisher("/depth_map/image_raw", Image, queue_size=1)

    def allocate_buffers(self, engine):
        """Host(RAM)와 Device(VRAM) 간의 메모리 버퍼를 할당합니다."""
        inputs = []
        outputs = []
        bindings = []
        stream = cuda.Stream()

        for i in range(engine.num_bindings):
            name = engine.get_binding_name(i)
            dtype = trt.nptype(engine.get_binding_dtype(i))
            shape = engine.get_binding_shape(i)
            size = trt.volume(shape)
            
            host_mem = cuda.pagelocked_empty(size, dtype)
            device_mem = cuda.mem_alloc(host_mem.nbytes)
            bindings.append(int(device_mem))

            if engine.binding_is_input(i):
                inputs.append({'host': host_mem, 'device': device_mem, 'shape': shape, 'dtype': dtype})
            else:
                outputs.append({'host': host_mem, 'device': device_mem, 'shape': shape, 'dtype': dtype})
                
        return inputs, outputs, bindings, stream

    def image_callback(self, msg):
        # 4. [수정] 콜백 쓰레드가 시작될 때 GPU 제어권(Context)을 획득(push)합니다.
        self.cuda_context.push()
        try:
            # 1. ROS Image -> OpenCV BGR 변환
            cv_img = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
            
            # 2. 전처리
            img_rgb = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
            img_resized = cv2.resize(img_rgb, (518, 518))
            
            img_normalized = img_resized.astype(np.float32) / 255.0
            mean = np.array([0.485, 0.456, 0.406], dtype=np.float32)
            std = np.array([0.229, 0.224, 0.225], dtype=np.float32)
            img_normalized = (img_normalized - mean) / std
            
            img_chw = np.transpose(img_normalized, (2, 0, 1))
            input_tensor = np.expand_dims(np.expand_dims(img_chw, axis=0), axis=0)

            # 3. Host 데이터 덮어쓰기
            np.copyto(self.inputs[0]['host'], input_tensor.ravel())

            # 4. TensorRT 추론 실행
            cuda.memcpy_htod_async(self.inputs[0]['device'], self.inputs[0]['host'], self.stream)
            self.context.execute_async_v2(bindings=self.bindings, stream_handle=self.stream.handle)
            cuda.memcpy_dtoh_async(self.outputs[0]['host'], self.outputs[0]['device'], self.stream)
            self.stream.synchronize()

            # 5. 후처리
            depth_output = self.outputs[0]['host'].reshape((518, 518))
            
            depth_min = depth_output.min()
            depth_max = depth_output.max()
            depth_scaled = 255 * (depth_output - depth_min) / (depth_max - depth_min + 1e-5)
            depth_uint8 = depth_scaled.astype(np.uint8)
            
            depth_colormap = cv2.applyColorMap(depth_uint8, cv2.COLORMAP_INFERNO)
            
            # 6. Publish
            depth_msg = self.bridge.cv2_to_imgmsg(depth_colormap, "bgr8")
            self.depth_pub.publish(depth_msg)

        except Exception as e:
            rospy.logerr(f"Inference Error: {e}")
            
        finally:
            # 5. [수정] 콜백 작업이 끝나면 (에러가 나더라도) 반드시 GPU 제어권을 반환(pop)합니다.
            self.cuda_context.pop()

if __name__ == '__main__':
    try:
        node = TRTDepthNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass