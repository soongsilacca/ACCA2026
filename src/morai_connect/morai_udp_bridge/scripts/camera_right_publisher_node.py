#!/usr/bin/env python3
"""Camera (right) → sensor_msgs/Image  |  UDP 9295"""
import sys
from pathlib import Path
sys.path.append(str(Path(__file__).resolve().parents[2]))

import cv2, numpy as np
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from lib.network.UDP import Receiver
from lib.define.Camera import Camera

IP       = '127.0.0.1'
PORT     = 9295
TOPIC    = '/camera/right/image_raw'
FRAME_ID = 'camera_right'


def build_image_msg(image_np, stamp):
    msg = Image()
    msg.header   = Header(stamp=stamp, frame_id=FRAME_ID)
    msg.height   = image_np.shape[0]
    msg.width    = image_np.shape[1]
    msg.encoding = 'bgr8'
    msg.step     = image_np.shape[1] * 3
    msg.data     = image_np.tobytes()
    return msg


def main():
    rospy.init_node('morai_camera_right_publisher', anonymous=False)
    port  = rospy.get_param('~port', PORT)
    topic = rospy.get_param('~topic', TOPIC)
    pub   = rospy.Publisher(topic, Image, queue_size=5)
    cam_data = Receiver(IP, port, Camera())
    rospy.loginfo(f"[Camera Right] UDP {IP}:{port} → {topic} (event-driven)")
    try:
        while not rospy.is_shutdown():
            try:
                data = cam_data._queue.get(timeout=0.5)
            except Exception:
                continue
            if not data.image.data:
                continue
            try:
                img = cv2.imdecode(np.frombuffer(data.image.data, dtype=np.uint8), cv2.IMREAD_COLOR)
            except Exception:
                continue
            if img is None:
                continue
            pub.publish(build_image_msg(img, rospy.Time.now()))
    finally:
        cam_data.stop()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
