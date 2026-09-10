#!/usr/bin/env python3
import rospy
import pandas as pd
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

def create_path_msg_from_csv(csv_filepath):
    df = pd.read_csv(csv_filepath)
    path = Path()
    path.header.frame_id = "map"

    for _, row in df.iterrows():
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.pose.position.x = float(row['x'])
        pose.pose.position.y = float(row['y'])
        pose.pose.orientation.w = 1.0
        
        path.poses.append(pose)
        
    return path

def main():
    rospy.init_node('global_path_publisher')
    pub = rospy.Publisher('/global_path', Path, queue_size=10, latch=True)
    
    path_msg = create_path_msg_from_csv('/home/acca/acca_ws/global_path/global_path.csv')
    
    rate = rospy.Rate(1) # 1Hz
    while not rospy.is_shutdown():
        now = rospy.Time.now()
        path_msg.header.stamp = now
        for pose in path_msg.poses:
            pose.header.stamp = now
            
        pub.publish(path_msg)
        rospy.loginfo("Published global path message")
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass