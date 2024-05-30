#!/usr/bin/env python
import rospy
import pandas as pd

from nav_msgs.msg import Odometry




def callback(data):
    global df
    # 提取位置和姿态数据
    x = data.pose.pose.position.x
    y = data.pose.pose.position.y
    z = data.pose.pose.position.z
    qx = data.pose.pose.orientation.x
    qy = data.pose.pose.orientation.y
    qz = data.pose.pose.orientation.z
    qw = data.pose.pose.orientation.w

    # 添加到DataFrame
    new_row = {'Time': rospy.get_time(), 'X': x, 'Y': y, 'Z': z, 'QX': qx, 'QY': qy, 'QZ': qz, 'QW': qw}
    df = df.append(new_row, ignore_index=True)

def listener():
    global df
    df = pd.DataFrame(columns=['Time', 'X', 'Y', 'Z', 'QX', 'QY', 'QZ', 'QW'])
    rospy.init_node('odom_to_csv', anonymous=True)
    rospy.Subscriber("/Odometry", Odometry, callback)
    rospy.spin()

    # 当节点关闭时，保存到CSV
    df.to_csv('odom_data.csv', index=False)

if __name__ == '__main__':
    listener()
