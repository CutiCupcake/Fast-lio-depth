#!/usr/bin/env python3
# coding=utf8

import tf
import tf2_ros
import geometry_msgs.msg
import math
import rospy
# 创建一个tf2的广播器
rospy.init_node('trans', anonymous=True)
br = tf2_ros.TransformBroadcaster()

# 创建一个坐标系转换的消息
t = geometry_msgs.msg.TransformStamped()

# 设置父坐标系和子坐标系的名称
t.header.frame_id = "camera_init"
t.child_frame_id = "royale_camera_0_optical_frame"

# 设置时间戳
t.header.stamp = rospy.Time.now()

# 第一次旋转：绕y轴顺时针（实际上使用逆时针）旋转90度
q1 = tf.transformations.quaternion_from_euler(0, -math.pi/2, 0)

# 第二次旋转：绕z轴逆时针旋转90度
q2 = tf.transformations.quaternion_from_euler(0, 0, math.pi/2)

# 将两次旋转合并（四元数的乘法）
q_combined = tf.transformations.quaternion_multiply(q1, q2)

# 将合并后的旋转应用到坐标转换消息中
t.transform.rotation.x = q_combined[0]
t.transform.rotation.y = q_combined[1]
t.transform.rotation.z = q_combined[2]
t.transform.rotation.w = q_combined[3]

# 发送坐标系转换
br.sendTransform(t)
