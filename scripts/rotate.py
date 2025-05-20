#!/usr/bin/env python3

import rospy
import math
from geometry_msgs.msg import Twist

def clamp(value, min_value, max_value):
    return max(min(value, max_value), min_value)

def rotate():
    rospy.init_node('rotate_node', anonymous=True)

    ANGULAR_SPEED_RAD = 0.5
    MAX_ANGLE_DEG = 720.0

    angle_deg = rospy.get_param('~angular_angle', 0.0)
    angle_deg = clamp(angle_deg, -MAX_ANGLE_DEG, MAX_ANGLE_DEG)

    rospy.loginfo(f"Rotating: angle={angle_deg}°")

    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.sleep(1)
    rate = rospy.Rate(10)
    move_cmd = Twist()
    direction = 1 if angle_deg > 0 else -1

    angle_rad = abs(angle_deg) * math.pi / 180.0
    rotate_time = angle_rad / ANGULAR_SPEED_RAD

    start_time = rospy.Time.now()
    while not rospy.is_shutdown():
        elapsed_time = (rospy.Time.now() - start_time).to_sec()
        if elapsed_time >= rotate_time:
            break
        move_cmd.linear.x = 0
        move_cmd.angular.z = ANGULAR_SPEED_RAD * direction
        cmd_vel_pub.publish(move_cmd)
        rate.sleep()

    move_cmd.angular.z = 0
    cmd_vel_pub.publish(move_cmd)
    rospy.loginfo("Rotation complete.")

if __name__ == '__main__':
    try:
        rotate()
    except rospy.ROSInterruptException:
        pass
