#!/usr/bin/env python3

import rospy
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

# Globals
current_yaw = None

def clamp(value, min_value, max_value):
    return max(min(value, max_value), min_value)

def normalize_angle(angle):
    """Normalize angle to [-pi, pi]"""
    return (angle + math.pi) % (2 * math.pi) - math.pi

def odom_callback(msg):
    global current_yaw
    orientation_q = msg.pose.pose.orientation
    _, _, yaw = euler_from_quaternion([
        orientation_q.x,
        orientation_q.y,
        orientation_q.z,
        orientation_q.w
    ])
    current_yaw = yaw

def rotate():
    rospy.init_node('rotate_node', anonymous=True)


    ANGULAR_SPEED_RAD = 0.3 
    MAX_ANGLE_DEG = 720.0
    ANGLE_TOLERANCE_RAD = math.radians(1)  # 1°

    angle_deg = rospy.get_param('~angular_angle', 0.0)
    angle_deg = clamp(angle_deg, -MAX_ANGLE_DEG, MAX_ANGLE_DEG)
    target_rotation = math.radians(abs(angle_deg))
    direction = 1 if angle_deg > 0 else -1

    rospy.loginfo(f"Rotating: angle={angle_deg}°")

    # Setup
    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/odom', Odometry, odom_callback)
    rospy.sleep(1.0)

    rate = rospy.Rate(10)
    while current_yaw is None and not rospy.is_shutdown():
        rospy.loginfo("Waiting for odometry...")
        rospy.sleep(0.1)

    # Tracking rotation
    last_yaw = current_yaw
    rotated = 0.0

    move_cmd = Twist()
    move_cmd.linear.x = 0
    move_cmd.angular.z = ANGULAR_SPEED_RAD * direction

    while not rospy.is_shutdown() and rotated < target_rotation - ANGLE_TOLERANCE_RAD:
        if current_yaw is None:
            continue

        delta_yaw = normalize_angle(current_yaw - last_yaw)
        rotated += abs(delta_yaw)
        last_yaw = current_yaw

        cmd_vel_pub.publish(move_cmd)
        rate.sleep()

    # Stop
    move_cmd.angular.z = 0
    cmd_vel_pub.publish(move_cmd)
    rospy.loginfo(f"Rotation complete (rotated {math.degrees(rotated):.1f}°).")

if __name__ == '__main__':
    try:
        rotate()
    except rospy.ROSInterruptException:
        pass
