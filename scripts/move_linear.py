#!/usr/bin/env python3

import rospy
import math
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

# Globals
obstacle_detected = False
movement_direction = 0

def clamp(value, min_value, max_value):
    return max(min(value, max_value), min_value)

def ramp_speed(target_speed, current_speed, ramp_rate, dt):
    delta_speed = ramp_rate * dt
    if current_speed < target_speed:
        current_speed = min(current_speed + delta_speed, target_speed)
    elif current_speed > target_speed:
        current_speed = max(current_speed - delta_speed, target_speed)
    return current_speed

def decelerate_speed(current_speed, ramp_rate, dt):
    delta_speed = ramp_rate * dt
    if current_speed > 0:
        current_speed = max(current_speed - delta_speed, 0)
    elif current_speed < 0:
        current_speed = min(current_speed + delta_speed, 0)
    return current_speed

def laser_callback(msg):
    global obstacle_detected, movement_direction

    if movement_direction == 0:
        obstacle_detected = False
        return

    center_angle = 0.0 if movement_direction == 1 else math.pi
    angle_tolerance = math.radians(10)

    angles = [msg.angle_min + i * msg.angle_increment for i in range(len(msg.ranges))]
    relevant_ranges = [
        r for a, r in zip(angles, msg.ranges)
        if abs((a - center_angle + math.pi) % (2 * math.pi) - math.pi) <= angle_tolerance
        and r > 0.0 and not math.isinf(r)
    ]

    obstacle_detected = any(r < 0.25 for r in relevant_ranges)

def move_linear():
    rospy.init_node('move_linear_node', anonymous=True)
    global movement_direction

    # Constants
    RAMP_RATE = 0.1
    MAX_LINEAR_SPEED = 0.25
    MAX_LINEAR_DISTANCE = 10.0

    # Parameters
    linear_distance = rospy.get_param('~linear_distance', 0.0)
    linear_speed = rospy.get_param('~linear_speed', 0.2)

    linear_distance = clamp(linear_distance, -MAX_LINEAR_DISTANCE, MAX_LINEAR_DISTANCE)
    linear_speed = clamp(linear_speed, 0.01, MAX_LINEAR_SPEED)

    rospy.loginfo(f"Linear motion: distance={linear_distance}m, speed={linear_speed}m/s")

    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/scan', LaserScan, laser_callback)
    rospy.sleep(1)

    rate = rospy.Rate(10)
    move_cmd = Twist()
    current_speed = 0.0

    movement_direction = 1 if linear_distance > 0 else -1
    abs_distance = abs(linear_distance)
    abs_speed = abs(linear_speed)
    linear_time = abs_distance / abs_speed
    accel_time = abs_speed / RAMP_RATE
    total_time = linear_time + accel_time

    start_time = rospy.Time.now()
    while not rospy.is_shutdown():
        elapsed_time = (rospy.Time.now() - start_time).to_sec()
        dt = rate.sleep_dur.to_sec()

        if elapsed_time >= total_time or obstacle_detected:
            break

        if elapsed_time <= linear_time:
            current_speed = ramp_speed(abs_speed * movement_direction, current_speed, RAMP_RATE, dt)
        else:
            current_speed = decelerate_speed(current_speed, RAMP_RATE, dt)

        move_cmd.linear.x = current_speed
        move_cmd.angular.z = 0
        cmd_vel_pub.publish(move_cmd)
        rate.sleep()

    move_cmd.linear.x = 0
    cmd_vel_pub.publish(move_cmd)

    movement_direction = 0
    if obstacle_detected:
        rospy.logwarn("Obstacle detected! Stopped linear movement.")
    rospy.loginfo("Linear motion complete.")

if __name__ == '__main__':
    try:
        move_linear()
    except rospy.ROSInterruptException:
        pass
