#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

def clamp(value, min_value, max_value):
    """
    Clamps a given value between min_value and max_value.
    """
    return max(min(value, max_value), min_value)

def ramp_speed(target_speed, current_speed, ramp_rate, dt):
    """
    Gradually ramps up the speed to avoid sudden changes.
    Args:
        target_speed (float): Target speed to reach.
        current_speed (float): Current speed of the robot.
        ramp_rate (float): Rate of speed change (meters per second).
        dt (float): Time step since the last update (in seconds).
    Returns:
        float: Updated speed after ramping.
    """
    delta_speed = ramp_rate * dt
    if current_speed < target_speed:
        current_speed = min(current_speed + delta_speed, target_speed)
    elif current_speed > target_speed:
        current_speed = max(current_speed - delta_speed, target_speed)
    return current_speed

def decelerate_speed(current_speed, ramp_rate, dt):
    """
    Gradually decelerates the speed to zero.
    Args:
        current_speed (float): Current speed of the robot.
        ramp_rate (float): Rate of speed change (meters per second).
        dt (float): Time step since the last update (in seconds).
    Returns:
        float: Updated speed after deceleration.
    """
    delta_speed = ramp_rate * dt
    if current_speed > 0:
        current_speed = max(current_speed - delta_speed, 0)
    elif current_speed < 0:
        current_speed = min(current_speed + delta_speed, 0)
    return current_speed

def move_robot():
    """
    Moves the robot based on ROS parameters, with fixed angular speed. Only one type of movement is executed at a time, if provided (non-zero)parameters for both types - only rotation will be executed.
    Parameters:
        ~linear_distance (float): Distance to move in meters (positive = forward, negative = backward).
        ~angular_angle (float): Rotation in degrees (positive = counterclockwise, negative = clockwise).
        ~linear_speed (float): Speed in meters per second (must be positive).
    """
    rospy.init_node('move_robot', anonymous=True)

    # Constants
    RAMP_RATE = 0.1            # m/s
    ANGULAR_SPEED_RAD = 0.5    # rad
    MAX_LINEAR_SPEED = 0.25    # m/s
    MAX_LINEAR_DISTANCE = 10.0 # m
    MAX_ANGULAR_ANGLE = 720.0  # deg
    
    # Parameters
    linear_distance = rospy.get_param('~linear_distance', 0.0)
    angular_angle = rospy.get_param('~angular_angle', 0.0)
    linear_speed = rospy.get_param('~linear_speed', 0.2)
    
    linear_distance = clamp(linear_distance, -MAX_LINEAR_DISTANCE, MAX_LINEAR_DISTANCE)
    angular_angle = clamp(angular_angle, -MAX_ANGULAR_ANGLE, MAX_ANGULAR_ANGLE)
    linear_speed = clamp(linear_speed, 0.01, MAX_LINEAR_SPEED)
	
    if (abs(linear_distance) > 0 or linear_speed > 0) and abs(angular_angle) > 0:
        rospy.logwarn("Both linear and angular movement were requested. Prioritizing rotation.")
        linear_distance = 0
        linear_speed = 0
	
    rospy.loginfo(f"Executing movement: distance={linear_distance}m, speed={linear_speed}m/s, angle={angular_angle}°")

    # Calculate times
    linear_time = abs(linear_distance) / abs(linear_speed) if linear_distance != 0 else 0
    angular_time = abs(angular_angle) / (ANGULAR_SPEED_RAD * (180 / 3.14159265))
    linear_accel_time = abs(linear_speed) / RAMP_RATE
    max_linear_time = linear_time + linear_accel_time

    # Publisher for /cmd_vel
    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)

    move_cmd = Twist()
    current_linear_speed = 0.0

    # Linear Motion
    start_time = rospy.Time.now()
    while not rospy.is_shutdown():
        elapsed_time = (rospy.Time.now() - start_time).to_sec()
        if elapsed_time >= max_linear_time:
            break

        dt = rate.sleep_dur.to_sec()

        if elapsed_time <= linear_time:
            current_linear_speed = ramp_speed(
                target_speed=linear_speed * (1 if linear_distance > 0 else -1),
                current_speed=current_linear_speed,
                ramp_rate=RAMP_RATE,
                dt=dt,
            )
        else:
            current_linear_speed = decelerate_speed(
                current_speed=current_linear_speed,
                ramp_rate=RAMP_RATE,
                dt=dt,
            )

        move_cmd.linear.x = current_linear_speed
        move_cmd.angular.z = 0
        cmd_vel_pub.publish(move_cmd)
        rate.sleep()

    move_cmd.linear.x = 0
    cmd_vel_pub.publish(move_cmd)

    # Rotation
    start_time = rospy.Time.now()
    while not rospy.is_shutdown():
        elapsed_time = (rospy.Time.now() - start_time).to_sec()
        if elapsed_time >= angular_time:
            break

        move_cmd.linear.x = 0
        move_cmd.angular.z = ANGULAR_SPEED_RAD * (1 if angular_angle > 0 else -1)
        cmd_vel_pub.publish(move_cmd)
        rate.sleep()

    # Stop rotation
    move_cmd.angular.z = 0
    cmd_vel_pub.publish(move_cmd)

    rospy.loginfo("Motion complete.")

if __name__ == '__main__':
    try:
        move_robot()
    except rospy.ROSInterruptException:
        rospy.loginfo("Node terminated.")
