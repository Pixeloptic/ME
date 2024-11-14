#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32
import numpy as np

def generate_sin_wave(amplitude, frequency, time, phase=0):
    """ Generate sinusoidal values based on amplitude, frequency, and current time. """
    return amplitude * np.sin(2 * np.pi * frequency * time + phase)

if __name__ == '__main__':
    try:
        # Initialize the ROS node
        rospy.init_node("set_point_generator")

        # Publisher for publishing set point values
        pub = rospy.Publisher('/set_point', Float32, queue_size=10)

        # Configure the Node: set the loop rate
        rate = rospy.Rate(200)  # 200 Hz

        # Parameters for the sinusoidal signal
        amplitude = 1.0
        frequency = 0.5  # Frequency in Hz
        phase = np.pi / 2  # Initial phase to start from positive peak

        # Main loop: generate and publish set points
        while not rospy.is_shutdown():
            current_time = rospy.Time.now().to_sec()  # Current time in seconds
            set_point = generate_sin_wave(amplitude, frequency, current_time, phase)

            # Ensure set_point is non-negative
            set_point = max(set_point, 0.0)

            # Publish the set point
            pub.publish(Float32(set_point))

            # Sleep for the remainder of the loop rate
            rate.sleep()

    except rospy.ROSInterruptException:
        rospy.logerr("ROSInterruptException occurred, shutting down set_point_generator node.")
