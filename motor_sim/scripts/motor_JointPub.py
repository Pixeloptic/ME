#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32

def wrap_to_pi(theta):
    """
    Normalize an angle to the range [-pi, pi).
    """
    normalized_theta = (theta + np.pi) % (2 * np.pi)
    if normalized_theta < 0:
        normalized_theta += 2 * np.pi
    return normalized_theta - np.pi

def stop():
    """
    Callback function to run when the node is shutting down.
    Publishes the last known position of the joint.
    """
    rospy.loginfo("Stopping the motor joint publisher")
    cont_joints.header.stamp = rospy.Time.now()
    cont_joints.position[0] = wrap_to_pi(angular_position)
    joint_pub.publish(cont_joints)

def motor_output_callback(msg):
    """
    Callback function for motor output subscriber.
    Updates the global motor output value.
    """
    global motor_output
    motor_output = msg.data

def initialize_joints():
    """
    Initializes the joint state message.
    """
    cont_joints.header.frame_id = "joint1"
    cont_joints.name = ["joint2"]
    cont_joints.position = [0.0]
    cont_joints.velocity = [0.0]
    cont_joints.effort = [0.0]

if __name__ == '__main__':
    # Initialize the ROS node
    rospy.init_node("dc_motor_joints")

    # Get ROS parameters
    sample_time = rospy.get_param("~motor_joint_sample_time", 0.01)
    rate = rospy.get_param("~motor_joint_rate", 100)

    # Initialize variables
    angular_position = 0.0
    motor_output = 0.0
    first_iteration = True

    # Initialize messages and publishers/subscribers
    cont_joints = JointState()
    initialize_joints()
    joint_pub = rospy.Publisher("/joint_states", JointState, queue_size=1)
    rospy.Subscriber("/motor_output", Float32, motor_output_callback)

    # Set up shutdown hook
    rospy.on_shutdown(stop)

    # Logging
    rospy.loginfo("Motor joint publisher is running")

    # Main loop
    loop_rate = rospy.Rate(rate)
    try:
        while not rospy.is_shutdown():
            current_time = rospy.get_time()

            if first_iteration:
                last_time = start_time = current_time
                first_iteration = False

            dt = current_time - last_time
            if dt >= sample_time:
                angular_position += motor_output * dt
                cont_joints.header.stamp = rospy.Time.now()
                cont_joints.position[0] = wrap_to_pi(angular_position)
                last_time = current_time

                # Publish joint state
                joint_pub.publish(cont_joints)

            loop_rate.sleep()
    except rospy.ROSInterruptException:
        pass

