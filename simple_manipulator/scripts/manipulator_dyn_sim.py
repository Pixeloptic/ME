#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float32
from sensor_msgs.msg import JointState

# Dynamic Parameters
time_step = 0.01  # Sample time

# Define mass and physical parameters
mass1 = 3.0
mass2 = 3.0
mass_base1 = 1.5
mass_base2 = 1.5
arm_length = 0.2
offset = 0.2
link1 = 0.4
link2 = 0.4

# Set initial conditions for the joints
theta1 = 0.10
theta2 = 0.1
theta1_dot = 0.0
theta2_dot = 0.0
torque1 = 0.0
torque2 = 0.0

# Define gravitational acceleration
gravity = 9.8

# Callbacks for receiving torque updates
def callback_torque_x(msg):
    global torque1
    torque1 = msg.data

def callback_torque_y(msg):
    global torque2
    torque2 = msg.data

# Function to normalize angle within [-pi, pi]


if __name__ == '__main__':
    rospy.init_node("manipulator_dynamics_simulator")
    subscriber_x = rospy.Subscriber('/tau_x', Float32, callback_torque_x)
    subscriber_y = rospy.Subscriber('/tau_y', Float32, callback_torque_y)
    publisher_joint_states = rospy.Publisher('/joint_states', JointState, queue_size=10)
    rate = rospy.Rate(rospy.get_param("~update_rate", 100))

    print("Simulator is active...")
    try:
        while not rospy.is_shutdown():
            if time_step == 0.01:
                mmM = np.array([
                    [mass1 * arm_length**2 + mass_base1 * link1**2 + mass2 * (link1**2 + offset**2 + 2 * link1 * offset * np.cos(theta2)) + mass_base2 * (link1**2 + link2**2 + 2 * link1 * link2 * np.cos(theta2)),
                     mass2 * offset * (link1 * np.cos(theta2) + offset) + mass_base2 * link2 * (link1 * np.cos(theta2) + link2)],
                    [mass2 * offset * (link1 * np.cos(theta2) + offset) + mass_base2 * link2 * (link1 * np.cos(theta2) + link2),
                     mass2 * offset**2 + mass_base2 * link2**2]
                ])

                mmC = np.array([
                    [-2 * link1 * np.sin(theta2) * (mass2 * offset + mass_base2 * link2) * theta2_dot,
                     -link1 * np.sin(theta2) * (mass2 * offset + mass_base2 * link2) * theta2_dot],
                    [link1 * np.sin(theta2) * (mass2 * offset + mass_base2 * link2) * theta1_dot, 0]
                ])

                mmG = np.array([
                    mass1 * arm_length * np.cos(theta1) + mass_base1 * link1 * np.cos(theta1) + mass2 * (link1 * np.cos(theta1) + offset * np.cos(theta1 + theta2)) + mass_base2 * (link1 * np.cos(theta1) + link2 * np.cos(theta1 + theta2)),
                    np.cos(theta1 + theta2) * (mass2 * offset + mass_base2 * link2)
                ]) * gravity

                net_torque = np.array([torque1, torque2]) - np.dot(mmC, [theta1_dot, theta2_dot]) - mmG
                mmTheta_dot_dot = np.linalg.inv(mmM).dot(net_torque)

                theta1_dot += mmTheta_dot_dot[0] * time_step
                theta2_dot += mmTheta_dot_dot[1] * time_step

                theta1 += theta1_dot * time_step
                theta2 += theta2_dot * time_step

        

                joint_state = JointState()
                joint_state.header.stamp = rospy.Time.now()
                joint_state.name = ["joint2", "joint3"]
                joint_state.position = [theta1, theta2]
                joint_state.velocity = [theta1_dot, theta2_dot]
                publisher_joint_states.publish(joint_state)

            rate.sleep()

    except rospy.ROSInterruptException:
        pass  # Gracefully handle shutdown
