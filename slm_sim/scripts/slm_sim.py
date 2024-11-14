#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float32
from sensor_msgs.msg import JointState

# Declare Variables to be used
k = 0.01  # Coeficiente de fricción
m = 0.75  # Masa
l = 0.36  # Longitud
g = 9.8   # Gravedad
J = m * l ** 2 / 12  # Momento de inercia
Tau = 0.0  # Torque externo
dt = 0.01  # Intervalo de tiempo
x1 = 0.0  # Posición angular inicial
x2 = 0.0  # Velocidad angular inicial
first = True
# Define the callback functions
def tau_callback(msg):
    global Tau
    Tau = msg.data

# wrap to pi function
def wrap_to_pi(theta):
    result = np.fmod((theta + np.pi), (2 * np.pi))
    if result < 0:
        result += 2 * np.pi
    return result - np.pi

if __name__=='__main__':
    # Initialise and Setup node
    rospy.init_node("SLM_Sim")


    # Configure the Node
    loop_rate = rospy.Rate(rospy.get_param("~node_rate", 100))

    # Setup the Subscribers
    sub = rospy.Subscriber('/tau', Float32, tau_callback)

    # Setup the Publishers
    pub = rospy.Publisher('/joint_states', JointState, queue_size=10)

    print("The SLM sim is Running")
    try:
        # Run the node
        while not rospy.is_shutdown(): 
            if first == True: 
                last_time = rospy.get_time()
                current_time = rospy.get_time()
                first = False
        #System
            else:
            #Define sampling time
                current_time = rospy.get_time()
                dt = current_time - last_time 
                if dt >= .01:
            # Calculate the angular acceleration
                    x2_dot = (1 / (J + m * l ** 2)) * (-m * g * l * np.cos(x1) / 2 - k * x2 + Tau)
            
            # Update angular velocity and position
                    x2 += x2_dot * dt
                    x1 += x2 * dt
            
            # Ensure the angle x1 wraps around pi
                    x1 = wrap_to_pi(x1)
            
            # Publish the current joint state
                    joint_state = JointState()
                    joint_state.header.stamp = rospy.Time.now()
                    joint_state.name = ['single_joint']
                    joint_state.position = [x1]
                    joint_state.velocity = [x2]
                    pub.publish(joint_state) 
                    last_time = rospy.get_time()

            # Wait and repeat
            loop_rate.sleep()
    
    except rospy.ROSInterruptException:
        pass
