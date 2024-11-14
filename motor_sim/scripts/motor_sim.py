import rospy
import numpy as np
from std_msgs.msg import Float32

# Setup Variables to be used
last_time = 0
motor_position = 0.0
angular_position = 0.0
#Motor Parameters
R = 6.0
L = 0.3
k1 = 0.04
k2 = k1
J = 0.00008
b = 0.00025
m = 0.0


#Initial conditions
omega = 0.0
current = 0.0

# Declare the input Message
motorInput = Float32()

# Declare the  process output message
motorOutput = Float32()

#Callback
def input_callback(msg):
    global motorInput
    motorInput = msg

 #Stop Condition
def stop():
  #Setup the stop message (can be the same as the control message)
   print("Simulation Stoped")
if __name__=='__main__':
    #Initialise and Setup node
    rospy.init_node("Motor_Sim")
    
    # Configure the Node
    loop_rate = rospy.Rate(rospy.get_param("/motorSimRate",200))
    rospy.on_shutdown(stop)

    # Setup the Subscribers
    rospy.Subscriber("/motor_input",Float32,input_callback)

    #Setup de publishers
    motor_pub = rospy.Publisher("/motor_output", Float32, queue_size=10)


    print("The Motor is Running")
    try:
        #Run the node
        while not rospy.is_shutdown(): 

            #########WRITE YOUR CODE HERE ####   
            current_time = rospy.get_time()
            dt = current_time - last_time
            if dt>= .01: 
              #Motor governing equations
              current += (-(R/L)*current-(k1/L)*omega+(1/L)*motorInput.data)*dt
              omega += ((k2/J)*current-(b/J)*omega-(1/J)*m)*dt

              #Message to publish
              motorOutput.data = omega

              #Publish message
              motor_pub.publish(motorOutput)
              last_time = rospy.get_time()
                        
            #Wait and repeat
            loop_rate.sleep()
    except rospy.ROSInterruptException:
        pass #Initialise and Setup node