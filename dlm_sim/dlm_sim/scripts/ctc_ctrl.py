#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import JointState

# Parámetros del modelo
m1, m2, l1, l2, a, d, M1, M2, g = 1.0, 1.0, 1.0, 1.0, 0.5, 0.5, 0.1, 0.1, 9.81

# Variables globales para el estado del sistema
system_states = JointState()
flag = False

# Función callback para actualizar el estado del sistema
def callback(data):
    global system_states, flag
    flag = True
    system_states = data

if __name__ == '__main__':
    # Inicialización del nodo ROS
    rospy.init_node("CTC_ctrl")
    rospy.Subscriber("joint_states", JointState, callback)
    controlInput = rospy.Publisher("dlm_input", Float32MultiArray, queue_size=1)
    loop_rate = rospy.Rate(200)  # 200 Hz

    # Bucle principal
    while not rospy.is_shutdown():
        if flag:
            # Obtener tiempo actual y estados del sistema
            t = rospy.get_time()
            q1, q2 = system_states.position[:2]
            q1_punto, q2_punto = system_states.velocity[:2]

            # Puntos de ajuste cambiando con el tiempo
            r1 = 2 + 0.3 * np.sin(t)
            r2 = 1 + 0.7 * np.sin(t)

            # Ganancias del controlador PD
            Kp1, Kp2, Kd1, Kd2 = 100, 100, 20, 20

            # Cálculo de errores y errores de velocidad
            e1, e2 = r1 - q1, r2 - q2
            e_punto1, e_punto2 = -q1_punto, -q2_punto

            # Construcción de matrices de dinámica del sistema
            M = np.array([[m1 * a**2 + M1 + m2 * (l1**2 + d**2 + 2 * l1 * d * np.cos(q2)) + M2,
                           m2 * (d**2 + l1 * d * np.cos(q2)) + M2],
                          [m2 * (d**2 + l1 * d * np.cos(q2)) + M2,
                           m2 * d**2 + M2]])

            C = np.array([[-m2 * l1 * d * np.sin(q2) * q2_punto,
                           -m2 * l1 * d * np.sin(q2) * (q1_punto + q2_punto)],
                          [m2 * l1 * d * np.sin(q2) * q1_punto, 0]])

            G = np.array([[m1 * g * a * np.cos(q1) + m2 * g * (l1 * np.cos(q1) + d * np.cos(q1 + q2))],
                          [m2 * g * d * np.cos(q1 + q2)]])

            # Control de torque computado
            Kp = np.array([Kp1, Kp2])
            Kd = np.array([Kd1, Kd2])
            error = np.array([e1, e2])
            error_punto = np.array([e_punto1, e_punto2])
            ut= np.dot(M, Kp * error + Kd * error_punto) + np.dot(C, np.array([q1_punto, q2_punto])) + G.flatten()

            # Preparar y enviar el mensaje de control
            msg = Float32MultiArray(data=tt.tolist())
            controlInput.publish(msg)

            flag = False
        loop_rate.sleep()
