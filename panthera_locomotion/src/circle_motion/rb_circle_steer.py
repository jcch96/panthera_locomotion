#!/usr/bin/env python

'''
Things to note for each motor node:
1. Address #
2. Port /dev/ttyUSB* if using multiple USB ports
3. PID parameters
4. Subscribing to correct twist message for both subscribers
5. Node name
'''

import rospy
import time
import orienbus
import serial.tools.list_ports
import math

from std_msgs.msg import Int64
from geometry_msgs.msg import Twist
#from panthera_locomotion.srv import Status, StatusResponse

# PID parameters
kp = 1
ki = 0
kd = 0

MAX_SPEED = 400
motor_speed = 0

######################
address = 12
sn = rospy.get_param('/rb_serial_number')
p = list(serial.tools.list_ports.grep(sn))
port = '/dev/' + p[0].name
######################


target = 0     # target motor position
position = 0   # current motor position

tolerance = 1.0#rospy.get_param('/angle_tolerance')

# Errors for pid controller
current_error = 0
prev_error = 0
accu_error = 0
integral_reset = 75#rospy.get_param('/rb_pid')['ir']

orienbus = orienbus.OrienBus(port)
motor = orienbus.initialize(address)


### Subscriber Functions ###
######################################
# Reading actual encoder position
def encoder_pos(data):
    global position, current_time
    position = data.linear.y # take note using correct Twist msg data
    #time = rospy.get_rostime()
    #current_time = time.secs + time.nsecs*(10**(-9))

# Reading target position
def desired_speed(data):
    global target
    if abs(data.data) < 80:
        target = 0
    else:
        target = data.data * 2
     #take note using correct Twist msg data
######################################

### PID Controller Functions ###
######################################
def proportional(desired, actual): #error - current angle
    prop =  kp * (desired - actual)
    #print("Kp: " + str(prop))
    return prop

def derivative(curr, prev, dt): #d angle-error/ dt
    #dt = 0.05 #current_time - prev_time
    #print("dt is: " + str(dt))
    if dt == 0:
        deriv = 0
    else:
        deriv = kd * (curr - prev) / dt
    #print("Kd: "+ str(deriv))
    return deriv

def integral(accu, dt):
    integral =  ki * accu * dt
    #print("Ki: " +str(integral))
    return integral

def control_pid(error, s):
    ks = 1.1  
    ka = 0.5
    output = ks * s * math.tanh(ka * abs(error) + 1)
    return output
######################################

# Adjust motor speed
def adjust_speed(dt):
    global motor_speed
    global current_error
    global prev_error
    global accu_error
    global prev_time
    global integral_reset
    current_error = target - motor_speed
    #print("current error is:" + str(current_error))
    accu_error += current_error
    p = proportional(target, motor_speed)
    d = derivative(current_error, prev_error, dt)
    i = integral(accu_error, dt)

    speed = p + i + d + motor_speed
    speed =  int(speed)
    #print("input speed is: " + str(speed))
    #if abs(current_error) >= tolerance:
    if abs(speed) < abs(MAX_SPEED):
        motor_speed = speed
        #motor.writeSpeed(motor_speed)
        #print("Speed: "+str(speed))
    else:
        if speed < 0:
            motor_speed = -MAX_SPEED
            #motor.writeSpeed(motor_speed)
            #print("Speed: "+str(-MAX_SPEED))
        else:
            motor_speed = MAX_SPEED
            #motor.writeSpeed(motor_speed)
            #print("Speed: "+str(MAX_SPEED))
    #else:
        #motor_speed = 0
        #motor.writeSpeed(0)
    motor.writeSpeed(motor_speed)
    print("rb speed: " + str(motor_speed))
    print("current error: " + str(current_error))
    prev_error = current_error
    #prev_time = current_time
    #print("accu_error: " + str(accu_error))
    
    if abs(i) >= integral_reset:
    	accu_error = 0

if __name__ == "__main__":
    try:
        rospy.init_node("rb_steering_motor") # Note correct node name
        rospy.Subscriber("encoder_positions", Twist, encoder_pos)
        rospy.Subscriber("lb_steer_speed", Int64, desired_speed)
        #rospy.on_shutdown(kill_procedure)
        period = 0.05
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
        #while True:
            if abs(position) >= 95:
                motor.writeSpeed(0)
                break
            else:
                start = rospy.get_time()
                adjust_speed(period)
                #print('adjusting speed')
                rate.sleep()
                end = rospy.get_time()
            #print(rospy.is_shutdown())
        #motor.writeSpeed(0)
    except rospy.ROSInterruptException:
        #motor.writeSpeed(0)
        print("stopping motor")