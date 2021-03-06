#!/usr/bin/env python

import math
import rospy
from geometry_msgs.msg import Twist, Point32
from std_msgs.msg import Empty
from panthera_locomotion.srv import Status, StatusRequest, StatusResponse
from ds4_driver.msg import Status as st

class Ds4Controller():
	def __init__(self):
		rospy.init_node('Controller')

		rospy.Subscriber('/status', st, self.ds4_sub)
		#rospy.Subscriber('/can_encoder', Twist, self.encoder_pos)

		self.pub = rospy.Publisher('/panthera_cmd', Twist, queue_size=1)
		self.recon = rospy.Publisher('/reconfig', Twist, queue_size=1)

		'''
		rospy.wait_for_service('/lb_steer_status')
		rospy.wait_for_service('/lf_steer_status')
		rospy.wait_for_service('/rb_steer_status')
		rospy.wait_for_service('/rf_steer_status')

		self.lb_status = rospy.ServiceProxy('lb_steer_status', Status)
		self.lf_status = rospy.ServiceProxy('lf_steer_status', Status)
		self.rb_status = rospy.ServiceProxy('rb_steer_status', Status)
		self.rf_status = rospy.ServiceProxy('rf_steer_status', Status)
		
		rospy.wait_for_service('/lb_reconfig_status')
		rospy.wait_for_service('/lf_reconfig_status')
		rospy.wait_for_service('/rb_reconfig_status')
		rospy.wait_for_service('/rf_reconfig_status')

		self.lb_stat = rospy.ServiceProxy('lb_reconfig_status', Status)
		self.lf_stat = rospy.ServiceProxy('lf_reconfig_status', Status)
		self.rb_stat = rospy.ServiceProxy('rb_reconfig_status', Status)
		self.rf_stat = rospy.ServiceProxy('rf_reconfig_status', Status)
		'''
		self.width = 0.6
		self.length = 1.31

		self.forward = 0
		self.rot_right = 0
		self.rot_left = 0
		self.reverse = 0

		self.stop = 0

		self.forward_right = 0
		self.forward_left = 0
		self.reverse_right = 0
		self.reverse_left = 0

		self.holo_right = 0
		self.holo_left = 0

		self.d_vx = 0
		self.d_wz = 0
		self.decrease = 0

		self.mode = 1 # Mode 0:= reconfiguration, Mode 1:= movement steering

		#### Initial VX, WZ and reconfig speed ####
		self.vx = 0.045
		self.wz = 0.02618
		self.reconfig_vel = 0.05

		self.step = 0.01 # Step to adjust speed
		###########################################

		self.twist = Twist()

		#self.run()

	def encoder_pos(self,data):
		self.width = (data.angular.y + data.angular.z)/2

	def ds4_sub(self,data):
		self.forward = data.button_dpad_up * (not (data.button_dpad_right or data.button_dpad_left))
		self.rot_right = data.button_dpad_right * (not (data.button_dpad_up or data.button_dpad_down))
		self.rot_left = data.button_dpad_left * (not (data.button_dpad_up or data.button_dpad_down))
		self.reverse = data.button_dpad_down * (not (data.button_dpad_right or data.button_dpad_left))

		self.forward_right = data.button_dpad_up and data.button_dpad_right
		self.forward_left = data.button_dpad_up and data.button_dpad_left
		self.reverse_right = data.button_dpad_down and data.button_dpad_right
		self.reverse_left = data.button_dpad_down and  data.button_dpad_left

		self.holo_right = data.button_r1
		self.holo_left = data.button_l1

		self.mode = (self.mode + data.button_options) % 2
		#self.reconfig(not self.mode)

		self.d_vx = data.button_triangle# and (not data.button_share)
		self.d_wz = data.button_cross# and (not data.button_share)
		self.decrease = -data.button_share

		self.stop = data.button_square
		'''
		print("forward: " + str(self.forward))
		print("rot_right: " + str(self.rot_right))
		print("rot_left: " + str(self.rot_left))
		print("reverse: " + str(self.reverse))
		'''
		#print(self.d_vx)

	def change_vx(self):
		if self.d_vx == 0:
			pass
		else:
			while self.d_vx == 1:
				pass
			self.vx += (1*(not self.decrease) + self.decrease) * self.step

	def change_wz(self):
		if self.d_wz == 0:
			pass
		else:
			while self.d_wz == 1:
				pass
			self.wz += (1*(not self.decrease) + self.decrease) * self.step

	def check(self):
		if self.mode == 0:
			req = StatusRequest()
			req.reconfig = True
			signal = False
			rate = rospy.Rate(2)
			while signal == False and not rospy.is_shutdown():
				rate.sleep()
				lb = self.lb_status(req)
				rb = self.rb_status(req)
				lf = self.lf_status(req)
				rf = self.rf_status(req)
				signal = (lb.status and rb.status and lf.status and rf.status)
				print([lb.status, rb.status, lf.status, rf.status])
				print("Status of steering motors:" + str(signal))
		else:
			pass

	def reconfig(self, state):
		req = StatusRequest()
		req.reconfig = state
		stat = not state
		while stat!= state:
			lb = self.lb_stat(req)
			lf = self.lf_stat(req)
			rb = self.rb_stat(req)
			rf = self.rf_stat(req)
			stat = (lb.status and rb.status and lf.status and rf.status)

	def adjust_wheels(self, vx, wz): # radius in m, direction c(-1) or ccw(1)
		if wz == 0:
			radius = float('inf')
		else:
			radius = vx/wz
		left = radius - self.width/2
		right = radius + self.width/2
		
		lf = round(math.degrees(math.atan((self.length*0.5) / left)), 2)
		rf = round(math.degrees(math.atan((self.length*0.5) / right)), 2)
		lb = -lf
		rb = -rf
		return (lb,rb,lf,rf)

	def locomotion(self):
		self.change_wz()
		self.change_vx()
		f = (self.forward - self.reverse - self.reverse_left - self.reverse_right + self.forward_right + self.forward_left) * self.vx
		s = (-self.rot_right + self.rot_left - self.reverse_left + self.reverse_right - self.forward_right + self.forward_left) * self.wz
		h_r = self.holo_right * 90
		h_l = -self.holo_left * 90
		lb,rb,lf,rf = self.adjust_wheels(f, s)
		self.twist.linear.x = lb + h_r + h_l
		self.twist.linear.y = rb + h_r + h_l
		self.twist.linear.z = lf + h_r + h_l
		self.twist.angular.x = rf + h_r + h_l
		self.twist.angular.y = 0
		self.twist.angular.z = 0
		self.pub.publish(self.twist)

		#self.check()

		self.twist.angular.y = f
		self.twist.angular.z = s
		self.pub.publish(self.twist)
		#print(self.twist)
		#print('VX: ' + str(self.vx))
		#print('WZ: ' + str(self.wz))

	def run(self):
		rate = rospy.Rate(10)
		while not rospy.is_shutdown():
			self.locomotion()
			self.print_instructions()
			rate.sleep()

	def print_instructions(self):
		print('\n')
		print("    MOVEMENT    ")
		print("    --------    ")
		print("[up]: Forward")
		print("[right]: Rotate Right")
		print("[left]: Rotate Left")
		print("[down]: Reverse" + '\n')

		print("    HOLONOMIC MOVEMENT    ")
		print("    ------------------    ")
		print("[r1] + [up]: Holonomic Right")
		print("[l1] + [up]: Holonomic Left" + '\n')

		print("    ADJUST SPEED    ")
		print("    ------------    ")
		print("[up] + [right/left]: Turn Right/Left")
		print("[down] + [right/left]: Reverse Right/Left")
		print("[Triangle/Cross]: + VX/WZ")
		print("[share] + [triangle/cross]: - VX/WZ" + '\n')

		print("    Current Velocity: ")
		print("    -----------------")
		print("VX: " + str(self.vx))
		print("WZ: " + str(self.wz))
		print("Turning Radius: " + str(round(self.vx/self.wz,2)))

if __name__ == "__main__":
	start = Ds4Controller()
	#start.print_instructions()
	while not rospy.is_shutdown():
		start.run()