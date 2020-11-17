# -*- coding: utf-8 -*-
#!/usr/bin/env python

import socket
import time

zero_pose = "[0,d2r(-90),0,d2r(-90),0,0]"
ready_pose = "[d2r(-90),d2r(-60),d2r(-90),d2r(-100),d2r(90),0]"

def d2r(val):
	return val * 3.1415926 / 180

def string2array(string):
	string = string.strip('[]')
	string = string.split(", ")
	a = 0
	for i in string:
		if i[:3] == 'd2r':
			i = i.strip('d2r()')
		string[a] = float(i)
		a += 1
	return string

def gripper_grasping_msg(grasp_param, gripper_name="gripper_socket", tab=1):
	msg_tab = "\t" * tab
	msg  = msg_tab + "socket_set_var(\"POS\", %d, \"%s\")\n"%(grasp_param, gripper_name) #catched chair!
	msg += msg_tab + "sync()\n"
	msg += msg_tab + "sleep(3)\n"
	return msg

class gripper:
	def __init__(self, _gripper_IP="127.0.0.1", _gripper_PORT="63352", _gripper_name="gripper_socket"):
		self.IP 	= _gripper_IP
		self.PORT 	= _gripper_PORT
		self.name 	= _gripper_name
		self.param 	= 0

class UR5_manipulator:

	def __init__(self, IP, PORT, name=0):
		self._HOST	= IP
		self._PORT 	= PORT
		self.s 		= socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		self.pose   = 0#p[] data, for movel
		self.s.connect((IP,PORT))
		if name == 0:
			self._name = IP
		else:
			self._name = name

	def go_zero_pose(self):
		self.movej(zero_pose, t=5, move="get_zero_pose")

	def go_ready_pose(self):
		self.movej(ready_pose, t=6, move="get_ready_pose")
		self.pose = [-0.133,-0.346,0.547,0,-3.034,0.81]

	def movej(self, position, a=1.4, v=1.05, t=0, r=0, move="movej"):
		tx_msg  = "def goal_pose():\n"
		tx_msg += "\tmovej(%s, a=%f, v=%f, t=%f, r=%f)\n"%(position, a,v,t,r)
		tx_msg += "end\n"

		self.s.send(tx_msg.encode('ascii'))
		print("\t%s: %s sended to %s"%(move, position, self._name))
		time.sleep(t+2)
		if position[0] == "p":
			self.pose = string2array(position.replace("p",""))

	def movel(self, position, a=1.2, v=0.25, t=0, r=0, move="movel"):
		tx_msg  = "def goal_pose():\n"
		tx_msg += "\tmovel(%s, a=%f, v=%f, t=%f, r=%f)\n"%(position, a,v,t,r)
		tx_msg += "end\n"

		self.s.send(tx_msg.encode('ascii'))
		print("\t%s: %s sended to %s"%(move, position, self._name))
		if position[0] == "p":
			if t == 0:
				position_sub = string2array(position.replace("p",""))
				position_sub[0] = position_sub[0] - self.pose[0]
				position_sub[1] = position_sub[1] - self.pose[1]
				position_sub[2] = position_sub[2] - self.pose[2]
				pose_length = pow(position_sub[0],2) + pow(position_sub[1],2) + pow(position_sub[2],2)
				pose_length = pow(pose_length, 0.5)
				if pow(v,2) < pose_length:
					t = (pose_length/2 + pow(v,2)/2/a) / v
				else:
					t = pow(pose_length/a, 0.5)
				t = t * 2
			self.pose = string2array(position.replace("p",""))
		time.sleep(t+2)

	def movel_add(self, add_pose, a=1.2, v=0.25, t=0, r=0, move="move_adder"):
		tx_msg  = "def goal_pose():\n"
		tx_msg += "\tactual_tcp = get_actual_tcp_pose()\n"
		tx_msg += "\tactual_tcp = acutal_tcp + p%s"%(add_pose)
		tx_msg += "\tmovel(acutal_tcp, a=%f, v=%f, t=%f, r=%f)\n"%(a,v,t,r)
		tx_msg += "end\n"

		self.s.send(tx_msg.encode('ascii'))
		print("\t%s: %s sended to %s"%(move, add_pose, self._name))
		if t == 0:
			add_pose = string2array(add_pose)
			pose_length = pow(add_pose[0],2) + pow(add_pose[1],2) + pow(add_pose[2],2)
			pose_length = pow(pose_length, 0.5)
			if pow(v,2) < pose_length:
				t = (pose_length/2 + pow(v,2)/2/a) / v
			else:
				t = pow(pose_length/a, 0.5)
			t = t * 2
		time.sleep(t+2)

	def force_mode(self, selection_vector, wrench, limits, t):
		tx_msg  = "def set_force_M():\n"
		tx_msg += "\tforce_mode(tool_pose(), %s, %s, 2, %s)\n"%(selection_vector, wrench, limits)
		tx_msg += "\tsleep(%f)\n"%(t)
		tx_msg += "\tend_force_mode()\n"
		tx_msg += "end\n"
		self.s.send(tx_msg.encode('ascii'))
		print("set_force sended to %s"%(self._name))
		time.sleep(t+1)

	def initial_setting_for_gripper(self, gripper_data, speed=255, force=100):
		msg  = "def UR5_gripper_set():\n"
		msg += "\tsocket_open(\"%s\", %s, \"%s\")\n"%(gripper_data.IP, gripper_data.PORT, gripper_data.name)
		#set input and output voltage ranges
		msg += "\tset_analog_inputrange(0,0)\n"
		msg += "\tset_analog_inputrange(1,0)\n"
		msg += "\tset_analog_inputrange(2,0)\n"
		msg += "\tset_analog_inputrange(3,0)\n"
		msg += "\tset_analog_outputdomain(0,0)\n"
		msg += "\tset_analog_outputdomain(1,0)\n"
		msg += "\tset_tool_voltage(0)\n"
		msg += "\tset_runstate_outputs([])\n"
		#set payload, speed and force
		msg += "\tset_payload(1.1)\n"
		msg += "\tsocket_set_var(\"SPE\",%d,\"%s\")\n"%(speed, gripper_data.name)
		msg += "\tsync()\n"
		msg += "\tsocket_set_var(\"FOR\",%d,\"%s\")\n"%(force, gripper_data.name)
		msg += "\tsync()\n"
		#initialize the gripper
		msg += "\tsocket_set_var(\"ACT\", 1, \"%s\")\n"%(gripper_data.name)
		msg += "\tsync()\n"
		msg += "\tsocket_set_var(\"GTO\", 1, \"%s\")\n"%(gripper_data.name)
		msg += "\tsync()\n"
		msg += "\tsleep(0.5)\n"
		msg += gripper_grasping_msg(0, gripper_data.name, tab=1)
		gripper_data.param = 0
		msg += "\ttextmsg(\"gripper setting complete\")\n"
		msg += "end\n"
		self.s.send(msg.encode('ascii'))
		print("gripper_setting_msg sended to %s"%(self._name))
		time.sleep(4.5)

	def gripper_grasping(self, grasp_param, gripper_data):
		msg  = "def UR5_gripper_grasp():\n"
		msg += "\tsocket_open(\"%s\", %s, \"%s\")\n"%(gripper_data.IP, gripper_data.PORT, gripper_data.name)
		msg += gripper_grasping_msg(grasp_param, gripper_data.name, tab=1)
		gripper_data.param = grasp_param
		msg += "end\n"
		self.s.send(msg.encode('ascii'))
		print("gripper_grasping_msg sended to %s"%(self._name))
		time.sleep(1)

	def set_tcp(self, pose):
		msg  = "def UR5_set_tcp():\n"
		msg += "\tset_tcp(%s)\n"%(pose)
		msg += "end\n"
		self.s.send(msg.encode('ascii'))
		print("tcp set to %s"%(pose))
		time.sleep(0.1)