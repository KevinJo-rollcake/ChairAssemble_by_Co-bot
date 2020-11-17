# -*- coding: utf-8 -*-
#!/usr/bin/env python

#파트를 넘겨받아 자신의 워크스페이스에 정리

from UR_serial_scripts import *
import copy

worker_park = UR5_manipulator(IP="192.168.10.111", PORT=30002, name="Park")
time.sleep(0.05)
default_speed = 0.2

taking_part_pose 	= [-0.05,-0.748,0.5,d2r(69.4),d2r(-69.3),d2r(69.3)]
part_place_1		= [-0.065, -0.413,0.02,0,-3.14,0]
part_place_2		= [-0.435, -0.413,0.02,0,-3.14,0]
part_place_3_1		= []
part_place_3_2		= []
work_place_1		= [-0.065, -0.6, 0.006, d2r(127), d2r(127), 0]
work_place_2		= [0.096,-0.67,0.006,d2r(180),0,0]

def park_ready_take_off_part():
	goal_pose = copy.deepcopy(taking_part_pose)
	goal_pose[1] += 0.1
	worker_park.movej("p"+repr(goal_pose), t=5) #Park씨는 파트를 넘겨 받을 준비가 되었다.
	goal_pose[2] += 0.09
	worker_park.movej("p"+repr(goal_pose))

def park_take_off_part():
	if robotiq_gripper_park.param != 0:
		worker_park.gripper_grasping(0, robotiq_gripper_park)
	goal_pose = copy.deepcopy(worker_park.pose)
	goal_pose[1] -= 0.1
	worker_park.movel("p"+repr(goal_pose),v=0.03)
	worker_park.gripper_grasping(100, robotiq_gripper_park)
	worker_park.force_mode("[0, 1, 0, 0, 0, 0]", "[0, -2.5, 0, 0, 0, 0]", "[0.1,3,0.1,0.17,0.17,0.17]",t=6)
	worker_park.gripper_grasping(255, robotiq_gripper_park) #파트를 넘겨 받았다.

def park_placing_part(part_place):
	goal_pose = copy.deepcopy(worker_park.pose)
	goal_pose[1] += 0.1
	worker_park.movel("p"+repr(goal_pose),v=0.1)
	worker_park.go_ready_pose()
	goal_pose = copy.deepcopy(part_place)
	goal_pose[2] += 0.3
	worker_park.movel("p"+repr(goal_pose),v=default_speed)
	worker_park.movel("p"+repr(part_place),v=0.1)
	worker_park.force_mode("[0,0,1,0,0,0]","[0,0,10,0,0,0]","[0.1,0.1,0.04,0.17,0.17,0.17]",t=2.5)
	worker_park.gripper_grasping(0, robotiq_gripper_park)
	worker_park.gripper_grasping(255, robotiq_gripper_park)
	worker_park.gripper_grasping(0, robotiq_gripper_park)
	goal_pose = copy.deepcopy(worker_park.pose)
	goal_pose[2] += 0.1
	worker_park.movel("p"+repr(goal_pose),v=default_speed)

def park_make_part1pose_for_part3_1():
	if robotiq_gripper_park.param != 0:
		worker_park.gripper_grasping(0, robotiq_gripper_park)
	goal_pose = copy.deepcopy(part_place_1)
	goal_pose[1] += 0.01
	goal_pose[2] += 0.1
	worker_park.movel("p"+repr(goal_pose),v=default_speed)
	goal_pose[2] -= 0.115
	worker_park.movel("p"+repr(goal_pose),v=default_speed)
	worker_park.gripper_grasping(100, robotiq_gripper_park)
	worker_park.force_mode("[0,1,0,0,0,0]","[0,-2,0,0,0,0]","[0.1,0.06,0.1,0.17,0.17,0.17]",t=2)
	worker_park.gripper_grasping(255, robotiq_gripper_park)
	#catched part1
	worker_park.movel_add("[0, 0, 0.006, 0, 0, 0]",t=0.5)
	worker_park.movel("p"+repr(work_place_1),t=3)
	worker_park.force_mode("[0,0,1,0,0,0]","[0,0,4,0,0,0]","[0.1,0.1,0.1,0.17,0.17,0.17]",t=1)

def park_rotate_part1_90deg_in_WKspace():
	worker_park.gripper_grasping(0, robotiq_gripper_park)
	goal_pose = copy.deepcopy(work_place_1)
	goal_pose[2] += 0.1
	worker_park.movel("p"+repr(goal_pose), v=default_speed)
	goal_pose = copy.deepcopy(work_place_2)
	goal_pose[2] += 0.1
	worker_park.movel("p"+repr(goal_pose), v=default_speed)
	worker_park.movel("p"+repr(work_place_2), v=default_speed)
	worker_park.gripper_grasping(100,robotiq_gripper_park)
	worker_park.force_mode("[0,1,0,0,0,0]","[0,-2,0,0,0,0]","[0.1,0.08,0.1,0.17,0.17,0.17]",t=4)
	worker_park.gripper_grasping(255,robotiq_gripper_park)
	#rotate start
	worker_park.movel_add("[0, 0, 0.006, 0, 0, 0]",t=0.5)
	goal_pose = copy.deepcopy(work_place_1)
	goal_pose[2] += 0.006
	worker_park.movel("p"+repr(goal_pose),v=0.1)
	time.sleep(1)
	worker_park.force_mode("[0,0,1,0,0,0]","[0,0,4,0,0,0]","[0.1,0.1,0.1,0.17,0.17,0.17]",t=1)




#int seting=================================================================================

robotiq_gripper_park = gripper("127.0.0.1", "63352", "gripper_socket")
worker_park.initial_setting_for_gripper(robotiq_gripper_park)
worker_park.go_zero_pose()
worker_park.go_ready_pose()
worker_park.initial_setting_for_gripper(robotiq_gripper_park, speed=512, force=100)
worker_park.set_tcp("p[0.,0.,0.165,0.,0.,0.]")

#int main=================================================================================
park_ready_take_off_part()

#Kim's motion
input("go next?: ")
time.sleep(1)

park_take_off_part()


#Kim's motion
input("go next?: ")
time.sleep(1)

park_placing_part(part_place_1)

worker_park.go_ready_pose()
park_ready_take_off_part()

#Kim's motion
input("go next?: ")
time.sleep(1)

park_take_off_part()


#Kim's motion
input("go next?: ")
time.sleep(1)

park_placing_part(part_place_2)

# part1 part2 placing end

park_make_part1pose_for_part3_1()
#Kim's motion
input("go next?: ")
time.sleep(1)
park_rotate_part1_90deg_in_WKspace()