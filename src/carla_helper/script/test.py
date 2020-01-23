#!/usr/bin/env python
import glob
import os
import sys

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla

import argparse
import logging
import time
import random

def main():

	client = carla.Client("127.0.0.1", 2000)
	client.set_timeout(2.0)
	world = client.get_world()

	# get ego car
	for carla_actor in world.get_actors():
		if carla_actor.type_id.startswith("vehicle"):
			if carla_actor.attributes.get('role_name') == 'ego_vehicle':
				ego_vehicle = carla_actor
		
	batch = []
	batch.append(carla.command.ApplyVelocity(ego_vehicle, carla.Vector3D(-100.0,-10.0,100.0)))
	client.apply_batch(batch)
	print("accel")

	for i in range(1,10):
		print(ego_vehicle.get_velocity())
		time.sleep(0.01)

	time.sleep(1.0)
	batch.append(carla.command.ApplyVelocity(ego_vehicle, carla.Vector3D(0.0,0.0,0.0)))
	client.apply_batch(batch)
	print("brake")
	for i in range(1,10):
		print(ego_vehicle.get_velocity())
		time.sleep(0.01)

	# while True:
	# 	world.wait_for_tick()
	# 	ego_pose = ego_vehicle.get_transform()
	# 	time.sleep(0.1)


if __name__ == '__main__':

	main()
