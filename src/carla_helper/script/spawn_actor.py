#!/usr/bin/env python

import glob
import os
import sys
try:
	sys.path.append(glob.glob('**/carla-*%d.%d-%s.egg' % (
		sys.version_info.major,
		sys.version_info.minor,
		'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
	pass


# ==============================================================================
# -- imports -------------------------------------------------------------------
# ==============================================================================

import argparse
import random
import csv
import time
import carla
import logging


class SpawnActor(object):
	
	def __init__(self):
		
		self.client = None
		self.world = None
		self.ego_vehicle = None
		self.ego_pose = None
		self.blueprintVehicles = None
		self.blueprintWalkers = None
		self.actor_list = []
		self.scenario_list = []
		self.spawned_vehicle_list = []
		self.spawned_walker_list = []

	def readFile(self, filename):

		file = open(filename, 'r')
		reader = csv.reader(file)
		header = next(reader)

		in_list = [row for row in reader]
		return in_list

	def getEgoCar(self):
		
		for carla_actor in self.world.get_actors():
			if carla_actor.type_id.startswith("vehicle"):
				if carla_actor.attributes.get('role_name') == 'ego_vehicle':
					self.ego_vehicle = carla_actor


	def checkScenario(self):
		
		ego_pose = self.ego_vehicle.get_transform()
		intrusion_thres = 1.0

		for scenario in scenario_list:
			distance = (ego_pose.location.x - float(scenario[1])) ** 2 \
					   + (ego_pose.location.y - float(scenario[2])) ** 2
			if distance < intrusion_thres:
				if (scenario[4] == 'spawn'):
					spawnActor(scenario[4:])
				elif (scenario[4] == 'controll'):
					controllActor(scenario[4:])


	def spawnActor(self, actor_id_list):
		'''
		Spawn vehicles
		Spawn Walkers
		'''

    	logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.INFO)


		vehicle_batch = []
		walker_batch = []
		
		for actor_id in actor_id_list:

			try:
				index = self.actor_list[:][0].index(actor_id)

			except ValueError:
				print("actor_id {} does not registered in the actor file".format(actor_id))
				continue

			actor = self.actor_list[index]

			# spawn the walker object
			if (actor[1] == 'walker'):
				blueprint = random.choice(blueprintWalkers)

				# set as not invencible
				if blueprint.has_attribute('is_invincible'):
					blueprint.set_attribute('is_invincible', 'false')

				transform = carla.Transform(carla.Location(actor[2], actor[3], actor[4]), carla.Rotation(actor[5], actor[6], actor[7]))
				walker_batch.append(carla.command.SpawnActor(walker_bp, transform))


			# spawn the vehicle object
			elif (actor[1] == 'vehicle'):
				blueprint = random.choice(blueprintVehicles)
				
				if blueprint.has_attribute('color'):
					color = random.choice(blueprint.get_attribute('color').recommended_values)
					blueprint.set_attribute('color', color)

				if blueprint.has_attribute('driver_id'):
					driver_id = random.choice(blueprint.get_attribute('driver_id').recommended_values)
					blueprint.set_attribute('driver_id', driver_id)

				blueprint.set_attribute('role_name', 'autopilot')
				transform = carla.Transform(carla.Location(actor[2], actor[3], actor[4]), carla.Rotation(actor[5], actor[6], actor[7]))
				vehicle_batch.append(carla.command.SpawnActor(blueprint, transform).then(carla.command.SetAutopilot(carla.command.FutureActor, True)))


		results = client.apply_batch_sync(vehicle_batch)
		for response in range(len(results)):
			if response.error:
				logging.error(response.error)
			else:
				self.spawned_vehicle_list.append(response.actor_id)

		results = client.apply_batch_sync(walker_batch, True)
		for i in range(len(results)):
			if results[i].error:
				logging.error(results[i].error)
			else:
				walkers_list.append({"id": results[i].actor_id})


		# 3. we spawn the walker controller
		batch = []
		walker_controller_bp = world.get_blueprint_library().find('controller.ai.walker')
		for i in range(len(walkers_list)):
			batch.append(SpawnActor(walker_controller_bp, carla.Transform(), walkers_list[i]["id"]))
		results = client.apply_batch_sync(batch, True)
		for i in range(len(results)):
			if results[i].error:
				logging.error(results[i].error)
			else:
				walkers_list[i]["con"] = results[i].actor_id
		# 4. we put altogether the walkers and controllers id to get the objects from their id
		for i in range(len(walkers_list)):
			all_id.append(walkers_list[i]["con"])
			all_id.append(walkers_list[i]["id"])
		all_actors = world.get_actors(all_id)

		# wait for a tick to ensure client receives the last transform of the walkers we have just created
		world.wait_for_tick()



		# 5. initialize each controller and set target to walk to (list is [controler, actor, controller, actor ...])
		for i in range(0, len(all_id), 2):
			# start walker
			all_actors[i].start()
			# set walk to random point
			all_actors[i].go_to_location(world.get_random_location_from_navigation())
			# random max speed
			all_actors[i].set_max_speed(1 + random.random())    # max speed between 1 and 2 (default is 1.4 m/s)

		print('spawned %d vehicles and %d walkers, press Ctrl+C to exit.' % (len(vehicles_list), len(walkers_list)))


		while True:
			world.wait_for_tick()




	def game_loop(self, args):

		try:
	
			self.client = carla.Client(args.host, args.port)
			self.client.set_timeout(2.0)
			self.world = self.client.get_world()
			self.blueprintVehicles = self.world.get_blueprint_library().filter(args.filterv)
			self.blueprintWalkers = self.world.get_blueprint_library().filter(args.filterw)
			self.actor_list = self.readFile(args.actor_file)
			self.scenario_list = self.readFile(args.scenario_file)
			self.getEgoCar()


			while True:

				self.world.tick()
				time.sleep(0.1)

		finally:
			print('exit')
			# if self.world is not None:
				# self.world.destroy()

def main():

	argparser = argparse.ArgumentParser( description = __doc__ )
	argparser.add_argument(
		'--host',
		metavar='H',
		default='127.0.0.1',
		help='IP of the host server (default: 127.0.0.1)')
	argparser.add_argument(
		'-p', '--port',
		metavar='P',
		default=2000,
		type=int,
		help='TCP port to listen to (default: 2000)')
	argparser.add_argument(
		'-a', '--actor_file',
		metavar='A',
		default='~/share/catkin_ws/src/carla_helper/actor.csv',
		help='scenario file (default: actor.csv)')
	argparser.add_argument(
		'-s', '--scenario_file',
		metavar='S',
		default='~/share/catkin_ws/src/carla_helper/scenario.csv',
		help='scenario file (default: scenario.csv)')
	argparser.add_argument(
		'--filterv',
		metavar='PATTERN',
		default='vehicle.*',
		help='vehicles filter (default: "vehicle.*")')
	argparser.add_argument(
		'--filterw',
		metavar='PATTERN',
		default='walker.pedestrian.*',
		help='pedestrians filter (default: "walker.pedestrian.*")')
	args = argparser.parse_args()

	
	try:
		spawn_actor = SpawnActor()
		spawn_actor.game_loop(args)
	finally:
		print('EXIT')


if __name__ == '__main__':
	main()