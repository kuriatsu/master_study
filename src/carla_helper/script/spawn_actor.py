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
		self.blueprintWalkerController = None
		self.actor_profile_list = []
		self.scenario_list = []
		self.spawning_actor_list = []

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
		intrusion_thres = 2.0

		for scenario in self.scenario_list:
			distance = (ego_pose.location.x - float(scenario[1])) ** 2 \
					   + (ego_pose.location.y - float(scenario[2])) ** 2
			if distance < intrusion_thres:
				if (scenario[4] == 'spawn'):
					self.spawnActor(scenario[5:])
					print("spawned {}".format(scenario[5:]))
				elif (scenario[4] == 'control'):
					self.controllActor(scenario[5:])
					print("control {}".format(scenario[5:]))
				elif (scenario[4] == 'destroy'):
					self.destroyActor(scenario[5:])
					print("destroy {}".format(scenario[5:]))

				self.scenario_list.remove(scenario)

	def spawnActor(self, actor_id_list):

		logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.INFO)

    	# we spawn the actor
		batch = []
		for actor_id in actor_id_list:
			# print('profile list len = {}'.format(len(self.actor_profile_list)))
			# check whether the required actor id exists in the profile list or not
			if int(actor_id) > len(self.actor_profile_list):
				print("actor_id {} does not registered in the actor file".format(actor_id))
				actor_id_list.remove(actor_id)
				continue

			# get target object from not aligned list
			actor = self.actor_profile_list[int(actor_id)]
			# add id to the list of currently existing actor under the world
			self.spawning_actor_list.append({'id':int(actor_id)})
			self.spawning_actor_list[-1]['attribute'] = actor[1]

			# spawn the walker object
			if (actor[1] == 'walker'):
				blueprint = random.choice(self.blueprintWalkers)

				# set as not invencible
				if blueprint.has_attribute('is_invincible'):
					blueprint.set_attribute('is_invincible', 'false')

				# set walker's speed
				if blueprint.has_attribute('speed'):
					self.spawning_actor_list[-1]['walk'] = blueprint.get_attribute('speed').recommended_values[1]
					self.spawning_actor_list[-1]['run'] = blueprint.get_attribute('speed').recommended_values[2]
				else:
					self.spawning_actor_list[-1]['walk'] = 3
					self.spawning_actor_list[-1]['run'] = 10

				transform = carla.Transform(carla.Location(float(actor[2]), float(actor[3]), float(actor[4])), carla.Rotation(float(actor[5]), float(actor[6]), float(actor[7])))
				batch.append(carla.command.SpawnActor(blueprint, transform))


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
				transform = carla.Transform(carla.Location(float(actor[2]), float(actor[3]), float(actor[4])), carla.Rotation(actor[5], actor[6], actor[7]))
				batch.append(carla.command.SpawnActor(blueprint, transform))

		# conduct spawn
		results = self.client.apply_batch_sync(batch)
		for i in range(len(results)):
			if results[i].error:
				print(results[i].error)
			else:
				self.spawning_actor_list[i]["world_id"] = results[i].actor_id
				print("spawned", self.spawning_actor_list[i])
		# we spawn the walker controller
		batch = []
		for spawning_actor in self.spawning_actor_list:
			if spawning_actor['attribute'] == 'walker':
				batch.append(carla.command.SpawnActor(self.blueprintWalkerController, carla.Transform(), spawning_actor['world_id']))
		results = self.client.apply_batch_sync(batch, True)

		# conduct spawn
		for i in range(len(results)):
			if results[i].error:
				logging.error(results[i].error)
			else:
				self.spawning_actor_list[i]['controller'] = results[i].actor_id

		# wait for a tick to ensure client receives the last transform of the walkers we have just created
		self.world.wait_for_tick()


	def controllActor(self, actor_id_list):

		for actor_id in actor_id_list:
			for spawning_actor in self.spawning_actor_list:
				if spawning_actor['id'] == int(actor_id):
					if spawning_actor['attribute'] == 'walker':
						actor = self.world.get_actor(spawning_actor['controller'])
						actor_profile = self.actor_profile_list[int(actor_id)]
						destination = carla.Location(float(actor_profile[9]), float(actor_profile[10]), float(actor_profile[11]))
						actor.start()
						actor.go_to_location(destination)
						actor.set_max_speed(spawning_actor[actor_profile[8]])
						del actor_profile[8:12]

					if spawning_actor['attribute'] == 'vehicle':
						actor = self.world.get_actor(spawning_actor['world_id'])
						self.client.apply_batch(carla.command.SetAutopilot(actor, True))


	def destroyActor(self, actor_id_list):
		batch = []
		for actor_id in actor_id_list:
			for spawning_actor in self.spawning_actor_list:
				if spawning_actor['id'] == int(actor_id):
					if spawning_actor['attribute'] == 'walker':
						self.world.get_actor(spawning_actor['controller']).stop()
						batch.append(carla.command.DestroyActor(spawning_actor['controller']))

					batch.append(carla.command.DestroyActor(spawning_actor['world_id']))
					self.client.apply_batch(batch)
					self.spawning_actor_list.remove(spawning_actor)


	def game_loop(self, args):

		try:

			self.client = carla.Client(args.host, args.port)
			self.client.set_timeout(2.0)
			self.world = self.client.get_world()
			self.blueprintVehicles = self.world.get_blueprint_library().filter(args.filterv)
			self.blueprintWalkers = self.world.get_blueprint_library().filter(args.filterw)
			self.blueprintWalkerController = self.world.get_blueprint_library().find('controller.ai.walker')
			self.actor_profile_list = self.readFile(args.actor_file)
			self.scenario_list = self.readFile(args.scenario_file)
			self.getEgoCar()
			# self.world.set_pedestrians_cross_factor(100)

			# self.actor_profile_list.insert(0, ['0'])
			# print(self.actor_profile_list)
			while True:

				self.world.wait_for_tick()
				self.checkScenario()
				time.sleep(0.1)

		finally:
			batch = []
			for carla_actor in self.world.get_actors():
				if carla_actor.type_id.startswith("vehicle") or carla_actor.type_id.startswith("walker"):
					if carla_actor.attributes.get('role_name') != 'ego_vehicle':
						batch.append(carla.command.DestroyActor(carla_actor.id))

			self.client.apply_batch(batch)
			print('exit')

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
		default='../actor.csv',
		help='scenario file (default: actor.csv)')
	argparser.add_argument(
		'-s', '--scenario_file',
		metavar='S',
		default='/home/mad-carla/share/catkin_ws/src/carla_helper/scenario.csv',
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
