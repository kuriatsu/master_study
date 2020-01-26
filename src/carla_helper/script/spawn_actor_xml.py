#!/usr/bin/env python
"""
Welcome to CARLA scenario controller.
"""
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
import xml.etree.ElementTree as ET
import argparse
import warnings
import random
import math

class SpawnActor(object):

    def __init__(self):
        self.client = None
        self.world = None
        self.ego_vehicle = None
        self.scenario = None
		# self.current_scenario = None
		self.intrusion_thres = 2.0
		self.current_scenario_id = 0
		self.blueprintVehicles = None
		self.blueprintWalkers = None
		self.blueprintWalkerController = None
		self.control_actor_list = []

    def readFile(self, filename):

        tree = ET.parse(filename)
		root = tree.getroot()
		edit_tag_list = ['transform', 'location', 'goal']

		for tag in edit_tag_list:
			for itr in root.iter(tag):
				itr.text = [float(val) for val in itr.text.split(',')]

        return root


	def getEgoCar(self):

		for carla_actor in self.world.get_actors():
			if carla_actor.type_id.startswith("vehicle"):
				if carla_actor.attributes.get('role_name') == 'ego_vehicle':
					self.ego_vehicle = carla_actor


	def checkScenario(self):
		ego_pose = self.ego_vehicle.get_transform()
		current_scenario = self.scenario[self.current_scenario_id]
		distance = (ego_pose.location.x - current_scenario[0].text[0]) ** 2 \
				   + (ego_pose.location.y - current_scenario[0].text[1])) ** 2

		if distance < self.intrusion_thres:
			self.spawnActor(current_scenario.findall('spawn'))
			self.moveActor(current_scenario.findall('move'))
			self.killActor(current_scenario.findall('kill'))
			self.current_scenario_id += 1


	def getBlueprint(self, blueprint_list, name):

		if name == 'random':
			blueprint = random.choice(blueprint_list)
		else:
			try:
				blueprint = blueprint_list.find(name)
			except ValueError:
				blueprint = random.choice(blueprint_list)
				warnings.warn('spcecified blueprint is not exist : {}'.format(actor.find('blueprint').text))

		return blueprint


	def spawnActor(self, spawn_list):
    	# we spawn the actor
		batch = []
		ai_walkers_list = []

		for spawn in spawn_list:
			actor_ids = {}
			# add id to the list of currently existing actor under the world
			# actor_ids['scenario_id'] = int(spawn.attrib['id'])

			# set blueprint for walker
			if 'walker' in spawn.find('type').text:

				if spawn.find('type').text == 'ai_walker':
					ai_walkers_list.append(spawn)

				blueprint = self.getBlueprint(self.blueprintWalkers, spawn.find('type').text)
				# set as not invencible
				if blueprint.has_attribute('is_invincible'):
					blueprint.set_attribute('is_invincible', 'false')

			# set blueprint for vehicle
			elif 'vehicle' in spawn.find('type').text:
				blueprint = self.getBlueprint(self.blueprintVehicles, spawn.find('type').text)

				if blueprint.has_attribute('color'):
					color = random.choice(blueprint.get_attribute('color').recommended_values)
					blueprint.set_attribute('color', color)

				if blueprint.has_attribute('driver_id'):
					driver_id = random.choice(blueprint.get_attribute('driver_id').recommended_values)
					blueprint.set_attribute('driver_id', driver_id)

				blueprint.set_attribute('role_name', 'driver')

			# append command to spawn actor to batch
			buf = spawn.find('transform').text
			transform = carla.Transform(carla.Location(carla.Location(buf[0], buf[1], buf[2]), carla.Rotation(buf[3], buf[4], buf[5])))
			batch.append(carla.command.SpawnActor(blueprint, transform))
			self.actor_ids_list.append(actor_ids)

		# conduct spawn actor
		results = self.client.apply_batch_sync(batch)
		for i in range(len(results)):
			if results[i].error:
				warnings.warn(results[i].error)
			else:
				world_id = ET.SubElement(spawn_list[i], 'world_id')
				world_id.text = results[i].actor_id
				# self.actor_ids_list[i]["world_id"] = results[i].actor_id

		# append command to spawn ai_controller to batch
		batch = []
		for ai_walker in ai_walkers_list:
			batch.append(carla.command.SpawnActor(self.blueprintWalkerController, carla.Transform(), ai_walker.find('world_id')))
		results = self.client.apply_batch_sync(batch, True)

		# conduct spawn ai controller
		for i in range(len(results)):
			if (results[i].error):
				logging.error(results[i].error)
			else:
				ai_controller_id = ET.SubElement(ai_walkers_list[i], 'ai_controller_id')
				ai_controller_id.text = results[i].actor_id
		# wait for a tick to ensure client receives the last transform of the walkers we have just created
		self.world.wait_for_tick()


	def moveActor(self, move_list):

		def moveAiWalker(world_id, speed):
			actor = self.world.get_actor(world_id)
			actor.start()
			actor.go_to_location(self.world.get_random_location_from_navigation())
			actor.set_max_speed(speed)

		def moveAiVehicle(world_id):
			actor = self.world.get_actor(spawned_actor['world_id'])
			actor.set_autopilot(True)

		def moveInnocentActor(world_id, type, speed, goal):
			control_actor = {}
			control_actor['actor'] = self.world.get_actor(world_id)
			control_actor['type'] = type
			control_actor['goal'] = carla.Location(goal[0], goal[1], goal[2])
			control_actor['speed'] = speed
			self.control_actor_list.append(control_actor)

		for move in move_list:
			for spawn in self.scenario.iter('spawn'):
				if move.attrib.get('id') == spawn.attrib.get('id'):
					type = move.find('type').text
					if type == 'ai_walker':
						moveAiWalker(move.find('ai_controller_id').text, float(move.find('speed').text))
					elif type == 'ai_vehicle'
						moveAiVehicle(move.find('world_id').text)
					elif type == 'walker' or type == 'vehicle':
						moveInnocentActor(move.find('world_id').text, type, move.find('speed').text, move.find('goal').text)



	def controlActor(self):
		# debug = self.world.debug
		for control_actor in self.control_actor_list:
			if control_actor['actor'].is_alive == False:
				self.control_actor_list.remove(control_actor)
				continue

			else:
				actor = control_actor['actor']
				transform = actor.get_transform()
				vector = carla.Vector3D(control_actor['goal'].x - transform.location.x,
			 							 	 control_actor['goal'].y - transform.location.y,
										 	 control_actor['goal'].z)
				dist = math.sqrt(vector.x ** 2 + vector.y ** 2)
				vector.x = vector.x / dist
				vector.y = vector.y / dist

				# debug.draw_arrow(begin=transform.location ,end=transform.location + carla.Location(vector.x, vector.y, 0.0), life_time=0.5)

			if control_actor['type'] == 'walker':
				# print(control_actor['actor'].id, math.sqrt(control_actor['actor'].get_velocity().x**2 + control_actor['actor'].get_velocity().y**2), math.sqrt(vector.x**2 + vector.y**2),control_actor['speed'])
				if dist < 0.5:
					control = carla.WalkerControl(direction=carla.Vector3D(0.0,0.0,0.0),speed=0.0)
					self.control_actor_list.remove(control_actor)
				else:
					control = carla.WalkerControl(direction=vector, speed=control_actor['speed'])
					control_actor['actor'].apply_control(control)

			elif control_actor['type'] == 'vehicle':
				# calc vel and rotation
				if dist < 1.0:
					control_actor['actor'].set_velocity(carla.Vector3D(0.0,0.0,0.0))
					self.control_actor_list.remove(control_actor)
				else:
					alpha = math.atan(vector.y / vector.x) - transform.rotation.yaw
					omega = 2 * control_actor['speed'] * math.sin(alpha) / dist

					control = carla.VehicleControl()
					velocity = carla.Vector3D()
					velocity.x = vector.x * control_actor['speed']
					velocity.y = vector.y  * control_actor['speed']
					velocity.z = 0.0
					control_actor['actor'].set_velocity(velocity)
					control_actor['actor'].set_angular_velocity(carla.Vector3D(0.0, 0.0, omega))


	def killActor(self, death_note):
		batch = []
		for death in death_note:
			for spawn in self.scenario.iter('spawn'):
				if death.attrib.get('id') == spawn.attrib.get('id'):
					if spawn.find('type').text == 'ai_walker':
						self.world.get_actor(spawn.find('ai_controller_id').text).stop()
						batch.append(carla.command.DestroyActor(spawned_actor['controller']))

					batch.append(carla.command.DestroyActor(spawn.find('world_id').text))
					spawn.remove(spawn.find('world_id'))

		self.client.apply_batch(batch)


    def game_loop(self, args):

        try:
			self.client = carla.Client(args.host, args.port)
			self.client.set_timeout(2.0)
			self.world = self.client.get_world()
			self.blueprintVehicles = self.world.get_blueprint_library().filter(args.filterv)
			self.blueprintWalkers = self.world.get_blueprint_library().filter(args.filterw)
			self.blueprintWalkerController = self.world.get_blueprint_library().find('controller.ai.walker')
			self.scenario = self.readFile(args.scenario_file)

			self.getEgoCar()
			# self.current_scenario = self.scenario.find('scenario')

			while True:
				self.world.wait_for_tick()
				self.checkScenario()
				self.controlActor()
				time.sleep(0.1)

        finally:
			batch = []
			for carla_actor in self.world.get_actors():
				if carla_actor.type_id.startswith("vehicle") or carla_actor.type_id.startswith("walker"):
					if carla_actor.attributes.get('role_name') != 'ego_vehicle':
						batch.append(carla.command.DestroyActor(carla_actor.id))

			self.client.apply_batch(batch)
            print('collapsed')


def main():

    argparser = argparse.ArgumentParser( description = __doc__)
    argparser.add_argument(
        '--host',
        metavar='xx.x.x.x',
        default='127.0.0.1',
        help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument(
		'-p', '--port',
		metavar='P',
		default=2000,
		type=int,
		help='TCP port to listen to (default: 2000)')
    argparser.add_argument(
    	'-s', '--scenario_file',
    	metavar='S',
    	default='/home/mad-carla/share/catkin_ws/src/carla_helper/scenario.xml',
    	help='scenario file (default: scenario.xml)')
    args = argparser.parse_args()

    try:
        spawn_actor = SpawnActor()
        spawn_actor.game_loop(args)
    finally:
        print('collapsed')

if __name__ == '__main__':
    main()
