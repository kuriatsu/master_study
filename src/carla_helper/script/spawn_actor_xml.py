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
		self.actor_ids_list = []
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
		distance = (ego_pose.location.x - scenario[self.current_scenario_id][0].text[0]) ** 2 \
				   + (ego_pose.location.y - scenario[self.current_scenario_id][0].text[1])) ** 2

		if distance < self.intrusion_thres:
			self.spawnActor(scenario.findall('spawn'))
			self.moveActor(scenario.findall('move'))
			self.killActor(scenario.findall('kill'))
			self.current_scenario_id += 1


	def getBlueprint(self, blueprint_list, name):

		if (name == 'random'):
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
		for spawn in spawn_list:
			actor_ids = {}
			# add id to the list of currently existing actor under the world
			actor_ids['scenario_id'] = int(spawn.attrib['id'])

			# set blueprint for walker
			if ('walker' in spawn.find('type').text):

				if (spawn.find('type').text == 'ai_walker'):
					actor_ids['ai_controller'] = None

				blueprint = self.getBlueprint(self.blueprintWalkers, spawn.find('type').text)
				# set as not invencible
				if blueprint.has_attribute('is_invincible'):
					blueprint.set_attribute('is_invincible', 'false')

			# set blueprint for vehicle
		elif ('vehicle' in spawn.find('type').text):
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
			transform = carla.Transform(carla.Location(carla.Location(buf[0], buf[1], buf[2]), carla.Rotation(buf[3], buf[4], buf[5]))
			batch.append(carla.command.SpawnActor(blueprint, transform))
			self.actor_ids_list.append(actor_ids)

		# conduct spawn actor
		results = self.client.apply_batch_sync(batch)
		for i in range(len(results)):
			if results[i].error:
				warnings.warn(results[i].error)
			else:
				self.actor_ids_list[i]["world_id"] = results[i].actor_id

		# append command to spawn ai_controller to batch
		batch = []
		ai_walker_index = []
		for i, actor_ids in enumerate(self.actor_ids_list):
			if 'ai_controller' in actor_dict:
				ai_walker_index.append(i)
				batch.append(carla.command.SpawnActor(self.blueprintWalkerController, carla.Transform(), actor_ids['world_id']))
		results = self.client.apply_batch_sync(batch, True)

		# conduct spawn ai controller
		for i in range(len(results)):
			if results[i].error:
				logging.error(results[i].error)
			else:
				self.actor_ids_list[ai_walker_ids[i]]['ai_controller'] = results[i].actor_id
		# wait for a tick to ensure client receives the last transform of the walkers we have just created
		self.world.wait_for_tick()


	def moveActor(self, motion_list):

		for motion in motion_list:
			if motion['attribute'] == 'ai_walker':
				actor = self.world.get_actor(spawned_actor['controller'])
				actor.start()
				actor.go_to_location(self.world.get_random_location_from_navigation())
				actor.set_max_speed(spawned_actor[self.actor_profile_list[int(actor_id)][8]])

			elif spawned_actor['attribute'] == 'ai_vehicle':
				actor = self.world.get_actor(spawned_actor['world_id'])
				actor.set_autopilot(True)

			elif spawned_actor['attribute'] == 'walker' or spawned_actor['attribute'] == 'vehicle':
				control_actor = {}
				control_actor['actor'] = self.world.get_actor(spawned_actor['world_id'])
				control_actor['attribute'] = spawned_actor['attribute']
				goal = self.actor_profile_list[int(actor_id)][9:12]
				control_actor['goal'] = carla.Location(float(goal[0]), float(goal[1]), float(goal[2]))
				control_actor['speed'] = spawned_actor[self.actor_profile_list[int(actor_id)][8]]
				del self.actor_profile_list[int(actor_id)][8:12]
				self.control_actor_list.append(control_actor)


	def controlActor(self):
		debug = self.world.debug
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

				debug.draw_arrow(begin=transform.location ,end=transform.location + carla.Location(vector.x, vector.y, 0.0), life_time=0.5)

			if control_actor['attribute'] == 'walker':
				# print(control_actor['actor'].id, math.sqrt(control_actor['actor'].get_velocity().x**2 + control_actor['actor'].get_velocity().y**2), math.sqrt(vector.x**2 + vector.y**2),control_actor['speed'])
				if dist < 0.5:
					control = carla.WalkerControl(direction=carla.Vector3D(0.0,0.0,0.0),speed=0.0)
					self.control_actor_list.remove(control_actor)
				else:
					control = carla.WalkerControl(direction=vector, speed=control_actor['speed'])
					control_actor['actor'].apply_control(control)

			elif control_actor['attribute'] == 'vehicle':
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


	def destroyActor(self, actor_id_list):
		batch = []
		for actor_id in actor_id_list:
			for spawned_actor in self.spawned_actor_list:
				if spawned_actor['id'] == int(actor_id):
					if spawned_actor['attribute'] == 'ai_walker':
						self.world.get_actor(spawned_actor['controller']).stop()
						batch.append(carla.command.DestroyActor(spawned_actor['controller']))

					batch.append(carla.command.DestroyActor(spawned_actor['world_id']))
					self.spawned_actor_list.remove(spawned_actor)
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
