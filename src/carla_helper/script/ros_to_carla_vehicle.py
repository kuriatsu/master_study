#!/usr/bin/env python
import scenario_xml
import rospy


class TestEgoCar(object):

	def __init__(self):
		self.client = None
		self.world = None
		self.ego_vehicle = None
		self.ego_pose = None


	def getEgoCar(self):

		for carla_actor in self.world.get_actors():
			if carla_actor.type_id.startswith("vehicle"):
				if carla_actor.attributes.get('role_name') == 'ego_vehicle':
					self.ego_vehicle = carla_actor

	def game_loop(self, args):

		try:

			self.client = carla.Client(args.host, args.port)
			self.client.set_timeout(2.0)
			self.world = self.client.get_world()
			self.getEgoCar()

			while True:

				self.world.wait_for_tick()
				self.ego_pose = self.ego_vehicle.get_transform()
				time.sleep(0.1)

		finally:
			print('exit game loop')

def main(args):
    rospy.init_node('my_carla_bridge')
    try:
        sx = ScenarioXML(args.scenario_file)
        sx.client = carla.Client(args.host, args.port)
        sx.client.set_timeout(2.0)
        sx.world = sx.client.get_world()
        sx.blueprint = sx.world.get_blueprint_library()

        sx.getEgoCar()
        sx.spawnActor(sx.scenario[0].findall('spawn'))
        sx.moveActor(sx.scenario[0].findall('move'))
        sx.poseActor(sx.scenario[0].findall('pose'))


        while sx.checkTrigger():
            sx.world.wait_for_tick()
            if sx.ego_vehicle is None:
                sx.getEgoCar()
            else:
                sx.controlActor()

            time.sleep(0.1)

    except:
        return False

    finally:
        batch = []
        for carla_actor in sx.world.get_actors():
            if carla_actor.type_id.startswith("vehicle") or carla_actor.type_id.startswith("walker") or carla_actor.type_id.startswith("controller"):
                if carla_actor.attributes.get('role_name') != 'ego_vehicle':
                    batch.append(carla.command.DestroyActor(carla_actor.id))

        self.client.apply_batch(batch)
        return True


if __name__ == '__main__':

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
        '-s', '--scenario_file',
        metavar='S',
        default='/home/mad-carla/share/catkin_ws/src/carla_helper/scenario.xml',
        help='scenario file (default: scenario.xml)')
	args = argparser.parse_args()

    main(args)
