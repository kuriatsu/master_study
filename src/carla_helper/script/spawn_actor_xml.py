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

class SpawnActor(object):

    def __init__(self):
        self.client = None
        self.world = None
        self.ego_vehicle = None
        self.scenario = None

    def readFile(self, filename):

        tree = ET.parse(filename)
        return tree.getroot()

    def game_loop(self, args):

        try:
            self.scenario = self.readFile(args.scenario_file)
            for child in self.scenario:
                print(child.attrib)
                for gchild in child:
                    print(gchild.attrib)
            # print(self.scenario)
        finally:
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
