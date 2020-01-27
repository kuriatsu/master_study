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

import carla
import xml.etree.ElementTree as ET
import argparse
import csv
import warnings

color = [
carla.Color(0,134,179),
carla.Color(166,186,178),
carla.Color(111,186,44),
carla.Color(52,114,161),
carla.Color(249,194,112),
carla.Color(82,59,67),
carla.Color(232,55,74),
carla.Color(237,241,176),
carla.Color(15,216,45),
carla.Color(0,175,204),
]
line_thickness = {
'walker' : 0.2,
'vehicle' : 1.0
}
def readFile(filename):

    tree = ET.parse(filename)
    root = tree.getroot()
    edit_tag_list = ['transform', 'location', 'goal']

    for tag in edit_tag_list:
        for itr in root.iter(tag):
            itr.text = [float(val) for val in itr.text.split(',')]

    return root

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
    default='/home/mad-carla/share/catkin_ws/src/carla_helper/scenario.xml',
    help='scenario file (default: scenario.csv)')

    args = argparser.parse_args()
    client = carla.Client(args.host, args.port)
    client.set_timeout(2.0)
    world = client.get_world()
    debug = world.debug

    scenario = readFile(args.scenario_file)
    actor_position = {}

    for trriger in scenario:
        buf = trriger[0].text
        location = carla.Location(buf[0], buf[1], buf[2])
        debug.draw_point(location=location, life_time=30, size=0.5, color=color[int(trriger.attrib.get('id'))%10])
        debug.draw_string(location=location+carla.Location(z=1.0), text='trigger'+trriger.attrib.get('id'), color=carla.Color(255,255,255), life_time=30)

        for i, action in enumerate(trriger[1:]):
            print(action.tag)
            if action.tag == 'spawn':
                buf = action.find('transform').text
                location = carla.Location(buf[0], buf[1], buf[2])
                debug.draw_point(location=location, life_time=30, size=0.1, color=color[i%10])
                debug.draw_string(location=location+carla.Location(z=1.0), text='spawn'+action.find('type').text, color=carla.Color(255,255,255), life_time=30)
                actor_position[action.attrib.get('id')] = location

            if action.tag == 'move':
                goal = action.find('goal')
                start = actor_position.get(action.attrib.get('id'))
                if goal is not None:
                    if start is not None:
                        buf = goal.text
                        goal = carla.Location(buf[0], buf[1], buf[2])
                        debug.draw_line(begin=start, end=goal, color=color[int(trriger.attrib.get('id'))%10], thickness=0.5, life_time=30)
                        actor_position[action.attrib.get('id')] = goal
                    else:
                        warnings.warn('actor {} is not spawned but goal is set {}'.format(action.attrib.get('id'), trriger.attrib.get('id')))
                else:
                    if start is not None:
                        debug.draw_string(location=start+carla.Location(z=1.0), text='start_ai', color=carla.Color(255,255,255), life_time=30)
                    else:
                        warnings.warn('actor {} is not spawned but ai is start {}'.format(action.attrib.get('id'), trriger.attrib.get('id')))

if __name__ == '__main__':
    main()
