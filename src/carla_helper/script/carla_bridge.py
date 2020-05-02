#!/usr/bin/python
# -*- coding: utf-8 -*-

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

import rospy
import tf
import carla
from std_msgs.msg import Header
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Accel
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import PoseStamped
from derived_object_msgs.msg import ObjectArray
from derived_object_msgs.msg import Object
from shape_msgs.msg import SolidPrimitive

bike_blueprints = [
'vehicle.bh.crossbike',
'vehicle.diamondback.century',
]
motorcycle_blueprints = [
'vehicle.harley-davidson.low_rider',
'vehicle.kawasaki.ninja',
'vehicle.yamaha.yzf',
]
truck_blueprints = [
'vehicle.carlamotors.carlacola'
]

class CarlaBridge(object):
    def __init__(self):
        rospy.init_node('carla_bridge_node', anonymous=True)

        self.client = None
        self.world = None
        self.pub_pose = rospy.Publisher('/ndt_pose', TwistStamped, queue_size=1)
        self.pub_obj = rospy.Publisher('/carla_actors', ObjectArray, queue_size=5)
        self.timer = rospy.Timer(rospy.Duration(0.1), self.timerCallback, oneshot=False)
        self.ego_vehicle = None
        self.ego_pose = None
        # following 2 list muxt have same actor with the same index
        self.ros_actors = [] # derived_object_msgs
        self.carla_actors = [] # carla.actor


    def getWorld(self):
        # host = rospy.get_param('host')
        host = '127.0.0.1'
        # port = rospy.get_param('port')
        port = 2000
        self.client = carla.Client(host, port)
        self.client.set_timeout(2.0)
        self.world = self.client.get_world()


    def getActors(self):
        """Call simulator and get instances of actors and store them, create list of derived_object_msgs

        """
        # self.ros_actors.clear()

        for actor in self.world.get_actors():
            if actor.attributes.get('role_name') in ['ego_vehicle', 'hero']:
                self.ego_vehcle = actor

            elif actor.type_id.startswith("vehicle") or actor.type_id.startswith("walker"):
                self.carla_actors.append(actor)
                derived_obj = Object()
                derived_obj.id = actor.id
                derived_obj.shape.type = SolidPrimitive.BOX
                derived_obj.shape.dimensions = [actor.bounding_box.extent.x*2,
                actor.bounding_box.extent.y*2,
                actor.bounding_box.extent.z*2]

                if actor.type_id in bike_blueprints:
                    derived_obj.classification = Object.CLASSIFICATION_BIKE
                elif actor.type_id in motorcycle_blueprints:
                    derived_obj.classification = Object.CLASSIFICATION_MOTORCYCLE
                elif actor.type_id in truck_blueprints:
                    derived_obj.classification = Object.CLASSIFICATION_TRUCK
                elif actor.type_id.startswith("vehicle"):
                    derived_obj.classification = Object.CLASSIFICATION_CAR
                elif actor.type_id.startswith("walker"):
                    derived_obj.classification = Object.CLASSIFICATION_PEDESTRIAN

                self.ros_actors.append(derived_obj)



    def updateActors(self):
        """update actor dynamic info
        """
        self.ego_pose =
        for i, actor in enumerate(self.carla_actors):

            transform = actor.get_transform()
            if transform.location == carla.Vector3D(0.0,0.0,0.0):
                print('actor may be died. id : ', actor.id)
                del self.ros_actors[i]
                del self.carla_actors[i]
                continue

            accel = actor.get_acceleration()
            twist_angular = actor.get_angular_velocity()
            twist_liner = actor.get_velocity()

            self.ros_actors[i].header = Header(stamp=rospy.Time.now(), frame_id='map')
            self.ros_actors[i].pose = Pose(position=Point(x=transform.location.x, y=transform.location.y, z=transform.location.z),
                              orientation=self.rotation_to_quaternion(transform.rotation))
            self.ros_actors[i].twist = Twist(linear=Vector3(x=twist_liner.x, y=twist_liner.y, z=twist_liner.z),
                                angular=Vector3(x=twist_angular.x, y=twist_angular.y, z=twist_angular.z))
            self.ros_actors[i].accel = Accel(linear=Vector3(x=accel.x, y=accel.y, z=accel.z), angular=Vector3())


    def rotation_to_quaternion(self, rotation):
        quat = tf.transformations.quaternion_from_euler(rotation.roll, rotation.pitch, rotation.yaw)
        return Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])


    def timerCallback(self, event):
        self.updateActors()
        header = Header(stamp=rospy.Time.now(), frame_id='map')
        print(len(self.ros_actors))
        object_array = ObjectArray(header=header, objects=self.ros_actors)
        self.pub_obj.publish(object_array)


def main():

    try:
        carla_bridge = CarlaBridge()
        carla_bridge.getWorld()
        carla_bridge.getActors()
        rospy.spin()

    finally:
        print('finished')


if __name__ == '__main__':
    main()
