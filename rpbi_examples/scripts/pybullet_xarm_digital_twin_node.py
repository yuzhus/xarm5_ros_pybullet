#!/usr/bin/env python3
from rpbi.ros_node import RosNode
from cob_srvs.srv import SetString, SetStringRequest
from ros_pybullet_interface.msg import PybulletObject
from custom_ros_tools.config import config_to_str
from ros_pybullet_interface.srv import AddPybulletObject
import rospy
from geometry_msgs.msg import Point
import time



class Node(RosNode):


    interval = 15.0


    def __init__(self):
        RosNode.__init__(self, 'pybullet_objects_example_node')
        self.it = 0
        rospy.Subscriber("goal_position", Point, self.callback)
        # self.add_pybullet_object()
        # self.Timer(self.Duration(self.interval), self.main_loop)
        


    def callback(self,data):
        if data!=0:
            srv = 'rpbi/add_pybullet_object'
            self.wait_for_service(srv)
            config = {  'name': format(time.time()),
                        'createVisualShape': {'shapeType': "GEOM_BOX",
                                            'halfExtents': [0.025, 0.025, 0.025],
                                            'rgbaColor': [0.9, 1.0, 0.3, 1.0]},
                        'createCollisionShape': {'shapeType': "GEOM_BOX",
                                                'halfExtents': [0.025, 0.025, 0.025]},  
                        'baseMass': 0.5,
                        'basePosition': [data.x, data.y, data.z],
                        'changeDynamics': {'lateralFriction': 1.0,
                                        'spinningFriction': 0.0,
                                        'rollingFriction': 0.0,
                                        'restitution': 0.0,
                                        'linearDamping': 0.04,
                                        'angularDamping': 0.04,
                                        'contactStiffness': 2000.0,
                                        'contactDamping': 0.7}
                    }
            try:
                handle = self.ServiceProxy(srv, AddPybulletObject)
                req = PybulletObject(config=config_to_str(config))
                req.object_type=PybulletObject.DYNAMIC
                # req.filename='{rpbi_examples}/configs/pybullet_xarm_digital_twin_example/dynamic_box.yaml'

                resp = handle(req)
                if resp.success:
                    self.loginfo('successfully added object')
                else:
                    self.logwarn('failed to add object: %s' % resp.message)
            except Exception as e:
                self.logerr('failed to add object: %s' % str(e))
        


    def remove_object(self):
        srv = 'rpbi/remove_pybullet_object'
        self.wait_for_service(srv)
        try:
            handle = self.ServiceProxy(srv, SetString)
            req = SetStringRequest(data="dynamic_goal")
            resp = handle(req)
            if resp.success:
                self.loginfo('successfully removed object')
            else:
                self.logwarn('failed to remove object: %s' % resp.message)
        except Exception as e:
            self.logerr('failed to remove object: %s' % str(e))


    def main_loop(self, event):
        # self.remove_object()
        self.add_pybullet_object()
        self.it += 1


def main():
    Node().spin()


if __name__ == '__main__':
    main()
