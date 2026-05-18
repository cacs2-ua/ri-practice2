#!/usr/bin/env python3

import rospy, tf, rospkg
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import *
import numpy as np

if __name__ == '__main__':
    print("Waiting for gazebo services...")
    rospy.init_node("spawn_random_position")
    rospy.wait_for_service("/gazebo/spawn_sdf_model")
    rospy.wait_for_service("/gazebo/spawn_urdf_model")

    #Spawn object
    spawn_model = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)

    with open(rospkg.RosPack().get_path("intelligent_robotics")+"/models/object/model.sdf", "r") as f:
        object_model = f.read()
        
    # Red object spawn position near the UR5 robotic arm in Gazebo world coordinates.
    # The UR5 is spawned around x=4.5, y=3.0, so the object is placed close to its
    # reachable workspace.
    mu = np.array([5.0, 3.5])
    point = mu.copy()

    object_pose = Pose(
        Point(x=point[0], y=point[1], z=0.1),
        Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
    )

    spawn_model("Object", object_model, "object", object_pose, "world")

    #Spawn camera
    spawn_model = rospy.ServiceProxy("/gazebo/spawn_urdf_model", SpawnModel)

    with open(rospkg.RosPack().get_path("intelligent_robotics")+"/models/realsense/model.urdf", "r") as f:
        object_model = f.read()


    object_pose   =   Pose(Point(x=0, y=0,    z=10.0),   Quaternion(x=0.0, y=0.0, z=0.0, w=1.0))
    spawn_model("Realsense", object_model, "camera", object_pose, "world")