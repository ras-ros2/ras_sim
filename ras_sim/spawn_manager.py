#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from std_msgs.msg import String
from ras_common.config.loaders.lab_setup import LabSetup as LabLoader
import numpy as np
from tf_transformations import euler_from_quaternion, quaternion_from_euler
import yaml

class SpawnIgn(Node):
    def __init__(self):
        super().__init__("spawn_ign_node")
        self.get_logger().info("IGN SPAWN NODE STARTED")
        LabLoader.init()
        
        self.robot_pose = LabLoader.robot_pose

        self.spawn_pose = self.create_publisher(Pose, "/spawn_model", 10)
        self.despawn_pose = self.create_publisher(String, "/despawn_model", 10)

        self.robot_rot_eulers = euler_from_quaternion((self.robot_pose.orientation.x, self.robot_pose.orientation.y, self.robot_pose.orientation.z, self.robot_pose.orientation.w))

        np_pose = self.convert_pose_to_world()

        self.spawn_model(np_pose)

    def euler_to_matrix(self, roll, pitch, yaw):
        R_x = np.array([[1, 0, 0],
                        [0, np.cos(roll), -np.sin(roll)],
                        [0, np.sin(roll), np.cos(roll)]])
        
        R_y = np.array([[np.cos(pitch), 0, np.sin(pitch)],
                        [0, 1, 0],
                        [-np.sin(pitch), 0, np.cos(pitch)]])
        
        R_z = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                        [np.sin(yaw), np.cos(yaw), 0],
                        [0, 0, 1]])
    
        return R_z @ R_y @ R_x
    
    def convert_pose_to_world(self):

        old_pos = np.array([0.2, -0.35, 0.33])
        roll, pitch, yaw =  self.robot_rot_eulers[0], self.robot_rot_eulers[1], self.robot_rot_eulers[2]
        translation = np.array([self.robot_pose.position.x, self.robot_pose.position.y, self.robot_pose.position.z])

        rotation_matrix = self.euler_to_matrix(roll, pitch, yaw)

        new_pos = rotation_matrix @ old_pos + translation

        return new_pos



    def spawn_model(self, np_pose):
        self.get_logger().info("Spawning Model at : " + str(np_pose))

        spawn_pose = Pose()
        spawn_pose.position.x = np_pose[0]
        spawn_pose.position.y = np_pose[1]
        spawn_pose.position.z = np_pose[2]

        self.spawn_pose.publish(spawn_pose)

def main():
    rclpy.init(args=None)
    node = SpawnIgn()
    while rclpy.ok():
        rclpy.spin_once(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()




