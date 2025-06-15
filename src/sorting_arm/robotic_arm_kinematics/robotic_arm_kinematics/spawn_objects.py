#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity
import time

class ObjectSpawner(Node):
    def __init__(self):
        super().__init__('object_spawner')
        
        # Gazebo spawn service client
        self.spawn_client = self.create_client(SpawnEntity, '/spawn_entity')
        
        # Wait for Gazebo to be ready
        self.get_logger().info('Waiting for Gazebo spawn service...')
        self.spawn_client.wait_for_service()
        
        # Spawn test objects
        self.spawn_test_objects()
    
    def spawn_test_objects(self):
        """Spawn test objects for the robotic arm to pick up"""
        
        # Define test objects
        objects = [
            {
                'name': 'cube',
                'sdf': self.create_cube_sdf(),
                'x': 0.15, 'y': -0.15, 'z': 0.05
            },
            {
                'name': 'cylinder', 
                'sdf': self.create_cylinder_sdf(),
                'x': 0.20, 'y': -0.10, 'z': 0.05
            },
            {
                'name': 'hexagon',
                'sdf': self.create_hexagon_sdf(), 
                'x': 0.10, 'y': -0.20, 'z': 0.05
            }
        ]
        
        for obj in objects:
            self.spawn_object(obj['name'], obj['sdf'], obj['x'], obj['y'], obj['z'])
            time.sleep(0.5)  # Small delay between spawns
    
    def spawn_object(self, name, sdf_content, x, y, z):
        """Spawn a single object in Gazebo"""
        try:
            request = SpawnEntity.Request()
            request.name = name
            request.xml = sdf_content
            request.initial_pose.position.x = x
            request.initial_pose.position.y = y
            request.initial_pose.position.z = z
            request.initial_pose.orientation.w = 1.0
            
            future = self.spawn_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            
            if future.result():
                response = future.result()
                if response.success:
                    self.get_logger().info(f"Successfully spawned {name}")
                else:
                    self.get_logger().warn(f"Failed to spawn {name}: {response.status_message}")
            else:
                self.get_logger().error(f"Service call failed for {name}")
                
        except Exception as e:
            self.get_logger().error(f"Error spawning {name}: {str(e)}")
    
    def create_cube_sdf(self):
        """Create SDF content for a cube"""
        return """<?xml version="1.0"?>
        <sdf version="1.6">
          <model name="cube">
            <static>false</static>
            <link name="link">
              <inertial>
                <mass>0.1</mass>
                <inertia>
                  <ixx>0.000167</ixx>
                  <iyy>0.000167</iyy>
                  <izz>0.000167</izz>
                </inertia>
              </inertial>
              <collision name="collision">
                <geometry>
                  <box>
                    <size>0.03 0.03 0.03</size>
                  </box>
                </geometry>
              </collision>
              <visual name="visual">
                <geometry>
                  <box>
                    <size>0.03 0.03 0.03</size>
                  </box>
                </geometry>
                <material>
                  <ambient>1 0 0 1</ambient>
                  <diffuse>1 0 0 1</diffuse>
                </material>
              </visual>
            </link>
          </model>
        </sdf>"""
    
    def create_cylinder_sdf(self):
        """Create SDF content for a cylinder"""
        return """<?xml version="1.0"?>
        <sdf version="1.6">
          <model name="cylinder">
            <static>false</static>
            <link name="link">
              <inertial>
                <mass>0.1</mass>
                <inertia>
                  <ixx>0.000225</ixx>
                  <iyy>0.000225</iyy>
                  <izz>0.000125</izz>
                </inertia>
              </inertial>
              <collision name="collision">
                <geometry>
                  <cylinder>
                    <radius>0.015</radius>
                    <length>0.04</length>
                  </cylinder>
                </geometry>
              </collision>
              <visual name="visual">
                <geometry>
                  <cylinder>
                    <radius>0.015</radius>
                    <length>0.04</length>
                  </cylinder>
                </geometry>
                <material>
                  <ambient>0 1 0 1</ambient>
                  <diffuse>0 1 0 1</diffuse>
                </material>
              </visual>
            </link>
          </model>
        </sdf>"""
    
    def create_hexagon_sdf(self):
        """Create SDF content for a hexagon (approximated with a cylinder)"""
        return """<?xml version="1.0"?>
        <sdf version="1.6">
          <model name="hexagon">
            <static>false</static>
            <link name="link">
              <inertial>
                <mass>0.1</mass>
                <inertia>
                  <ixx>0.000200</ixx>
                  <iyy>0.000200</iyy>
                  <izz>0.000150</izz>
                </inertia>
              </inertial>
              <collision name="collision">
                <geometry>
                  <cylinder>
                    <radius>0.02</radius>
                    <length>0.02</length>
                  </cylinder>
                </geometry>
              </collision>
              <visual name="visual">
                <geometry>
                  <cylinder>
                    <radius>0.02</radius>
                    <length>0.02</length>
                  </cylinder>
                </geometry>
                <material>
                  <ambient>0 0 1 1</ambient>
                  <diffuse>0 0 1 1</diffuse>
                </material>
              </visual>
            </link>
          </model>
        </sdf>"""

def main(args=None):
    rclpy.init(args=args)
    spawner = ObjectSpawner()
    
    # Keep node alive briefly to ensure all objects are spawned
    time.sleep(3)
    
    spawner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()