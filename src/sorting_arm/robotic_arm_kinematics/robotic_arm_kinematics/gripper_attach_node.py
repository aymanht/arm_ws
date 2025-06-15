#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from robotic_arm_msgs.srv import GripperControl
from geometry_msgs.msg import Pose
from gazebo_msgs.srv import SpawnEntity, DeleteEntity, GetEntityState, SetEntityState
from gazebo_msgs.msg import EntityState
import time

class GripperAttachNode(Node):
    def __init__(self):
        super().__init__('gripper_attach_node')
        
        # Service to handle attach/detach requests
        self.gripper_service = self.create_service(
            GripperControl, 
            'gripper_control', 
            self.gripper_control_callback
        )
        
        # Gazebo service clients
        self.spawn_client = self.create_client(SpawnEntity, '/spawn_entity')
        self.delete_client = self.create_client(DeleteEntity, '/delete_entity')
        self.get_state_client = self.create_client(GetEntityState, '/get_entity_state')
        self.set_state_client = self.create_client(SetEntityState, '/set_entity_state')
        
        # Keep track of attached objects
        self.attached_objects = []
        self.gripper_link_name = "end_effector"
        self.object_offset = {'x': 0.0, 'y': 0.0, 'z': 0.05}  # Offset from gripper center
        
        self.get_logger().info('Gripper attach service started')
    
    def gripper_control_callback(self, request, response):
        """Handle gripper attach/detach requests"""
        try:
            if request.attach:
                success = self.attach_object(request.object_name)
                if success:
                    response.success = True
                    response.message = f"Object attached successfully"
                    self.get_logger().info(f"Object attached to gripper")
                else:
                    response.success = False
                    response.message = "Failed to attach object"
            else:
                success = self.detach_objects()
                if success:
                    response.success = True
                    response.message = "Objects detached successfully"
                    self.get_logger().info("Objects detached from gripper")
                else:
                    response.success = False
                    response.message = "Failed to detach objects"
                    
        except Exception as e:
            response.success = False
            response.message = f"Error: {str(e)}"
            self.get_logger().error(f"Gripper control error: {str(e)}")
            
        return response
    
    def attach_object(self, object_name=""):
        """Attach nearest object to gripper"""
        try:
            # Find nearby objects if no specific object name given
            if not object_name:
                object_name = self.find_nearest_object()
            
            if not object_name:
                self.get_logger().warn("No object found to attach")
                return False
            
            # Get gripper position
            gripper_state = self.get_entity_state(self.gripper_link_name)
            if not gripper_state:
                return False
            
            # Get object current state
            object_state = self.get_entity_state(object_name)
            if not object_state:
                return False
            
            # Calculate attachment position (offset from gripper)
            attach_pose = Pose()
            attach_pose.position.x = gripper_state.pose.position.x + self.object_offset['x']
            attach_pose.position.y = gripper_state.pose.position.y + self.object_offset['y'] 
            attach_pose.position.z = gripper_state.pose.position.z + self.object_offset['z']
            attach_pose.orientation = gripper_state.pose.orientation
            
            # Move object to attachment position
            new_state = EntityState()
            new_state.name = object_name
            new_state.pose = attach_pose
            new_state.reference_frame = self.gripper_link_name
            
            if self.set_entity_state(new_state):
                self.attached_objects.append(object_name)
                return True
                
        except Exception as e:
            self.get_logger().error(f"Error attaching object: {str(e)}")
            
        return False
    
    def detach_objects(self):
        """Detach all attached objects"""
        try:
            for obj_name in self.attached_objects:
                # Get current object state
                object_state = self.get_entity_state(obj_name)
                if object_state:
                    # Set object to world reference frame
                    new_state = EntityState()
                    new_state.name = obj_name
                    new_state.pose = object_state.pose
                    new_state.reference_frame = "world"
                    
                    self.set_entity_state(new_state)
            
            self.attached_objects.clear()
            return True
            
        except Exception as e:
            self.get_logger().error(f"Error detaching objects: {str(e)}")
            return False
    
    def find_nearest_object(self):
        """Find the nearest object to the gripper"""
        # For simplicity, look for common object names
        object_candidates = ["cube", "cylinder", "hexagon", "box", "sphere"]
        
        gripper_state = self.get_entity_state(self.gripper_link_name)
        if not gripper_state:
            return None
            
        min_distance = float('inf')
        nearest_object = None
        
        for obj_name in object_candidates:
            obj_state = self.get_entity_state(obj_name)
            if obj_state:
                # Calculate distance
                dx = gripper_state.pose.position.x - obj_state.pose.position.x
                dy = gripper_state.pose.position.y - obj_state.pose.position.y
                dz = gripper_state.pose.position.z - obj_state.pose.position.z
                distance = (dx*dx + dy*dy + dz*dz)**0.5
                
                if distance < min_distance and distance < 0.1:  # 10cm threshold
                    min_distance = distance
                    nearest_object = obj_name
        
        return nearest_object
    
    def get_entity_state(self, entity_name):
        """Get entity state from Gazebo"""
        try:
            if not self.get_state_client.wait_for_service(timeout_sec=1.0):
                return None
                
            request = GetEntityState.Request()
            request.name = entity_name
            
            future = self.get_state_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
            
            if future.result():
                return future.result()
        except Exception as e:
            self.get_logger().error(f"Error getting entity state: {str(e)}")
            
        return None
    
    def set_entity_state(self, entity_state):
        """Set entity state in Gazebo"""
        try:
            if not self.set_state_client.wait_for_service(timeout_sec=1.0):
                return False
                
            request = SetEntityState.Request()
            request.state = entity_state
            
            future = self.set_state_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
            
            if future.result():
                return future.result().success
        except Exception as e:
            self.get_logger().error(f"Error setting entity state: {str(e)}")
            
        return False

def main(args=None):
    rclpy.init(args=args)
    node = GripperAttachNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()