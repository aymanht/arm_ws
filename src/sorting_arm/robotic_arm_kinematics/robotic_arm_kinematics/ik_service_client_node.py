#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from numpy import pi
from robotic_arm_msgs.srv import IKSolver, GripperControl
from robotic_arm_msgs.msg import WorldObjectInference
import time
from functools import partial

DELAY = 5
TRAJECTORY_HEIGHT = 1000
# Side approach offset - distance to move away from object before approaching
SIDE_APPROACH_OFFSET = 50


class IKClientNode(Node):
    def __init__(self):
        super().__init__('ik_client_node')
        
        # Object pos, for testing, tobe removed
        self.object_position = [190, -190, 100]
 
        # List to store previous detected objects and their positions in the scene
        self.previous_positions = []
    
        # Z Position of the gripper when placing an object
        self.placing_coord_z = 200

        # Robot's home position
        # Robot's home position
        self.home_pos = [13, -241, 292]

        # List to store positions of the cylinders, cubes, and hexagons containers respectively
        self.cylinders_container_pos = [-242, 20, self.placing_coord_z]
        self.cubes_container_pos = [-150, -230, self.placing_coord_z]
        self.hexagons_container_pos = [ -150, 20, self.placing_coord_z]

        # Gripper States and their corresponding angles
        self.GRIPPER_OPEN = pi/3
        self.GRIPPER_CLOSE = pi/6

        # List to store the variable of where the object picked and is to be placed
        self.start_position = self.home_pos
        self.end_position = self.hexagons_container_pos

        # Subscriber to the worldObjectInference topic
        self.pos_subscriber = self.create_subscription(
            WorldObjectInference,
            '/world_object_inference',
            self.object_inference_callback,
            10)

        # Gripper control service client
        self.gripper_client = self.create_client(GripperControl, 'gripper_control')
        
        # self.timer = self.create_timer(5.0, self.timer_callback)
        

    def timer_callback(self):
        self.pick_and_place_object()
        self.get_logger().info("Ready to kill timer")
        self.timer.cancel()


    # Service client setup
    def service_client_setup(self, x, y, z, gripper_state):
        # Service client to the IK solver service
        self.client_ = self.create_client(IKSolver, 'ik_server')
        while not self.client_.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('service not available, waiting again...')

        self.request = IKSolver.Request()
        self.request.x_coord = int(x)
        self.request.y_coord = int(y)
        self.request.z_coord = int(z)
        self.request.gripper_state = gripper_state

        self.future_ = self.client_.call_async(self.request)
        self.future_ = self.future_.add_done_callback(partial(self.response_callback))


    # Callback function for the subscriber
    def object_inference_callback(self, msg):
        class_names = msg.class_names
        object_positions = msg.detected_obj_positions

        # Construct a list of lists from the class names and object positions
        objects_and_positions = self.construct_2d_list(class_names, object_positions)
        

        self.get_logger().info(f"Received class names: {class_names}")
        self.get_logger().info(f"Received object positions: {objects_and_positions}")
        self.get_logger().info(f"Previous object positions: {self.previous_positions}")

        # If the object positions are the same as the previous positions, do nothing 
        if  self.compare_lists(objects_and_positions, self.previous_positions) == True:
            self.get_logger().info("No new objects detected")
        elif self.compare_lists(objects_and_positions, self.previous_positions) == False:
            self.get_logger().info("Object positions changed")
            self.previous_positions = objects_and_positions
            object_to_pick = self.first_object_to_pick_determiner(self.previous_positions)
            class_name = object_to_pick[0]
            x = float(object_to_pick[1])
            y = float(object_to_pick[2])
            z = float(object_to_pick[3])

            self.object_position = [x, y, z]

            # Determine the end position based on the class name
            if class_name == "Cylinder":
                self.end_position = self.cylinders_container_pos
            elif class_name == "Cube":
                self.end_position = self.cubes_container_pos
            elif class_name == "Hexagon":
                self.end_position = self.hexagons_container_pos
            else:
                self.end_position = self.home_pos

            # Remove the object from the list
            self.previous_positions.pop(self.previous_positions.index(object_to_pick))
            
            # Pick and place the object
            self.pick_and_place_object()
        else:
            self.get_logger().warn("No Object to Pick")


    # Callback function for the service client
    def response_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f"Received joint angles: {response.joint_angles}")
        except Exception as e:
            self.get_logger().error(f"Service call failed {e}")


    # Function to open and close the gripper
    def open_close_gripper(self, gripper_state: bool):
        if gripper_state == True:
            print("Opening gripper")
            return self.GRIPPER_OPEN
        else:
            print("Closing gripper")
            return self.GRIPPER_CLOSE
        

    # Function to decide what Object to pick first
    def first_object_to_pick_determiner(self, objects_list):
        if len(objects_list) == 0:
            return None
        else:
            x_closest = objects_list[0][1]
            object_to_pick = objects_list[0]
            # Pick object closest to the robot's base position wrt x axis
            for object in range(len(objects_list)):
                if objects_list[object][1] < x_closest:
                    x_closest = objects_list[object][1]
                    object_to_pick = objects_list[object]

        return object_to_pick
            
    # Define the pick and place logic with side approach and gripper attach/detach
    def pick_and_place_object(self):
        # Step 1: Open gripper and move to home position
        self.get_logger().info("Step 1: Moving to home with gripper open")
        self.service_client_setup(self.home_pos[0], self.home_pos[1], self.home_pos[2], True)  # True = OPEN
        time.sleep(DELAY)
        
        # Step 2: Move to side approach position - offset from object position
        side_approach_x = self.object_position[0] - SIDE_APPROACH_OFFSET
        side_approach_y = self.object_position[1]
        side_approach_z = self.object_position[2]
        self.get_logger().info("Step 2: Moving to side approach position")
        self.service_client_setup(side_approach_x, side_approach_y, side_approach_z, True)  # True = OPEN
        time.sleep(DELAY)
        
        # Step 3: Approach object from the side horizontally
        self.get_logger().info("Step 3: Approaching object from the side")
        self.service_client_setup(self.object_position[0], self.object_position[1], self.object_position[2], True)  # True = OPEN
        time.sleep(DELAY)
        
        # Step 4: Close gripper to pick up object - SLOWER for better grip
        self.get_logger().info("Step 4: CLOSING gripper to grip object")
        self.service_client_setup(self.object_position[0], self.object_position[1], self.object_position[2], False)  # False = CLOSE
        time.sleep(DELAY + 1)  # Wait for gripper to close
        
        # Step 4.5: ATTACH object to gripper using the attach service
        self.get_logger().info("Step 4.5: Attaching object to gripper")
        attach_success = self.control_gripper(attach=True)
        if not attach_success:
            self.get_logger().warn("Failed to attach object - continuing anyway")
        time.sleep(1)  # Brief pause after attachment
        
        # Step 5: Back away slightly to side approach position with object
        self.get_logger().info("Step 5: Backing away from object position")
        self.service_client_setup(side_approach_x, side_approach_y, side_approach_z, False)  # False = KEEP CLOSED
        time.sleep(DELAY)
        
        # Step 6: Lift object to trajectory height
        self.get_logger().info("Step 6: Lifting object to trajectory height")
        self.service_client_setup(side_approach_x, side_approach_y, side_approach_z + TRAJECTORY_HEIGHT, False)  # False = KEEP CLOSED
        time.sleep(DELAY)
        
        # Step 7: Move to placement position at trajectory height
        self.get_logger().info("Step 7: Moving to placement area")
        self.service_client_setup(self.end_position[0], self.end_position[1], self.end_position[2] + TRAJECTORY_HEIGHT, False)  # False = KEEP CLOSED
        time.sleep(DELAY)
        
        # Step 8: Lower to placement position
        self.get_logger().info("Step 8: Lowering to placement position")
        self.service_client_setup(self.end_position[0], self.end_position[1], self.end_position[2], False)  # False = KEEP CLOSED
        time.sleep(DELAY)
        
        # Step 8.5: DETACH object from gripper using the detach service
        self.get_logger().info("Step 8.5: Detaching object from gripper")
        detach_success = self.control_gripper(attach=False)
        if not detach_success:
            self.get_logger().warn("Failed to detach object - continuing anyway")
        time.sleep(1)  # Brief pause after detachment
        
        # Step 9: Open gripper to release object
        self.get_logger().info("Step 9: OPENING gripper to release object")
        self.service_client_setup(self.end_position[0], self.end_position[1], self.end_position[2], True)  # True = OPEN
        time.sleep(DELAY)
        
        # Step 10: Return to home position with gripper open
        self.get_logger().info("Step 10: Returning to home position")
        self.service_client_setup(self.home_pos[0], self.home_pos[1], self.home_pos[2], True)  # True = OPEN
        time.sleep(DELAY)
        
        self.get_logger().info("Side approach pick and place sequence with gripper attach/detach completed!")


    # Function to compare two lists
    def compare_lists(self, list1, list2):
        # List1 and list2 are lists of lists
        # If the lists are of the same length, compare the elements
        if len(list1) == 0:
            return None
        else:
            if len(list1) != len(list2):
                return False
            else:
                # If the lists are of the same length, compare the first elements of each list
                for i in range(len(list1)):
                    if list1[i][0] != list2[i][0]:
                        return False
                    else:
                        continue
                
            return True
        
            

    # Function to construct a list of lists from 2 lists
    def construct_2d_list(self, list1, list2):
        list_of_lists = []

        for i in range(len(list1)):
            list_of_lists.append([list1[i], list2[i * 3], list2[i * 3 + 1], list2[i * 3 + 2]])

        return list_of_lists
    
    # Function to control gripper attach/detach
    def control_gripper(self, attach: bool, object_name: str = ""):
        """Call the gripper control service to attach or detach objects"""
        try:
            if not self.gripper_client.wait_for_service(timeout_sec=2.0):
                self.get_logger().warn('Gripper control service not available')
                return False
            
            request = GripperControl.Request()
            request.attach = attach
            request.object_name = object_name
            
            future = self.gripper_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            
            if future.result():
                response = future.result()
                if response.success:
                    self.get_logger().info(f"Gripper control: {response.message}")
                    return True
                else:
                    self.get_logger().warn(f"Gripper control failed: {response.message}")
                    return False
            else:
                self.get_logger().error("Gripper control service call failed")
                return False
                
        except Exception as e:
            self.get_logger().error(f"Error calling gripper control: {str(e)}")
            return False
    


def main():
    rclpy.init(args=None)
    ik_client_node = IKClientNode()
    rclpy.spin(ik_client_node)
    ik_client_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()