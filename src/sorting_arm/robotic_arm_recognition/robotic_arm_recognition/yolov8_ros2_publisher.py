#!/usr/bin/env python3

from ultralytics import YOLO
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2.aruco as aruco
import cv2
import numpy as np
from ament_index_python.packages import get_package_share_directory
import os

from robotic_arm_msgs.msg import ObjectInference
from robotic_arm_msgs.msg import Yolov8Inference


# Define the picking height of the robotic arm
PICKING_HEIGHT = 50

# Define the x-axis drift of the robotic arm
ARUCO_MARKER_SIZE_MM = 34
Y_DRIFT = 300 - int(ARUCO_MARKER_SIZE_MM / 2)

class Camera_subscriber(Node):

    def __init__(self):
        super().__init__('camera_subscriber')

        self.cv2_bridge = CvBridge()

        self.aruco_dict_type = aruco.DICT_4X4_50                
        self.aruco_marker_size_mm = ARUCO_MARKER_SIZE_MM   
        location_path = os.path.join(get_package_share_directory("robotic_arm_recognition"), "Computer_Vision_Models", "5best.pt")

        self.model = YOLO(location_path)

        self.yolov8_inference = Yolov8Inference()
        self.object_inference = ObjectInference()

        self.camera_matrix = np.array([[650, 0, 320], [0, 650, 240], [0, 0, 1]], dtype=np.float32)
        self.dist_coeffs = np.zeros((4, 1), dtype=np.float32)

        # Simulation mode parameters
        self.use_simulation_mode = True  
        self.sim_pixel_to_mm_ratio = 0.8  
        self.sim_reference_center = np.array([320, 240])  
        
        # Table center offset in robot base frame
        self.table_center_x_offset = 0    
        self.table_center_y_offset = -300  

        # Robot base filtering parameters - RELAXED VALUES
        self.robot_base_exclusion_zone = {
            'x_min': -50,   # mm - smaller exclusion zone
            'x_max': 50,    # mm 
            'y_min': -50,   # mm - smaller exclusion zone  
            'y_max': 100,   # mm - allow objects further from base
        }
        
        # Object size filtering - RELAXED VALUES
        self.min_object_size_pixels = 30    # Smaller minimum
        self.max_object_size_pixels = 8000  # Larger maximum
        
        # Reachability zone - EXPANDED VALUES
        self.reachable_zone = {
            'x_min': -600,  # mm - expanded reach
            'x_max': 600,   # mm - expanded reach
            'y_min': -600,  # mm - expanded reach
            'y_max': -100,  # mm - allow closer objects
        }

        self.subscription = self.create_subscription(
            Image,
            'rgb_cam/image_raw',
            self.camera_call_back,  # Use the filtering callback
            10)
        

        self.yolov8_pub = self.create_publisher(Yolov8Inference, "/Yolov8_Inference", 1)
        self.img_pub = self.create_publisher(Image, "/object_inference", 1)

    def camera_call_back(self, data):
        self.get_logger().info('Receiving video frame')         # Output log information indicating that the callback function has been entered
        image = self.cv2_bridge.imgmsg_to_cv2(data, 'bgr8')     # Convert the ROS image message to an OpenCV image
        self.detect_and_position_object(image)                  # Object detection and positioning

    # Detect the ArUco marker and get the reference center
    def detect_aruco_and_get_reference_center(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        dictionary = aruco.getPredefinedDictionary(self.aruco_dict_type)
        parameters = aruco.DetectorParameters()
        markerCorners, markerIds, _ = aruco.detectMarkers(gray, dictionary, parameters=parameters)

        reference_center = None
        pixel_to_mm_ratio = None
        centers = {}

        if markerCorners and markerIds is not None:
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(markerCorners, self.aruco_marker_size_mm,
                                                             self.camera_matrix, self.dist_coeffs)
            for idx, corners in enumerate(markerCorners):
                int_corners = np.int0(corners)
                center = np.mean(int_corners[0], axis=0)
                centers[int(markerIds[idx])] = center
                if markerIds[idx] == 1:
                    reference_center = center
                    aruco_side_pixels = np.linalg.norm(int_corners[0][0] - int_corners[0][1])
                    pixel_to_mm_ratio = self.aruco_marker_size_mm / aruco_side_pixels

                cv2.drawFrameAxes(frame, self.camera_matrix, self.dist_coeffs, rvecs[idx], tvecs[idx],
                                  self.aruco_marker_size_mm / 2)

        aruco.drawDetectedMarkers(frame, markerCorners, markerIds)
        return reference_center, pixel_to_mm_ratio, centers

    # Function to detect and position objects    
    def detect_and_position_object(self, image):
        reference_center, pixel_to_mm_ratio, aruco_centers = self.detect_aruco_and_get_reference_center(image)

        results = self.model(image)

        if reference_center is not None and pixel_to_mm_ratio is not None:
            for r in results:
                boxes = r.boxes
                for box in boxes:
                    self.object_inference = ObjectInference()

                    # Coordinates of the bounding box
                    x1, y1, x2, y2 = map(int, box.xyxy[0])    
                    # Class of the detected object     
                    c = box.cls

                    # Calculate bounding box area for filtering
                    bounding_box_area = (x2 - x1) * (y2 - y1)

                    # x_origin, y_origin, aruco
                    x_origin = reference_center[1] * pixel_to_mm_ratio
                    y_origin = reference_center[0] * pixel_to_mm_ratio

                    # Calculate the centres of the bounding box
                    x_center = (x1 + x2) / 2
                    y_center = (y1 + y2) / 2

                    x_center = x_center * pixel_to_mm_ratio
                    y_center = y_center * pixel_to_mm_ratio

                    x_pos = int(x_center - x_origin)
                    y_pos = int(y_center - y_origin)
                    

                    self.object_inference.class_name = self.model.names[int(c)]
                    self.object_inference.x_coord = x_pos
                    self.object_inference.y_coord = y_pos
                    self.object_inference.z_coord = PICKING_HEIGHT

                    # Apply filtering to exclude robot base and unreachable objects
                    if self.should_filter_object(x_pos, y_pos, bounding_box_area):
                        # Draw filtered objects in red with "FILTERED" label
                        cv2.putText(image, "FILTERED", (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                        cv2.rectangle(image, (x1, y1), (x2, y2), (0, 0, 255), 2)
                        continue  # Skip this object

                    if self.object_inference.class_name == "Cylinder" or self.object_inference.class_name == "Cube" or self.object_inference.class_name == "Hexagon":
                        self.yolov8_inference.class_names.append(self.object_inference.class_name)
                        self.yolov8_inference.detected_obj_positions.append(str(self.object_inference.x_coord))
                        self.yolov8_inference.detected_obj_positions.append(str(self.object_inference.y_coord))
                        self.yolov8_inference.detected_obj_positions.append(str(self.object_inference.z_coord))

                        cls = int(c)
                        currentClass = self.object_inference.class_name
                        myColor = (0, 255, 0)  # Green for valid objects
                        cv2.putText(image, f"{currentClass} VALID", (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, myColor, 2)
                        cv2.putText(image, f'({x_pos} mm, {y_pos} mm)', (x1, y1 + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, myColor, 2)
                        cv2.rectangle(image, (x1, y1), (x2, y2), myColor, 2)

            annotated_frame = results[0].plot()
            img_msg = self.cv2_bridge.cv2_to_imgmsg(annotated_frame)  

            self.img_pub.publish(img_msg)
            self.yolov8_pub.publish(self.yolov8_inference)
            self.yolov8_inference.class_names.clear()
            self.yolov8_inference.detected_obj_positions.clear()

        # Simulation mode: use predefined center and pixel-to-mm ratio
        elif self.use_simulation_mode:
            self.get_logger().info("Using simulation mode (no ArUco marker detected)")
            for r in results:
                boxes = r.boxes
                for box in boxes:
                    self.object_inference = ObjectInference()

                    # Coordinates of the bounding box
                    x1, y1, x2, y2 = map(int, box.xyxy[0])    
                    # Class of the detected object     
                    c = box.cls

                    # Calculate bounding box area for filtering
                    bounding_box_area = (x2 - x1) * (y2 - y1)

                    # Calculate actual object center from bounding box
                    x_center_pixels = (x1 + x2) / 2
                    y_center_pixels = (y1 + y2) / 2

                    # Convert to coordinates relative to image center
                    x_relative = x_center_pixels - self.sim_reference_center[0]
                    y_relative = y_center_pixels - self.sim_reference_center[1]

                    # Convert to millimeters and apply offsets
                    x_pos = int(x_relative * self.sim_pixel_to_mm_ratio + self.table_center_x_offset)
                    y_pos = int(-y_relative * self.sim_pixel_to_mm_ratio + self.table_center_y_offset)  # Negative Y for proper coordinate system
                    
                    self.object_inference.class_name = self.model.names[int(c)]
                    self.object_inference.x_coord = x_pos
                    self.object_inference.y_coord = y_pos
                    self.object_inference.z_coord = PICKING_HEIGHT

                    # Apply filtering to exclude robot base and unreachable objects
                    if self.should_filter_object(x_pos, y_pos, bounding_box_area):
                        # Draw filtered objects in red with "FILTERED" label
                        cv2.putText(image, "FILTERED", (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                        cv2.rectangle(image, (x1, y1), (x2, y2), (0, 0, 255), 2)
                        continue  # Skip this object

                    self.get_logger().info(f"Valid simulation object: {self.object_inference.class_name} at ({x_pos}, {y_pos}, {PICKING_HEIGHT})")

                    if self.object_inference.class_name == "Cylinder" or self.object_inference.class_name == "Cube" or self.object_inference.class_name == "Hexagon":
                        self.yolov8_inference.class_names.append(self.object_inference.class_name)
                        self.yolov8_inference.detected_obj_positions.append(str(self.object_inference.x_coord))
                        self.yolov8_inference.detected_obj_positions.append(str(self.object_inference.y_coord))
                        self.yolov8_inference.detected_obj_positions.append(str(self.object_inference.z_coord))

                        cls = int(c)
                        currentClass = self.object_inference.class_name
                        myColor = (0, 255, 0)  # Green for valid simulation objects
                        cv2.putText(image, f"{currentClass} VALID", (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, myColor, 2)
                        cv2.putText(image, f'({x_pos} mm, {y_pos} mm)', (x1, y1 + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, myColor, 2)
                        cv2.rectangle(image, (x1, y1), (x2, y2), myColor, 2)

            annotated_frame = results[0].plot()
            img_msg = self.cv2_bridge.cv2_to_imgmsg(annotated_frame)  

            self.img_pub.publish(img_msg)
            self.yolov8_pub.publish(self.yolov8_inference)
            self.yolov8_inference.class_names.clear()
            self.yolov8_inference.detected_obj_positions.clear()
        
        else:
            self.get_logger().warn("No ArUco marker detected and simulation mode disabled")

    # Function to check if an object should be filtered out (robot base or unreachable)
    def should_filter_object(self, x_pos, y_pos, bounding_box_area):
        """
        Filter out objects that are:
        1. Too close to robot base (likely robot parts)
        2. Outside reachable zone
        3. Too large or too small (likely robot base or noise)
        """
        
        self.get_logger().info(f"Checking object at ({x_pos}, {y_pos}) with size {bounding_box_area} pixels")
        
        # Check if object is in robot base exclusion zone
        if (self.robot_base_exclusion_zone['x_min'] <= x_pos <= self.robot_base_exclusion_zone['x_max'] and
            self.robot_base_exclusion_zone['y_min'] <= y_pos <= self.robot_base_exclusion_zone['y_max']):
            self.get_logger().info(f"FILTERED: Object at ({x_pos}, {y_pos}) - too close to robot base")
            return True
        
        # Check if object is outside reachable zone
        if not (self.reachable_zone['x_min'] <= x_pos <= self.reachable_zone['x_max'] and
                self.reachable_zone['y_min'] <= y_pos <= self.reachable_zone['y_max']):
            self.get_logger().info(f"FILTERED: Object at ({x_pos}, {y_pos}) - outside reachable zone")
            self.get_logger().info(f"Reachable zone: X({self.reachable_zone['x_min']} to {self.reachable_zone['x_max']}), Y({self.reachable_zone['y_min']} to {self.reachable_zone['y_max']})")
            return True
        
        # Check object size (filter out robot base which is typically large)
        if not (self.min_object_size_pixels <= bounding_box_area <= self.max_object_size_pixels):
            self.get_logger().info(f"FILTERED: Object at ({x_pos}, {y_pos}) - size {bounding_box_area} pixels outside valid range ({self.min_object_size_pixels}-{self.max_object_size_pixels})")
            return True
        
        self.get_logger().info(f"VALID: Object at ({x_pos}, {y_pos}) passed all filters")
        return False


if __name__ == '__main__':
    rclpy.init(args=None)
    camera_subscriber = Camera_subscriber()
    rclpy.spin(camera_subscriber)
    rclpy.shutdown()