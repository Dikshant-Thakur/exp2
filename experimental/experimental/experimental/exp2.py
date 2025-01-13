import sys
import time
import numpy as np
import cv2
import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, CompressedImage
from std_msgs.msg import String, Int32


class ArucoNode(Node):
    def __init__(self):
        super().__init__('assignment2_aruco_node')

        # Initialize parameters
        self.declare_parameter("aruco_dictionary_id", "DICT_ARUCO_ORIGINAL")
        self.declare_parameter("camera_info_topic", "/camera/camera_info")

        # Fetch parameter values
        dictionary_id_name = self.get_parameter("aruco_dictionary_id").get_parameter_value().string_value
        info_topic = self.get_parameter("camera_info_topic").get_parameter_value().string_value

        # Set up aruco dictionary
        try:
            dictionary_id = cv2.aruco.__getattribute__(dictionary_id_name)
            if not isinstance(dictionary_id, int):
                raise AttributeError
        except AttributeError:
            self.get_logger().error(f"Invalid aruco_dictionary_id: {dictionary_id_name}")
            valid_options = "\n".join([s for s in dir(cv2.aruco) if s.startswith("DICT")])
            self.get_logger().error(f"Valid options: {valid_options}")
            return

        # Subscriptions and Publishers
        self.dynamic_goal = self.create_publisher(String, '/dynamic_goal_topic', 10)
        self.marker_id_unique = self.create_publisher(Int32, '/MARKER_ID', 10 )


        self.info_sub = self.create_subscription(CameraInfo, info_topic, self.info_callback, qos_profile_sensor_data)
        self.create_subscription(CompressedImage, '/camera/image_raw/compressed', self.compressed_callback, qos_profile_sensor_data)


        # Camera parameters
        self.info_msg = None
        self.intrinsic_mat = None
        self.distortion = None

        # Aruco detector settings
        self.aruco_dictionary = cv2.aruco.Dictionary_get(dictionary_id)
        self.aruco_parameters = cv2.aruco.DetectorParameters_create()

        # Marker storage
        self.detected_markers = []  #List

    def compressed_callback(self, compressed_data):
        np_arr = np.frombuffer(compressed_data.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        # Detect ArUco markers
        corners, marker_ids, rejected = cv2.aruco.detectMarkers(image_np, self.aruco_dictionary, parameters=self.aruco_parameters)

        if marker_ids is not None:
            for i, marker_id in enumerate(marker_ids.flatten()):
                if marker_id == 0:
                    continue
                if marker_id not in self.detected_markers:  # Check if the marker is new
                    self.detected_markers.append(marker_id)  # Add to the set
                    self.get_logger().info(f"Detected new Marker ID: {marker_id}")
                    self.get_logger().info(f"Total unique markers detected: {len(self.detected_markers)}")
                    print(f"All Detected Marker IDs so far: {list(self.detected_markers)}")
                    self.marker_id_unique.publish(Int32(data=int(marker_id)))

                # Draw marker boundaries and ID on the image
                cv2.polylines(image_np, [np.int32(corners[i])], True, (0, 255, 0), 2)
                cv2.putText(image_np, f"ID: {marker_id}",
                            (int(corners[i][0][0][0]), int(corners[i][0][0][1] - 10)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # Check if at least one marker is detected
        if len(self.detected_markers) == 4:
            min_marker_id = min(self.detected_markers)
            self.get_logger().info(f"Smallest Marker ID: {min_marker_id}")
            
            # Publish the index of the smallest marker ID
            marker_index = self.detected_markers.index(min_marker_id) + 1
            self.dynamic_goal.publish(String(data=f"wp{marker_index}"))
            self.get_logger().info(f"Published waypoint: wp{marker_index}")

        # Display the image
        cv2.imshow("ArUco Markers", image_np)
        cv2.waitKey(1)

    def info_callback(self, info_msg):
        self.info_msg = info_msg
        self.intrinsic_mat = np.reshape(np.array(info_msg.k), (3, 3))
        self.distortion = np.array(info_msg.d)
        self.destroy_subscription(self.info_sub)


def main():
    rclpy.init()
    node = ArucoNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

