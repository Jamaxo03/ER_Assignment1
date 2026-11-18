#!/usr/bin/env python3

import math

import cv2
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage
from assignment_interfaces.msg import Marker, MarkerArray


class ImageProcessorNode(Node):
    def __init__(self):
        super().__init__('aruco_image_detector_py')

        self.bridge = CvBridge() 

        # dictionary for detecting ArUco markers correctly
        self.dictionary = cv2.aruco.getPredefinedDictionary(
            cv2.aruco.DICT_ARUCO_ORIGINAL
        )

        # detector with default parameters, to obtain marker corners and ids.
        if hasattr(cv2.aruco, 'ArucoDetector'):
            detector = cv2.aruco.ArucoDetector(self.dictionary, cv2.aruco.DetectorParameters())
            self.detect_markers = detector.detectMarkers
        else:
            parameters = cv2.aruco.DetectorParameters_create()
            self.detect_markers = lambda image: cv2.aruco.detectMarkers(
                image, self.dictionary, parameters=parameters)

        # ROS parameters: defaults = simulation, overridden by config/sim.yaml or config/rosbot.yaml
        image_topic = self.declare_parameter('image_topic', '/camera/image').value
        marker_array_topic = self.declare_parameter('marker_array_topic', '/marker_array').value
        compressed_topic = image_topic + '/compressed'

        # reliable (Gazebo bridge) or best effort (ROSbot), this subscriber matches both
        self.sub = self.create_subscription(
            CompressedImage, compressed_topic, self.image_callback, qos_profile_sensor_data)

        self.pub_marker = self.create_publisher(MarkerArray, marker_array_topic, 10)

        self.get_logger().info(
            f'Listening on {compressed_topic}, publishing on {marker_array_topic}')

    @staticmethod
    def _marker_side_length(corners):
        points = corners.reshape((4, 2))
        side_lengths = []
        for index in range(4):
            start = points[index]
            end = points[(index + 1) % 4]
            side_lengths.append(
                math.hypot(
                    float(end[0] - start[0]),
                    float(end[1] - start[1]),
                )
            )
        return sum(side_lengths) / len(side_lengths)

    def image_callback(self, msg):
        try:
            # converting image from ROS Image message to OpenCV image (matrix format)
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8') 
        except CvBridgeError as error:
            self.get_logger().error(f'Could not convert camera image: {error}')
            return

        # convert the image to grayscale for marker detection
        gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        marker_corners, marker_ids, _ = self.detect_markers(gray_image)

        # all the markers seen in this frame (empty list = no marker in view)
        observations = MarkerArray()
        observations.header = msg.header

        if marker_ids is not None:
            for marker_id, corners in zip(marker_ids.flatten(), marker_corners):
                # smallest circle containing the 4 marker corners (as in example_ball.py)
                ((x, y), radius) = cv2.minEnclosingCircle(corners.reshape((4, 2)))

                # pixel features for the IBVS
                marker = Marker()
                marker.id = int(marker_id)
                # centre
                marker.u = float(x) 
                marker.v = float(y)
                # pixel size
                marker.side = float(self._marker_side_length(corners))

                observations.markers.append(marker)
                    
        # published on every frame, also when empty
        self.pub_marker.publish(observations)

def main(args=None):
    rclpy.init(args=args)
    node = ImageProcessorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()