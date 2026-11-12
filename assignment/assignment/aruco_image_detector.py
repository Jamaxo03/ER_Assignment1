#!/usr/bin/env python3

# change output message topic type: from Image to CameraInfo (not sure)
# change condition for publishing: always publish camera image with modficiations
# add condition checking if marker is within certain range (has specific size in pixels) to publish signal to control node
# make topic names and parameters configurable through launch file (ROS parameters) -> needed to run code on ROSbot

import math

import cv2
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Int32
from assignment_interfaces.msg import Marker, MarkerArray
#from rclpy.qos import qos_profile_sensor_data


MIN_MARKER_SIDE_PX = 166.0


class ImageProcessorNode(Node):
    def __init__(self):
        super().__init__('aruco_image_detector_py')

        self.bridge = CvBridge() 

        # becomes True after /marker_image has been published once
        self.marker_image_published = False

        # dictionary for detecting ArUco markers correctly
        self.dictionary = cv2.aruco.getPredefinedDictionary(
            cv2.aruco.DICT_ARUCO_ORIGINAL
        )

        # default detector parameters to obtain marker corners and ids
        self.detector_parameters = cv2.aruco.DetectorParameters_create()

        # parameter to control whether to show the annotated image in a window
        self.show_image = self.declare_parameter('show_image', True).value

        self.sub = self.create_subscription(
            CompressedImage, '/camera/image/compressed', self.image_callback, 10)
        
        self.pub = self.create_publisher(Image, '/marker_image', 10)
        self.pub_marker = self.create_publisher(MarkerArray, '/marker_array', 10)

        if self.show_image:
            # define name of window where topic frames will be displayed
            cv2.namedWindow('marker_image', cv2.WINDOW_AUTOSIZE)

        self.get_logger().info(
            'Listening on /camera/image/compressed; publishing annotated frames on marker_image'
        )

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
        marker_corners, marker_ids, _ = cv2.aruco.detectMarkers(
            gray_image,
            self.dictionary,
            parameters=self.detector_parameters,
        )

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

                if marker.side >= MIN_MARKER_SIDE_PX and not self.marker_image_published:
                    output_image = cv_image.copy()  # annotated copy (circle + id)

                    # drawing
                    center = (int(x), int(y))
                    cv2.circle(output_image, center, int(radius), (0, 255, 0), 3)
                    label_position = (center[0] - 30, max(center[1] - int(radius) - 10, 30))
                    cv2.putText(output_image, f'id {marker.id}', label_position, cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)

                    # converting the annotated OpenCV image back to ROS Image message and publishing it on the topic
                    output_msg = self.bridge.cv2_to_imgmsg(output_image, encoding='bgr8') 
                    output_msg.header = msg.header
                    self.pub.publish(output_msg)

                    self.marker_image_published = True
                    self.get_logger().info(f'Marker {marker.id} reached (side {marker.side:.0f} px): /marker_image published')

                    if self.show_image:
                        # display the annotated image in a window
                        cv2.imshow('marker_image', output_image)
                        

        # published on every frame, also when empty
        self.pub_marker.publish(observations)

        if self.show_image:
            cv2.waitKey(1)

    def destroy_node(self):
        if self.show_image:
            cv2.destroyWindow('marker_image')
        super().destroy_node()


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