#!/usr/bin/env python3

import math

import cv2
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from sensor_msgs.msg import Image


MIN_MARKER_SIDE_PX = 166.0


class ImageProcessorNode(Node):
    def __init__(self):
        super().__init__('aruco_image_detector_py')

        self.bridge = CvBridge()
        self.dictionary = cv2.aruco.getPredefinedDictionary(
            cv2.aruco.DICT_ARUCO_ORIGINAL
        )
        self.detector_parameters = cv2.aruco.DetectorParameters_create()
        self.show_image = self.declare_parameter('show_image', True).value

        self.sub = self.create_subscription(
            Image, '/camera/image', self.image_callback, 10
        )
        self.pub = self.create_publisher(Image, 'marker_image', 10)

        if self.show_image:
            cv2.namedWindow('marker_image', cv2.WINDOW_AUTOSIZE)

        self.get_logger().info(
            'Listening on /camera/image; publishing annotated frames on marker_image'
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
            cv_image = self.bridge.imgmsg_to_cv2(
                msg, desired_encoding='bgr8'
            )
        except CvBridgeError as error:
            self.get_logger().error(f'Could not convert camera image: {error}')
            return

        gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        marker_corners, marker_ids, _ = cv2.aruco.detectMarkers(
            gray_image,
            self.dictionary,
            parameters=self.detector_parameters,
        )

        output_image = cv_image.copy()
        detected_markers = []
        if marker_ids is not None:
            for marker_id, corners in zip(marker_ids.flatten(), marker_corners):
                side_length = self._marker_side_length(corners)
                if side_length >= MIN_MARKER_SIDE_PX:
                    detected_markers.append((int(marker_id), corners))

        if detected_markers:
            center = (output_image.shape[1] // 2, output_image.shape[0] // 2)
            cv2.circle(output_image, center, 50, (0, 255, 0), 3)

            for marker_id, corners in detected_markers:
                outline = corners.reshape((4, 2)).round().astype('int32')
                cv2.polylines(
                    output_image,
                    [outline],
                    isClosed=True,
                    color=(0, 255, 0),
                    thickness=2,
                    lineType=cv2.LINE_AA,
                )

                x_min = int(outline[:, 0].min())
                x_max = int(outline[:, 0].max())
                y_min = int(outline[:, 1].min())
                y_max = int(outline[:, 1].max())
                label = f'ID: {marker_id}'
                font = cv2.FONT_HERSHEY_SIMPLEX
                scale = 0.7
                thickness = 2
                (text_width, text_height), baseline = cv2.getTextSize(
                    label, font, scale, thickness
                )

                text_x = max(
                    0,
                    min(
                        (x_min + x_max - text_width) // 2,
                        output_image.shape[1] - text_width,
                    ),
                )
                text_y = y_max + text_height + baseline + 5
                if text_y >= output_image.shape[0]:
                    text_y = y_min - 5
                text_y = max(text_height, min(text_y, output_image.shape[0] - 1))

                cv2.putText(
                    output_image,
                    label,
                    (text_x, text_y),
                    font,
                    scale,
                    (0, 255, 255),
                    thickness,
                    cv2.LINE_AA,
                )

        if self.show_image:
            cv2.imshow('marker_image', output_image)
            cv2.waitKey(1)

        output_msg = self.bridge.cv2_to_imgmsg(output_image, encoding='bgr8')
        output_msg.header = msg.header
        self.pub.publish(output_msg)

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
