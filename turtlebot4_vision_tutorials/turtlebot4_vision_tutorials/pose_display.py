#!/usr/bin/env python3

# Copyright 2024 Clearpath Robotics, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# @author Hilary Luo (hluo@clearpathrobotics.com)

import cv2
import numpy as np

from cv_bridge import CvBridge
from geometry_msgs.msg import PoseArray
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String as string_msg
from sensor_msgs.msg import Image

from turtlebot4_vision_tutorials.MovenetDepthaiEdge import Body

SCORE_THRESH = 0.3
LINES_BODY = [[4, 2], [2, 0], [0, 1], [1, 3],
              [10, 8], [8, 6], [6, 5], [5, 7], [7, 9],
              [6, 12], [12, 11], [11, 5],
              [12, 14], [14, 16], [11, 13], [13, 15]]


class PoseDetection(Node):
    lights_on_ = False
    frame = None
    body = Body()

    def __init__(self):
        super().__init__('pose_display')

        self.output = None

        # Subscribe to the /semaphore_flags topic
        self.semaphore_flag_subscriber = self.create_subscription(
            string_msg,
            'semaphore_flag',
            self.semaphore_flag_callback,
            qos_profile_sensor_data)

        # Subscribe to the ffmpeg_decoded topic
        self.body_pose_subscriber = self.create_subscription(
            PoseArray,
            'body_pose',
            self.body_pose_callback,
            qos_profile_sensor_data)

        # Subscribe to the ffmpeg_decoded topic
        self.ffmpeg_subscriber = self.create_subscription(
            Image,
            'oakd/rgb/preview/ffmpeg_decoded',
            self.frame_callback,
            qos_profile_sensor_data)

        self.bridge = CvBridge()

    def semaphore_flag_callback(self, letter_msg: string_msg):
        # self.get_logger().info('semaphore_flag_callback')
        if self.frame is None:
            return
        if letter_msg.data:
            cv2.putText(self.frame,
                        letter_msg.data,
                        (self.frame.shape[1] // 2, 100),
                        cv2.FONT_HERSHEY_PLAIN,
                        5,
                        (0, 190, 255),
                        3)

    def body_pose_callback(self, pose_msg: PoseArray):
        # self.get_logger().info('body_pose_callback')
        temp_keypoints = []
        temp_scores = []
        for i, point in enumerate(pose_msg.poses):
            temp_keypoints.append((int(point.position.x), int(point.position.y)))
            temp_scores.append(point.position.z)
        self.body.keypoints = np.array(temp_keypoints)
        self.body.scores = np.array(temp_scores)

    def frame_callback(self, image_msg: Image):
        # self.get_logger().info('frame_callback')
        if image_msg.data is None:
            return
        self.frame = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
        if self.body.keypoints is not None:
            self.draw()
        self.waitKey()

    def draw(self):
        lines = [np.array([self.body.keypoints[point] for point in line])
                 for line in LINES_BODY if self.body.scores[line[0]] > SCORE_THRESH and
                 self.body.scores[line[1]] > SCORE_THRESH]
        if lines is not None:
            cv2.polylines(self.frame, lines, False, (255, 180, 90), 2, cv2.LINE_AA)

        for i, x_y in enumerate(self.body.keypoints):
            if self.body.scores[i] > SCORE_THRESH:
                if i % 2 == 1:
                    color = (0, 255, 0)
                elif i == 0:
                    color = (0, 255, 255)
                else:
                    color = (0, 0, 255)
                cv2.circle(self.frame, (x_y[0], x_y[1]), 4, color, -11)

    def waitKey(self, delay=1):
        # if self.show_fps:
        #        self.pose.fps.draw(self.frame, orig=(50,50), size=1, color=(240,180,100))
        cv2.imshow("Movenet", self.frame)
        if self.output:
            self.output.write(self.frame)
        key = cv2.waitKey(delay)
        if key == 32:
            # Pause on space bar
            cv2.waitKey(0)
        elif key == ord('f'):
            self.show_fps = not self.show_fps
        elif key == ord('c'):
            self.show_crop = not self.show_crop
        return key


def main(args=None):
    rclpy.init(args=args)
    node = PoseDetection()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
