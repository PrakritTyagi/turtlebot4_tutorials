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
from functools import partial
import numpy as np

from cv_bridge import CvBridge
from geometry_msgs.msg import PoseArray
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String as string_msg
from sensor_msgs.msg import BatteryState, Image

from turtlebot4_vision_tutorials.MovenetDepthaiEdge import Body

SCORE_THRESH = 0.3
LINES_BODY = [[4, 2], [2, 0], [0, 1], [1, 3],
              [10, 8], [8, 6], [6, 5], [5, 7], [7, 9],
              [6, 12], [12, 11], [11, 5],
              [12, 14], [14, 16], [11, 13], [13, 15]]

class PoseDisplay(Node):
    lights_on_ = False
    frame = [ None, None, None, None, None, None, None]
    body = [ None, None, None, None, None, None, None]
    percentage = [ None, None, None, None, None, None, None]


    def __init__(self):
        super().__init__('pose_display')
        self.declare_parameter('tile_x', 3)
        self.declare_parameter('tile_y', 2)
        self.declare_parameter('namespaces', ['tb11', 'tb12'])
        self.declare_parameter('image_height', 432)
        self.declare_parameter('image_width', 768)

        self.tile_x = self.get_parameter('tile_x').get_parameter_value().integer_value
        self.tile_y = self.get_parameter('tile_y').get_parameter_value().integer_value
        self.namespaces = self.get_parameter('namespaces').get_parameter_value().string_array_value
        self.image_height = self.get_parameter('image_height').get_parameter_value().integer_value
        self.image_width = self.get_parameter('image_width').get_parameter_value().integer_value

        self.full_frame = np.zeros((self.image_height*self.tile_y,
                                    self.image_width*self.tile_x, 3), np.uint8)

        self.output = None

        # # Subscribe to the /semaphore_flags topic
        # self.semaphore_flag_subscriber = self.create_subscription(
        #     string_msg,
        #     'semaphore_flag',
        #     self.semaphore_flag_callback,
        #     qos_profile_sensor_data)

        self.body_pose_subscribers = []
        self.ffmpeg_subscribers = []
        self.battery_subscribers = []

        # Naming a window
        cv2.namedWindow("Movenet", cv2.WINDOW_NORMAL)

        # Subscribe to the pose topics
        for i, ns in enumerate(self.namespaces):
            subscriber = self.create_subscription(
                PoseArray,
                f'/{ns}/body_pose',
                partial(self.body_pose_callback, num = i),
                qos_profile_sensor_data)
            self.body_pose_subscribers.append(subscriber)

            # Subscribe to the ffmpeg_decoded topics
            subscriber = self.create_subscription(
                Image,
                f'/{ns}/oakd/rgb/preview/ffmpeg_decoded',
                partial(self.frame_callback, num = i),
                qos_profile_sensor_data)
            self.ffmpeg_subscribers.append(subscriber)

            # Subscribe to the battery topics
            subscriber = self.create_subscription(
                BatteryState,
                f'/{ns}/battery_state',
                partial(self.battery_callback, num = i),
                qos_profile_sensor_data)
            self.battery_subscribers.append(subscriber)


        timer_period = 0.0833  # seconds
        self.timer = self.create_timer(timer_period, self.updateDisplay)

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

    def body_pose_callback(self, pose_msg: PoseArray, num: int):
        self.get_logger().info(f'Body_pose_callback {num} - start')
        temp_keypoints = []
        temp_scores = []
        for i, point in enumerate(pose_msg.poses):
            temp_keypoints.append((int(point.position.x), int(point.position.y)))
            temp_scores.append(point.position.z)
        b = Body()
        b.keypoints = np.array(temp_keypoints)
        b.scores = np.array(temp_scores)
        self.body[num] = b
        # self.get_logger().info(f'Body_pose_callback {num} - end')

    def frame_callback(self, image_msg: Image, num: int):
        self.get_logger().info(f'Frame_callback {num} - start')
        if image_msg.data is None:
            return
        self.frame[num] = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
        if self.body[num] is not None and self.body[num].keypoints is not None:
            self.draw(num)
        self.updateFrame(num)
        # self.get_logger().info(f'Frame_callback {num} - end')

    def draw(self, num: int):
        lines = [np.array([self.body[num].keypoints[point] for point in line])
                 for line in LINES_BODY if self.body[num].scores[line[0]] > SCORE_THRESH and
                 self.body[num].scores[line[1]] > SCORE_THRESH]
        if lines is not None:
            cv2.polylines(self.frame[num], lines, False, (255, 180, 90), 2, cv2.LINE_AA)

        for i, x_y in enumerate(self.body[num].keypoints):
            if self.body[num].scores[i] > SCORE_THRESH:
                if i % 2 == 1:
                    color = (0, 255, 0)
                elif i == 0:
                    color = (0, 255, 255)
                else:
                    color = (0, 0, 255)
                cv2.circle(self.frame[num], (x_y[0], x_y[1]), 4, color, -11)

    def updateFrame(self, num: int):
        self.get_logger().info(f'Updated frame {num}')
        x = num%self.tile_x
        y = int((num - x)/self.tile_y)
        cv2.putText(self.frame[num],
                        f'{self.namespaces[num]}',
                        (50, 50),
                        # (self.frame[num].shape[1] // 2, 100),
                        cv2.FONT_HERSHEY_PLAIN,
                        2,
                        (0, 0, 255),
                        2)
        if self.percentage[num]:
            cv2.putText(self.frame[num],
                        f'{self.percentage[num]:.1f}%',
                        (self.frame[num].shape[1] - 120, self.frame[num].shape[0] - 20),
                        cv2.FONT_HERSHEY_PLAIN,
                        2,
                        (0, 0, 255),
                        2)

        self.full_frame[
            int(y*self.image_height):int((y+1)*self.image_height),
            int(x*self.image_width):int((x+1)*self.image_width),
            0:3] = self.frame[num]

    def updateDisplay(self):
        self.get_logger().info(f'Updated display')
        cv2.imshow("Movenet", self.full_frame)
        cv2.waitKey(1)

    def waitKey(self, delay=0.1):
        # if self.show_fps:
        #        self.pose.fps.draw(self.frame, orig=(50,50), size=1, color=(240,180,100))
        cv2.imshow("Movenet", self.full_frame)
        if self.output:
            self.output.write(self.full_frame)
        key = cv2.waitKey(delay)
        if key == 32:
            # Pause on space bar
            cv2.waitKey(0)
        elif key == ord('f'):
            self.show_fps = not self.show_fps
        elif key == ord('c'):
            self.show_crop = not self.show_crop
        return key

    def battery_callback(self, batt_msg: BatteryState, num: int):
        self.percentage[num] = batt_msg.percentage * 100

def main(args=None):
    rclpy.init(args=args)
    node = PoseDisplay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
