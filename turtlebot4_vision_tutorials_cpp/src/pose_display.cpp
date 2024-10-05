// Copyright 2024 Clearpath Robotics, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// @author Hilary Luo (hluo@clearpathrobotics.com)

#include <chrono>
#include <string>

#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"

#include "geometry_msgs/msg/pose_array.hpp"
#include "image_transport/image_transport.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "sensor_msgs/msg/image.hpp"

#define IMAGE_HEIGHT 432
#define IMAGE_WIDTH 768
#define TILE_X 2
#define TILE_Y 3

#define SCORE_THRESH 0.4
#define LINE_NUM 16

#define NODE_NAME "pose_display"
#define WINDOW_NAME "Clearpath Turtlebot 4 Demo"
#define DISPLAY_PERIOD 83 //ms; 12 fps

const int max_num_streams = TILE_X * TILE_Y;

const int lines_body[LINE_NUM][2] = {{4, 2}, {2, 0}, {0, 1}, {1, 3},
                                    {10, 8}, {8, 6}, {6, 5}, {5, 7}, 
                                    {7, 9}, {6, 12}, {12, 11}, {11, 5},
                                    {12, 14}, {14, 16}, {11, 13}, {13, 15}};

cv::Mat full_frame(IMAGE_HEIGHT*TILE_Y, IMAGE_WIDTH*TILE_X, CV_8UC3);
int batt_perc[max_num_streams] = {};
geometry_msgs::msg::PoseArray::SharedPtr body[max_num_streams] = {};

void imageCallback(uint num, std::string ns, const sensor_msgs::msg::Image::ConstSharedPtr & msg)
{
  try {
    auto frame = cv_bridge::toCvShare(msg, "bgr8")->image;
    const int x = (num % TILE_X) * IMAGE_WIDTH;
    const int y = (num / TILE_X) * IMAGE_HEIGHT;
    frame.copyTo(full_frame(cv::Rect(x, y, IMAGE_WIDTH, IMAGE_HEIGHT)));
    cv::putText(full_frame, ns, cv::Point(x + 50, y + 50),
                cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(0, 0, 255), 2);
    if (batt_perc[num])
    {
      cv::putText(full_frame, std::to_string(batt_perc[num]) + "%",
                  cv::Point(x + IMAGE_WIDTH - 120, y + IMAGE_HEIGHT - 20),
                  cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(0, 0, 255), 2);
    }

    if (body[num])
    {
      auto poses = body[num]->poses;
      for (int i = 0; i < LINE_NUM; i++)
      {
        auto p1 = poses[lines_body[i][0]].position;
        auto p2 = poses[lines_body[i][1]].position;
        if (p1.z > SCORE_THRESH && p2.z > SCORE_THRESH)
        {
          float x1 = x + p1.x;
          float y1 = y + p1.y;
          float x2 = x + p2.x;
          float y2 = y + p2.y;
          if ((x1 > x && x2 > x && x1 < x + IMAGE_WIDTH && x2 < x + IMAGE_WIDTH) &&
              (y1 > y && y2 > y && y1 < y + IMAGE_HEIGHT && y2 < y + IMAGE_HEIGHT))
          {
            cv::line(full_frame, cv::Point(x1, y1), cv::Point(x2, y2),
                      cv::Scalar(255, 180, 90), 2, cv::LINE_AA);
          }
        }
      }
    }
  } 
  catch(const std::exception& e)
  {
    auto logger = rclcpp::get_logger(NODE_NAME);
    RCLCPP_ERROR(logger, "Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

void poseCallback(uint num, const geometry_msgs::msg::PoseArray::SharedPtr & msg)
{
  body[num] = msg;
  auto logger = rclcpp::get_logger(NODE_NAME);
  // RCLCPP_ERROR(logger, "Scores for %d: LSh %.2f RSh %.2f LEl %.2f REl %.2f", num, 
  //              msg->poses[5].position.z, msg->poses[6].position.z, msg->poses[7].position.z, msg->poses[8].position.z);
}

void batteryCallback(int num, const sensor_msgs::msg::BatteryState::SharedPtr & msg)
{
  batt_perc[num] = (int)(msg->percentage * 100);
}

void displayCallback()
{
  try
  {
    if (cv::getWindowProperty(WINDOW_NAME, 0) < 0)
    {
      rclcpp::shutdown();
      return;
    }
    cv::imshow(WINDOW_NAME, full_frame);
    cv::waitKey(10);
  }
  catch(const std::exception& e)
  {
    rclcpp::shutdown();
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared(NODE_NAME);
  node->declare_parameter("namespaces", std::vector<std::string>());
  std::vector<std::string> namespaces{node->get_parameter("namespaces").as_string_array()};
  auto logger = rclcpp::get_logger(NODE_NAME);

  if (namespaces.size() == 0)
  {
    RCLCPP_ERROR(logger, "No namespaces provided to display.");
    return 1;
  }
  if (namespaces.size() > max_num_streams)
  {
    RCLCPP_ERROR(logger, "Too many namespaces provided. Max number of namespaces is %d.", max_num_streams);
    return 1;
  }

  cv::namedWindow(WINDOW_NAME, cv::WINDOW_NORMAL);
  cv::startWindowThread();
  cv::resizeWindow(WINDOW_NAME, 1000, 400);

  image_transport::Subscriber frame_sub[max_num_streams];
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr body_sub[max_num_streams];
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub[max_num_streams];

  for (uint i=0; i < namespaces.size(); i++)
  {
    frame_sub[i] = image_transport::create_subscription(
      node.get(), "/"+ namespaces[i]+"/oakd/rgb/preview/encoded",
      [i, namespaces] (const sensor_msgs::msg::Image::ConstSharedPtr & msg) {imageCallback(i, namespaces[i], msg);},
      "ffmpeg", rmw_qos_profile_sensor_data);

    body_sub[i] = node->create_subscription<geometry_msgs::msg::PoseArray>(
      "/"+ namespaces[i]+"/body_pose",
      rclcpp::SensorDataQoS(),
      [i] (const geometry_msgs::msg::PoseArray::SharedPtr msg) {poseCallback(i, msg);});

    battery_sub[i] = node->create_subscription<sensor_msgs::msg::BatteryState>(
      "/"+ namespaces[i]+"/battery_state",
      rclcpp::SensorDataQoS(),
      [i] (const sensor_msgs::msg::BatteryState::SharedPtr msg) {batteryCallback(i, msg);});
  }

  auto timer = node->create_wall_timer(std::chrono::milliseconds(DISPLAY_PERIOD), displayCallback);

  rclcpp::spin(node);

  return 0;
}