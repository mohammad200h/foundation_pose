import os
import sys
import numpy as np

import rclpy

from foundation_pose.fake_realsense_node import FakeRealsenseNode
from threading import Thread

from rclpy.node import Node
from std_msgs.msg import String
from rclpy.executors import SingleThreadedExecutor
from rclpy.executors import MultiThreadedExecutor
import pytest
import time

from sensor_msgs.msg import Image as rosImg
from foundation_pose_interfaces.srv import Registration
from sensor_msgs.msg import  CameraInfo
from geometry_msgs.msg import Pose, PoseStamped

from cv_bridge import CvBridge

import multiprocessing
import subprocess
import signal

import matplotlib.pyplot as plt

import cv2


folder_path = os.path.dirname(os.path.abspath(__file__))

TEST_DATA_PATH=folder_path + '/test_data/mustred0'


def load_cam_k():
  path = TEST_DATA_PATH+"/registeration_npy/register_K.npy"
  return np.load(path)

def load_color():
  path = TEST_DATA_PATH+"/registeration_npy/register_rgb.npy"
  return np.load(path)

def load_depth():
  path = TEST_DATA_PATH+"/registeration_npy/register_depth.npy"
  return np.load(path)

def load_mask():
  path = TEST_DATA_PATH+"/registeration_npy/register_ob_mask.npy"
  return np.load(path)

class TestFakeRealsenseSubscriber(Node):
  def __init__(self):
    super().__init__('test_subscriber')

    # Subscribers
    self.sub_color =  self.create_subscription(rosImg,
                                                '/camera/camera/color/image_raw',
                                                self.grab_color, 10)
    self.sub_depth = self.create_subscription(rosImg,
                                                '/camera/camera/aligned_depth_to_color/image_raw',
                                                self.grab_depth, 10)

    self.message_received = {
      "color":None,
      "depth":None

    }

  # callbacks
  def grab_color(self, msg):
    self.get_logger().info('grab_color called! ')
    self.message_received["color"] = msg.data

  def grab_depth(self, msg):
    self.get_logger().info('grab_depth called! ')
    self.message_received["depth"] = msg.data


class TestFakeRealsenseClient(Node):
  def __init__(self):
    super().__init__('test_client')

    self.flags = {
      "service_is_online":False,
      "received_response":False
    }

    self.data = {
      "cam_k":None,
      "color":None,
      "depth":None,
      "mask":None
    }

    # try to register the object
    self.regeneration_client = self.create_client(Registration,
                                                  '/foundation_pose_regeneration')

    while not self.regeneration_client.wait_for_service(timeout_sec=1.0):
      self.get_logger().info('Waiting for foundation_pose_regeneration service...')

    self.flags["service_is_online"] = True
    self.send_registration_request()



  def send_registration_request(self):
     # Since there is no request part, we send an empty request
     self.request = Registration.Request()
     self.future = self.regeneration_client.call_async(self.request)
     self.future.add_done_callback(self.registration_callback)

  def registration_callback(self, future):
      try:
        response = future.result()

        print("\n")
        print(f"registration_callback::response::{response.mask.header.frame_id}")
        print(f"registration_callback::response::{response.color.header.frame_id}")
        print(f"registration_callback::response::{response.depth.header.frame_id}")
        print("\n")

        cam_K = response.camera_info.k
        cam_K = np.array(cam_K).reshape((3,3))

        color = CvBridge().imgmsg_to_cv2(response.color, desired_encoding="bgr8")
        color = cv2.cvtColor(color, cv2.COLOR_BGR2RGB)

        depth = CvBridge().imgmsg_to_cv2(response.depth, desired_encoding="64FC1")


        mask = CvBridge().imgmsg_to_cv2(response.mask, desired_encoding="mono8").astype(bool).astype(np.uint8)

        self.flags["received_response"] = True

        self.data["cam_k"] = cam_K
        self.data["color"] = color
        self.data["depth"] = depth
        self.data["mask"] = mask

      except Exception as e:
        self.get_logger().error(f"Service call failed: {e}")


@pytest.fixture
def ros_subscriber_setup():
    rclpy.init()
    test_node = TestFakeRealsenseSubscriber()
    executor = SingleThreadedExecutor()
    executor.add_node(test_node)
    yield test_node, executor
    test_node.destroy_node()
    rclpy.shutdown()

@pytest.fixture
def ros_client_setup():
  rclpy.init()
  test_node = TestFakeRealsenseClient()
  executor = SingleThreadedExecutor()
  executor.add_node(test_node)
  yield test_node, executor
  test_node.destroy_node()
  rclpy.shutdown()


# @pytest.mark.launch_test
# def generate_test_description():

#     import launch
#     import launch_testing
#     from launch import LaunchDescription
#     from launch_ros.actions import Node


#     test_node = Node(
#         package='foundation_pose',
#         executable='fake_realsense',
#         name='test_node',
#         output='screen',
#     )

#     return LaunchDescription([
#         test_node,
#         launch_testing.actions.ReadyToTest(),
#     ]), {
#         'test_node': test_node,
#     }



def test_color_message_published(ros_subscriber_setup):
    test_node, executor = ros_subscriber_setup

    # Wait for the publisher to initialize
    time.sleep(1)  # Adjust this value based on the expected setup time
    # Run the executor and check for messages
    try:
        for _ in range(4):
            executor.spin_once(timeout_sec=0.1)
            if test_node.message_received["color"]:
                assert test_node.message_received["color"] != None , \
                f"color image was not received "
                return
            else:
              test_node.get_logger().info('Waiting for messages...')

        assert False
    finally:
      pass

def test_depth_message_published(ros_subscriber_setup):
    test_node, executor = ros_subscriber_setup



     # Wait for the publisher to initialize
    time.sleep(1)  # Adjust this value based on the expected setup time
    # Run the executor and check for messages

    # Run the executor and check for messages
    try:
        for _ in range(4):
            executor.spin_once(timeout_sec=0.1)
            if test_node.message_received["depth"]:
                assert test_node.message_received["depth"] != None , \
                f"depth image was not received "
                return
            else:
              test_node.get_logger().info('Waiting for messages...')
        assert False
    finally:
      pass

def test_registration_service(ros_client_setup):
  test_node, executor = ros_client_setup

  # Wait for the publisher to initialize
  time.sleep(1)  # Adjust this value based on the expected setup time
  # Run the executor and check for messages
  try:
    for _ in range(4):
      executor.spin_once(timeout_sec=0.1)
      if test_node.flags["service_is_online"]  and  test_node.flags["received_response"]:

        expected_color = load_color()
        color = test_node.data["color"]
        assert np.array_equal(expected_color, color), \
          f"color arrays do not match"

        expected_depth = load_depth()
        depth = test_node.data["depth"]
        assert np.array_equal(expected_depth, depth), \
          f"depth arrays do not match"

        expected_cam_k = load_cam_k()
        cam_k = test_node.data["cam_k"]
        assert np.array_equal(expected_cam_k, cam_k), \
          f"cam_k arrays do not match"

        expected_mask = load_mask()
        mask = test_node.data["mask"]
        assert np.array_equal(expected_mask, mask), \
          f"mask arrays do not match"

        return
      else:
        test_node.get_logger().info('Waiting for client...')
    assert False
  finally:
    pass


# Note: Need to launch publisher in terminal. the test does not launch a publisher
# ros2 run foundation_pose fake_realsense