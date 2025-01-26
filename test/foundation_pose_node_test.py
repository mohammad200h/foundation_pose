import os
import sys
import json
import numpy as np

import pytest
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor


from foundation_pose.foundation_pose_node import FoundationPoseNode

folder_path = os.path.dirname(os.path.abspath(__file__))

TEST_DATA_PATH=folder_path + '/test_data/mustred0'


def get_pose_for_image(image_str):
  path = TEST_DATA_PATH+'/photo_pose.json'
  with open(path, "r") as json_file:
    loaded_data = json.load(json_file)
    return loaded_data[image_str]

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


class FoundationPoseNodeTest(FoundationPoseNode):
  def __init__(self,prompt):
    super().__init__(prompt)

    self.flags = {
      "image_name":None,

      # registering
      "est_pose_last": None,
      "registered": False,
      "registering": False,
      "registration_callback_called":False,
      "registration_image_id":None,
      "registration_pose":None
    }


  def registration_callback(self,future):
    print("FoundationPoseNodeTest::registration_callback::called")
    super().registration_callback(future)
    self.flags["registration_image_id"] = future.result().color.header.frame_id
    self.flags["registration_callback_called"] = True

  def register(self, cam_K, color, depth, mask):
    print("FoundationPoseNodeTest::register::called")
    self.flags["registering"] = True
    super().register(cam_K, color, depth, mask)

    self.flags["est_pose_last"] = self.est.pose_last
    self.flags["registering"] = False
    self.flags["registered"] = True
    self.flags["registration_pose"] = self.pose

  def grab_color(self,msg):
    print("FoundationPoseNodeTest::grab_color::called")
    self.flags["image_name"] = msg.header.frame_id
    super().grab_color(msg)

@pytest.fixture
def ros_client_setup():
  rclpy.init()

  code_dir = os.path.dirname(os.path.realpath(__file__))
  prompt = "mustard"

  test_node = FoundationPoseNodeTest(prompt)
  executor = SingleThreadedExecutor()
  executor.add_node(test_node)
  yield test_node, executor
  test_node.destroy_node()
  rclpy.shutdown()


def test_registration(ros_client_setup):
  test_node, executor = ros_client_setup

  # Wait for the publisher to initialize
  time.sleep(1)  # Adjust this value based on the expected setup time
  # Run the executor and check for messages
  try:
    for _ in range(4):
      executor.spin_once(timeout_sec=0.1)
      if test_node.flags["registration_callback_called"]:

        assert isinstance(test_node.cam_K, np.ndarray)
        assert isinstance(test_node.color, np.ndarray)
        assert isinstance(test_node.depth, np.ndarray)


        # TODO: Check color depth and mask match stored numpy array
        assert np.array_equal(test_node.color , load_color())
        assert np.array_equal(test_node.depth , load_depth())
        assert np.array_equal(test_node.mask , load_mask())
        assert np.array_equal(test_node.cam_K , load_cam_k())

        while test_node.flags["registering"]:
          print("registering...")

        return
    assert False
  finally:
    pass

def test_registration_pose(ros_client_setup):
  test_node, executor = ros_client_setup

  # Wait for the publisher to initialize
  time.sleep(1)  # Adjust this value based on the expected setup time
  # Run the executor and check for messages
  try:
    for _ in range(4):
      executor.spin_once(timeout_sec=0.1)
      if test_node.flags["registration_callback_called"]:
        while test_node.flags["registering"]:
          print("registering...")

        if test_node.flags["registered"] and isinstance(test_node.depth,np.ndarray)  \
           and isinstance(test_node.color,np.ndarray) \
           and isinstance(test_node.depth, np.ndarray):


          assert test_node.flags["registration_image_id"] == "1581120424100262102"
          assert isinstance(test_node.pose, np.ndarray)
          expected_pose = get_pose_for_image(test_node.flags["registration_image_id"])
          assert expected_pose == test_node.flags["registration_pose"].tolist()

        return
    assert False
  finally:
    pass

# Note: Need to launch publisher in terminal. the test does not launch a publisher
# ros2 run foundation_pose fake_realsense

# to run test
# colcon test