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
  with open("data.json", "r") as json_file:
    loaded_data = json.load(json_file)
    return loaded_data[image_str]


class FoundationPoseNodeTest(FoundationPoseNode):
  def __init__(self,mesh_file):
    super().__init__(mesh_file)

    self.flags = {
      "est_pose_last": None,
      "registered": False,
      "image_name":None,
      "registration_callback_called":False
    }


  def registration_callback(self,future):
    print("FoundationPoseNodeTest::registration_callback::called")
    super().registration_callback(future)
    self.flags["registration_callback_called"] = True

  def register(self, cam_K, color, depth, mask):
    print("FoundationPoseNodeTest::register::called")
    super().register(cam_K, color, depth, mask)
    self.flags["est_pose_last"] = self.est.pose_last
    self.flags["registered"] = True

  def grab_color(self,msg):
    print("FoundationPoseNodeTest::grab_color::called")
    self.flags["image_name"] = msg.header.frame_id
    super().grab_color(msg)

@pytest.fixture
def ros_client_setup():
  rclpy.init()


  code_dir = os.path.dirname(os.path.realpath(__file__))
  mesh_file = TEST_DATA_PATH+'/mustard0.obj'

  test_node = FoundationPoseNodeTest(mesh_file)
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

        # assert test_node.flags["est_pose_last"] != None
        assert test_node.cam_K
        return
    assert False
  finally:
    pass

def test_on_track(ros_client_setup):
  test_node, executor = ros_client_setup

  # Wait for the publisher to initialize
  time.sleep(1)  # Adjust this value based on the expected setup time
  # Run the executor and check for messages
  try:
    for _ in range(4):
      executor.spin_once(timeout_sec=0.1)
      if test_node.flags["registered"] and test_node.depth !=None \
         and test_node.color !=None:

        assert test_node.pose != None
        expected_pose = get_pose_for_image(test_node.flags["image_name"])
        assert expected_pose == test_node.pose.tolist()

        return
      assert False
  finally:
    pass

# Note: Need to launch publisher in terminal. the test does not launch a publisher
# ros2 run foundation_pose fake_realsense

# to run test
# colcon test