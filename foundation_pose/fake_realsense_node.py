#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from sensor_msgs.msg import  CameraInfo

from cv_bridge import CvBridge
import cv2
import os
import glob

import numpy as np

import matplotlib.pyplot as plt

import imageio

from foundation_pose_interfaces.srv import Registration


class FakeRealsenseNode(Node):
  def __init__(self,):
    super().__init__('FakeRealsense')
    self.get_logger().info('FakeRealsense is Up! ')
    self.frame_rate = 60 # FPS

    # timer callbacks
    self._timer = self.create_timer(1.0 / self.frame_rate, self.timer_callback)


    # services
    self.regeneration_service =  self.create_service(Registration,
                                                    '/foundation_pose_regeneration',
                                                    self.handle_registration)
    # publishers
    self.rgb_pub =  self.create_publisher(Image, '/camera/camera/color/image_raw', 10)
    self.depth_pub =  self.create_publisher(Image, '/camera/camera/aligned_depth_to_color/image_raw', 10)

    # images
    self.bridge = CvBridge()
    code_dir = os.path.dirname(os.path.realpath(__file__))
    self.test_scene_dir = f'{code_dir}/demo_data/mustard0'

    self.rgb_counter = 0

    self.id_strs = []
    self.rgbs = sorted(glob.glob(f"{self.test_scene_dir}/rgb/*.png"))
    self.depths = sorted(glob.glob(f"{self.test_scene_dir}/depth/*.png"))

    for color_file in self.rgbs:
      id_str = os.path.basename(color_file).replace('.png','')
      self.id_strs.append(id_str)


    print(f"self.rgbs[0]::{self.rgbs[0]}")
    print(f"self.id_strs[0]::{self.id_strs[0]}")

    self.rgb_count = len(self.rgbs)-1

  # service callback
  def handle_registration(self,_,response):
    response.mask = self.get_mask()
    response.camera_info = self.get_camera_info()

    # color
    cv_image = cv2.imread(self.rgbs[0], cv2.IMREAD_COLOR)
    msg =  self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
    msg.header.frame_id = self.id_strs[0]
    response.color = msg

    # depth
    cv_image = cv2.imread(self.depths[0], -1)/1e3
    cv_image [(cv_image <0.001) | (cv_image >=np.inf)] = 0
    msg =  self.bridge.cv2_to_imgmsg(cv_image, encoding="64FC1")
    msg.header.frame_id = self.id_strs[0]
    response.depth = msg

    return response

  # timer callbacks
  def timer_callback(self):
    self.depth_pub.publish(self.get_depth())
    self.rgb_pub.publish(self.get_color())

    if self.rgb_counter < self.rgb_count:
      self.rgb_counter += 1

  # util
  def get_mask(self):
    mask_path = self.test_scene_dir+'/masks/1581120424100262102.png'
    cv_image = cv2.imread(mask_path, -1)
    msg =  self.bridge.cv2_to_imgmsg(cv_image, encoding="mono8")
    msg.header.frame_id = self.id_strs[0]

    return msg

  def get_color(self):
    cv_image = cv2.imread(self.rgbs[self.rgb_counter], cv2.IMREAD_COLOR)
    msg =  self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
    msg.header.frame_id = self.id_strs[self.rgb_counter]

    return msg

  def get_depth(self):
    cv_image = cv2.imread(self.depths[self.rgb_counter], cv2.IMREAD_GRAYSCALE)
    msg =  self.bridge.cv2_to_imgmsg(cv_image, encoding="mono8")
    msg.header.frame_id = self.id_strs[self.rgb_counter]
    return msg

  def get_camera_info(self):
    msg = CameraInfo()
    msg.height = 720
    msg.width =  1280
    msg.distortion_model = "plumb_bob"
    msg.d = [0.0]*5
    msg.k = [ 3.195820007324218750e+02, 0.000000000000000000e+00, 3.202149847676955687e+02,
              0.000000000000000000e+00, 4.171186828613281250e+02, 2.443486680871046701e+02,
              0.000000000000000000e+00, 0.000000000000000000e+00, 1.000000000000000000e+00
    ]
    msg.r = [ 1.0, 0.0, 0.0,
              0.0, 1.0, 0.0,
              0.0, 0.0, 1.0
    ]
    msg.p = [ 652.2882690429688 ,0.0 ,644.7269897460938,
              0.0, 0.0, 652.2882690429688,
              350.6556396484375, 0.0, 0.0,
              0.0, 1.0, 0.0
    ]
    msg.binning_x = 0
    msg.binning_y = 0
    msg.roi.x_offset = 0
    msg.roi.y_offset = 0
    msg.roi.height = 0
    msg.roi.width = 0
    msg.roi.do_rectify = False

    return msg

def main(args=None):
    rclpy.init(args=args)

    node = FakeRealsenseNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node stopped by user.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
  main()