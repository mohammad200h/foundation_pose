#!/usr/bin/env python3

import rclpy
from rclpy.node import Node


from scipy.spatial.transform import Rotation as R

from cv_bridge import CvBridge

import numpy as np

import cv2

from PIL import Image as Img


from threading import Event

from .estimater import *
# from datareader import *
import argparse

from sensor_msgs.msg import Image as rosImg
from sensor_msgs.msg import  CameraInfo
from geometry_msgs.msg import Pose, PoseStamped

from foundation_pose_interfaces.srv import Registration
from std_msgs.msg import String

objects = ["mustard","apple","drill","tuna"]
code_dir = os.path.dirname(os.path.realpath(__file__))

def get_mesh_path(prompt:str):
  if prompt == "mustard":
    return f'{code_dir}/demo_data/mustard0/mustard0.obj'
  elif prompt == "apple":
    return f'{code_dir}/demo_data/apple/apple.obj'
  elif prompt == "drill":
    return f'{code_dir}/demo_data/drill/drill.obj'
  elif prompt == "tuna":
    return f'{code_dir}/demo_data/tuna/tuna.obj'
  elif prompt == "banana":
    return f'{code_dir}/demo_data/banana/banana.obj'
  else:
    raise ValueError("object not supported")

class FoundationPoseNode(Node):
  def __init__(self,prompt:str):
    super().__init__('FoundationPose')
    self.get_logger().info('FoundationPose is Up! ')

    self.stream_visulization = True

    self.flags ={
      "color":False,
      "depth":False,
      "cam_K":False,
      "mask":False,
      "registered":False,
    }
    self.registering = False
    self.first_run = True

    self.color = None
    self.depth = None
    self.cam_K = None
    self.cur_time = None
    self.mask = None
    self.pose = None

    self.prompt = prompt
    self.bridge = CvBridge()

    # Foundation pose
    debug = 1
    debug_dir = "outputs"
    mesh = trimesh.load(get_mesh_path(prompt))
    scorer = ScorePredictor()
    refiner = PoseRefinePredictor()
    glctx = dr.RasterizeCudaContext()
    self.to_origin, extents = trimesh.bounds.oriented_bounds(mesh)
    self.bbox = np.stack([-extents/2, extents/2], axis=0).reshape(2,3)

    self.est = FoundationPose(model_pts = mesh.vertices, model_normals = mesh.vertex_normals,
                              mesh = mesh, scorer = scorer, refiner = refiner,
                              debug_dir = debug_dir, debug = debug, glctx = glctx)

    # try to register the object
    self.regeneration_client = self.create_client(Registration,
                                                  '/foundation_pose_regeneration')

    while not self.regeneration_client.wait_for_service(timeout_sec=1.0):
      self.get_logger().info('Waiting for foundation_pose_regeneration service...')
    self.send_registration_request()

    # Subscribers
    self.sub_color = self.create_subscription(rosImg,
                                                '/camera/camera/color/image_raw',
                                                self.grab_color, 10)
    self.sub_depth = self.create_subscription(rosImg,
                                                '/camera/camera/aligned_depth_to_color/image_raw',
                                                self.grab_depth, 10)
    # Publisher
    self.pose_publisher = self.create_publisher(PoseStamped,
                                                "/foundation_pose", 10)

    self.pose_image_publisher = self.create_publisher(rosImg,
                                                '/camera/camera/color/pose_image',
                                                 10)

  # foundation pose setup
  def register(self, cam_K, color, depth, mask):
    print("register::called")
    self.registering = True
    mask_binary = mask.astype(bool)
    self.pose = self.est.register(K = cam_K,
                                  rgb = color,
                                  depth = depth,
                                  ob_mask = mask_binary,
                                  iteration=5)

    print(f"registered::pose::{self.pose}")
    print(f"self.est.pose_last::{self.est.pose_last}")
    self.flags["registered"] = True
    self.registering = False

  def reset(self,pose_init):
    self.color = None
    self.depth = None
    self.cur_time = None
    self.pose = self.est.register(K=self.cam_K, rgb=self.color, depth=self.depth, ob_mask=self.mask, iteration=5)

  def send_registration_request(self):
    # Since there is no request part, we send an empty request
    print("send_registration_request::called")
    self.request = Registration.Request()
    string_msg = String()
    string_msg.data = self.prompt
    self.request.prompt = string_msg
    self.future = self.regeneration_client.call_async(self.request)
    self.future.add_done_callback(self.registration_callback)

  def registration_callback(self, future):
    print("registration_callback::called")
    try:
      response = future.result()

      print("\n")
      print(f"registration_callback::response::mask::{response.mask.header.frame_id}")
      print(f"registration_callback::response::color::{response.color.header.frame_id}")
      print(f"registration_callback::response::depth::{response.depth.header.frame_id}")
      print("\n")

      cam_K = response.camera_info.k

      print(f"registration_callback::cam_K::{cam_K}")
      cam_K = np.array(cam_K).reshape((3,3))
      print(f"registration_callback::cam_K::{cam_K}")
      self.cam_K = cam_K
      color = CvBridge().imgmsg_to_cv2(response.color, desired_encoding="bgr8")
      color = cv2.cvtColor(color, cv2.COLOR_BGR2RGB)
      depth = CvBridge().imgmsg_to_cv2(response.depth, desired_encoding="passthrough")
      depth[(depth<0.001) | (depth>=np.inf)] = 0

      np.save("depth.npy", depth)

      mask = CvBridge().imgmsg_to_cv2(response.mask, desired_encoding="passthrough").astype(np.uint8)
      np.save('mask.npy', mask)
      self.register(cam_K,color,depth,mask)

    except Exception as e:
      self.get_logger().error(f"Service call failed: {e}")

  # callbacks
  def run(self):
      self.on_track()

  def grab_depth(self, msg):
    print(f"grab_depth::image::id::{msg.header.frame_id}")
    depth = CvBridge().imgmsg_to_cv2(msg, desired_encoding="passthrough")
    depth[(depth<0.001) | (depth>=np.inf)] = 0

    self.depth = depth
    print(f"self.color ::{isinstance(self.color, np.ndarray) }")
    print(f"self.depth ::{isinstance(self.depth, np.ndarray) }")
    print(f"self.cam_K ::{isinstance(self.cam_K, np.ndarray) }")

    if isinstance(self.color, np.ndarray) and \
       isinstance(self.depth, np.ndarray) and \
       isinstance(self.cam_K, np.ndarray) and \
      self.flags["registered"] == True:
      self.run()
      self.depth = None
      self.color = None

  def grab_color(self, msg):
    print(f"grab_color::image::id::{msg.header.frame_id}")
    self.cur_time = msg.header.stamp
    color = CvBridge().imgmsg_to_cv2(msg, desired_encoding="bgr8")
    color = cv2.cvtColor(color, cv2.COLOR_BGR2RGB)
    self.color = color

  def on_track(self):
    print("TRACKING")

    color = self.color
    depth = self.depth
    self.pose = self.est.track_one(rgb = color, depth = depth,
                                   K = self.cam_K, iteration = 2)

    print(f"on_track::pose::{self.pose}")

    if self.stream_visulization == True:
      center_pose = self.pose@np.linalg.inv(self.to_origin)
      vis = draw_posed_3d_box(self.cam_K,
                              img=color, ob_in_cam=center_pose, bbox=self.bbox)
      vis = draw_xyz_axis(color,
                          ob_in_cam=center_pose,
                          scale=0.1, K=self.cam_K, thickness=3,
                          transparency=0, is_input_rgb=True)

      msg =  self.bridge.cv2_to_imgmsg(vis, encoding="bgr8")
      self.pose_image_publisher.publish(msg)


    # Convert center_pose to the pose format
    position = self.pose[:3, 3]
    rotation_matrix = self.pose[:3, :3]
    quaternion = R.from_matrix(rotation_matrix).as_quat()

    # print(f"on_track::position::{position}")
    # print(f"on_track::position::type::{type(position)}")


    pose_stamped_msg = PoseStamped()

    pose_stamped_msg.header.stamp = self.cur_time
    pose_stamped_msg.header.frame_id = "camera_color_optical_frame"
    # pose_stamped_msg.child_frame_id = "tracked_object_origin_in_cam"

    pose_stamped_msg.pose.position.x = float(position[0])
    pose_stamped_msg.pose.position.y = float(position[1])
    pose_stamped_msg.pose.position.z = float(position[2])

    pose_stamped_msg.pose.orientation.w = quaternion[3]
    pose_stamped_msg.pose.orientation.x = quaternion[0]
    pose_stamped_msg.pose.orientation.y = quaternion[1]
    pose_stamped_msg.pose.orientation.z = quaternion[2]

    self.pose_publisher.publish(pose_stamped_msg)


def main(args=None):
    # Initialize rclpy
    rclpy.init(args=args)



    tracker_node = FoundationPoseNode("mustard")
    try:
      while rclpy.ok():
        pass
        rclpy.spin_once(tracker_node)
    except KeyboardInterrupt:
      tracker_node.get_logger().info('Tracker node stopped.')
    finally:
      tracker_node.destroy_node()
      rclpy.shutdown()

if __name__ == '__main__':
  main()
