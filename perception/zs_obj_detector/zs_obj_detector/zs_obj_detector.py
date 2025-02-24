# Import ROS2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

# Import OpenCV
import cv2
from cv_bridge import CvBridge

# Import other dependencies
import torch
from transformers import CLIPProcessor, CLIPModel
from PIL import Image as PILImage
import numpy as np

class ZsObjDetector(Node):
    def __init__(self):
        super().__init__('zs_obj_detector')
        self.bridge = CvBridge()