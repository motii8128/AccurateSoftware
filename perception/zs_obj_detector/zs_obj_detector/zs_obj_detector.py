# Import ROS2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

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
        self.model = CLIPModel.from_pretrained("openai/clip-vit-base-patch32")
        self.processor = CLIPProcessor.from_pretrained("openai/clip-vit-base-patch32")
        self.sub_image = self.create_subscription(Image, "/image", self.topic_callback, 0)
        self.target_txt = "yellow ball"

    def img_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        inputs = self.processor(text=self.target_txt, images=image, return_tensors="pt", padding=True)
        
        with torch.no_grad():
            outputs = self.model(**inputs)
            logits_per_image = outputs.logits_per_image



def main(args=None):
    rclpy.init(args=args)
    zs_obj_detector = ZsObjDetector()
    rclpy.spin(zs_obj_detector)
    rclpy.shutdown()

if __name__ == '__name__':
    main