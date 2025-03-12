import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
import cv2
from cv_bridge import CvBridge
import numpy as np

class ObjDetector(Node):
    def __init__(self):
        super.__init__('obj_detector')
        self.target_txt = 'ball'

        self.sub_target_txt = self.create_subscription(
            String, '/target', self.target_txt_callback, 0
        )

        self.sub_image = self.create_subscription(
            Image, '/image', self.image_callback, 0
        )

        self.img_pub = self.create_publisher(Image, '/detect_img', 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/pose', 0)

        self.bridge = CvBridge()
        self.BALL_SIZE = 0.068
        self.fx = 643.7341918945312
        self.fy = 643.2481079101562
        self.cx = 320.0
        self.cy = 240.0

    def target_txt_callback(self, msg):
        self.target_txt = msg.data

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        img_yuv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2YUV)
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
        img_yuv[:,:,0] = clahe.apply(img_yuv[:,:,0])
        result_img = cv2.cvtColor(img_yuv, cv2.COLOR_YUV2BGR)

        hsv = cv2.cvtColor(result_img, cv2.COLOR_BGR2HSV)

        hsv_min = np.array([25, 90, 64])
        hsv_max = np.array([40, 255, 255])
        mask1 = cv2.inRange(hsv, hsv_min, hsv_max)

        hsv_min = np.array([40, 64, 0])
        hsv_max = np.array([70, 255, 255])
        mask2 = cv2.inRange(hsv, hsv_min, hsv_max)

        mask = cv2.bitwise_or(mask1, mask2)

        obj = cv2.bitwise_and(result_img,result_img, mask=mask)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        large_area = 0.0
        best_contour = None

        for contour in contours:
            if cv2.contourArea(contour) > large_area:  # 面積が100以上の物体のみ検出
                large_area = cv2.contourArea(contour)
                best_contour = contour
                
        x, y, w, h = cv2.boundingRect(best_contour)
        cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
        distance = (self.fx * self.BALL_SIZE) / w
        center_x = x + w / 2
        center_y = y + h / 2

        obj_pose = PoseStamped()
        obj_pose.header.frame_id = 'camera_link'
        obj_pose.pose.position.x = (center_x - self.cx) * distance / self.fx
        obj_pose.pose.position.y = (center_y - self.cy) * distance / self.fy
        obj_pose.pose.position.z = 0.0
        self.pose_pub.publish(obj_pose)
        
        ros_image = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
        self.img_pub.publish(ros_image)

def main(args=None):
    rclpy.init(args=args)
    obj_detector = ObjDetector()
    rclpy.spin(obj_detector)
    obj_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()