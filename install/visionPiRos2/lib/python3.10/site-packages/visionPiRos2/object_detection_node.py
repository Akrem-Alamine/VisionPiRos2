import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import torch

class ObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('object_detection_node')
        self.bridge = CvBridge()
        self.publisher = self.create_publisher(Image, 'processed_image', 10)

        # Load YOLOv5 model
        self.model = torch.hub.load('ultralytics/yolov5', 'yolov5m', pretrained=True)

        # Open camera
        self.cap = cv2.VideoCapture(0)
        self.timer = self.create_timer(0.1, self.process_frame)

    def process_frame(self):
        ret, frame = self.cap.read()
        if ret:
            results = self.model(frame)
            frame = results.render()[0]

            # Convert frame to ROS2 image
            ros_image = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.publisher.publish(ros_image)

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
