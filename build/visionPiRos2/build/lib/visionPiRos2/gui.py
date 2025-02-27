import sys
import subprocess
from PyQt6.QtWidgets import QApplication, QMainWindow, QPushButton, QVBoxLayout, QHBoxLayout, QLabel, QWidget, QTabWidget, QSizePolicy
from PyQt6.QtGui import QImage, QPixmap
from PyQt6.QtCore import QThread, pyqtSignal, Qt, pyqtSlot
import cv2
import rclpy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


class ROS2CameraThread(QThread):
    """Thread to receive and process images from ROS2 topic."""
    frame_signal = pyqtSignal(QImage)

    def __init__(self):
        super().__init__()
        self.node = rclpy.create_node('camera_gui_subscriber')
        self.bridge = CvBridge()
        self.subscription = self.node.create_subscription(Image, 'processed_image', self.image_callback, 10)
        self.running = False  

    def image_callback(self, msg):
        """Convert ROS2 image message to QImage and emit signal."""
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        rgb_image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        qt_image = QImage(rgb_image.data, rgb_image.shape[1], rgb_image.shape[0],
                          rgb_image.strides[0], QImage.Format.Format_RGB888)
        self.frame_signal.emit(qt_image)

    def run(self):
        self.running = True
        while self.running:
            rclpy.spin_once(self.node, timeout_sec=0.1)

    def stop(self):
        """Stop the thread and destroy the ROS2 node."""
        self.running = False
        self.node.destroy_node()


class VisionPiGUI(QMainWindow):
    def __init__(self):
        super().__init__()

        # Set up main window
        self.setWindowTitle('Vision Pi ROS 2')
        self.setGeometry(100, 100, 900, 500)

        # Sidebar buttons
        self.btn_object_detection = QPushButton('Start Object Detection')
        self.btn_face_recognition = QPushButton('Face Recognition')
        self.btn_gesture_recognition = QPushButton('Gesture Recognition')
        self.btn_pose_estimation = QPushButton('Pose Estimation')

        self.btn_object_detection.clicked.connect(self.start_ros2_camera)
        self.btn_face_recognition.clicked.connect(lambda: self.switch_tab(1))
        self.btn_gesture_recognition.clicked.connect(lambda: self.switch_tab(2))
        self.btn_pose_estimation.clicked.connect(lambda: self.switch_tab(3))

        # Sidebar layout
        sidebar_layout = QVBoxLayout()
        sidebar_layout.addWidget(self.btn_object_detection)
        sidebar_layout.addWidget(self.btn_face_recognition)
        sidebar_layout.addWidget(self.btn_gesture_recognition)
        sidebar_layout.addWidget(self.btn_pose_estimation)
        sidebar_layout.addStretch(5)
        sidebar_widget = QWidget()
        sidebar_widget.setLayout(sidebar_layout)

        # Main content (tab system)
        self.tabs = QTabWidget()
        self.tabs.addTab(self.create_video_tab(), 'Object Detection')
        self.tabs.addTab(self.create_placeholder_tab('Face Recognition'), 'Face Recognition')
        self.tabs.addTab(self.create_placeholder_tab('Gesture Recognition'), 'Gesture Recognition')
        self.tabs.addTab(self.create_placeholder_tab('Pose Estimation'), 'Pose Estimation')

        # Main layout
        main_layout = QHBoxLayout()
        main_layout.addWidget(sidebar_widget)
        main_layout.addWidget(self.tabs, 1)
        main_widget = QWidget()
        main_widget.setLayout(main_layout)
        self.setCentralWidget(main_widget)

        # Initialize ROS2 camera thread
        self.camera_thread = ROS2CameraThread()
        self.camera_thread.frame_signal.connect(self.update_image)
        self.process = None  

    def create_video_tab(self):
        """Create the object detection tab with a dynamically resizing video label."""
        self.video_label = QLabel()
        self.video_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.video_label.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)  # Allow resizing
        self.video_label.setMinimumSize(800, 300)  # Set a minimum size

        layout = QVBoxLayout()
        layout.addWidget(QLabel('Object Detection is Active'))
        layout.addWidget(self.video_label)
        container = QWidget()
        container.setLayout(layout)
        return container

    def create_placeholder_tab(self, title):
        """Create a placeholder tab for future features."""
        layout = QVBoxLayout()
        layout.addWidget(QLabel(title))
        container = QWidget()
        container.setLayout(layout)
        return container

    def switch_tab(self, index):
        """Switch to a specific tab."""
        self.tabs.setCurrentIndex(index)

    def start_ros2_camera(self):
        """Starts object detection node and video feed when button is clicked."""
        self.switch_tab(0)  # Switch to object detection tab

        if not self.process or self.process.poll() is not None:
            self.process = subprocess.Popen(["ros2", "run", "visionPiRos2", "object_detection_node"])
            print("Started object detection node")

        if not self.camera_thread.isRunning():
            self.camera_thread.start()
            print("Started camera thread")

    @pyqtSlot(QImage)
    def update_image(self, image):
        """Dynamically resizes the image to fit inside the QLabel while maintaining aspect ratio."""
        if not image.isNull():
            scaled_image = image.scaled(self.video_label.size(), Qt.AspectRatioMode.KeepAspectRatio)
            self.video_label.setPixmap(QPixmap.fromImage(scaled_image))

    def resizeEvent(self, event):
        """Ensure the video resizes dynamically when the window is resized."""
        if self.video_label.pixmap():  # If an image exists, resize it
            self.update_image(self.video_label.pixmap().toImage())
        super().resizeEvent(event)

    def closeEvent(self, event):
        """Handle GUI close event and stop all running processes."""
        print("Closing GUI...")

        if self.camera_thread.isRunning():
            self.camera_thread.stop()
            self.camera_thread.wait()

        if self.process and self.process.poll() is None:
            print("Stopping object detection node...")
            self.process.terminate()
            self.process.wait()

        event.accept()


def main():
    """Main function to initialize ROS2 and launch the GUI."""
    rclpy.init()
    app = QApplication(sys.argv)
    window = VisionPiGUI()
    window.show()
    sys.exit(app.exec())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
