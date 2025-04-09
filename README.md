VisionPi: Object Detection System

VisionPi is a computer vision application developed as part of my end-of-year project. The application leverages YOLOv5 for real-time object detection, capable of identifying and classifying objects in a live video feed. This project is designed to demonstrate the potential of machine learning in computer vision, making it scalable for use in real-time applications.


---

Key Features

Real-time Object Detection: Detects objects in a video stream using the YOLOv5 model.

Bounding Box Display: Draws bounding boxes around detected objects, with labels indicating the class.

Efficient and Scalable: Optimized for real-time processing, capable of running on standard hardware.

Customizable: Future enhancements will include adding more advanced features like face recognition, gesture detection, and pose estimation.



---

Technologies Used

YOLOv5: A state-of-the-art model for object detection, implemented using PyTorch.

OpenCV: For real-time video capture and image processing.

PyQt5: Used for building the GUI interface of the application.

Python: The core programming language used for developing the application.



---

Setup and Installation

1. Clone the repository:

git clone https://github.com/yourusername/visionpi.git


2. Navigate to the project directory:

cd visionpi


3. Create a virtual environment and activate it:

python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate


4. Install required dependencies:

pip install -r requirements.txt


5. Run the application:

python app.py




---

Project Structure

visionpi/
├── app.py                       # Main application entry point
├── gui/                         # UI and interface components
│   └── main_window.py           # PyQt main window (buttons, tabs, video area)
├── core/                        # Logic / detection / utilities
│   ├── yolo_detector.py         # YOLOv5 object detection class
│   └── camera_thread.py         # OpenCV camera thread that runs detection
├── data/                        # Folder for images, face recognition data (if added)
│   └── faces/                   # Folder for known face images
├── requirements.txt             # All dependencies for pip install
└── README.md                    # Project documentation


---

How It Works

Face Detection (and Recognition): The initial version of the project includes object detection with YOLOv5. The face detection feature can be expanded later for face recognition, allowing the application to label known faces and classify others as "unknown".

Object Detection: YOLOv5 is used to identify various objects in real-time video feeds, making it a versatile system for a wide range of use cases.



---

Future Features

Face Recognition: Implement the ability to recognize and label known individuals in the video feed.

Gesture Recognition: Add the capability to detect and interpret user gestures.

Pose Estimation: Enable real-time body pose recognition for various applications.



---

Acknowledgments

YOLOv5: A powerful object detection model by Ultralytics.

OpenCV: For handling video capture and processing.

PyQt5: For building the GUI interface.
