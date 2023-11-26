import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class ConeDetectionNode(Node):
    def __init__(self):
        super().__init__('cone_detection')
        self.subscription = self.create_subscription(Image, '/depth_camera/image_raw', self.image_callback, 10)
        self.publisher = self.create_publisher(Image, '/cone_detection/image', 10)
        self.bridge = CvBridge()

    @staticmethod
    def aspect_ratio(contour):
        x, y, w, h = cv2.boundingRect(contour)
        return float(w) / h if h != 0 else 0.0

    def perform_color_based_detection(self, cv_image):
        # Convert the image to the HSV color space
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Define the lower and upper bounds for orange color (cones)
        lower_orange = np.array([10, 100, 150])  # Adjust saturation and value to exclude brownish tones
        upper_orange = np.array([25, 255, 255]) 

        # Threshold the image to obtain only orange regions
        orange_mask = cv2.inRange(hsv_image, lower_orange, upper_orange)

        # Find contours in the orange mask
        contours, _ = cv2.findContours(orange_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Draw contours on the original image
        cv2.drawContours(cv_image, contours, -1, (0, 255, 0), 2)

    def perform_contour_based_detection(self, cv_image):
        # Convert the image to grayscale
        gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # Apply Gaussian blur to reduce noise and improve contour detection
        blurred_image = cv2.GaussianBlur(gray_image, (5, 5), 0)

        # Apply edge detection using the Canny edge detector
        edges = cv2.Canny(blurred_image, 50, 150)

        # Find contours in the edge-detected image
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Filter contours based on size and aspect ratio
        min_contour_area = 50  # Adjust as needed
        max_contour_area = 5000  # Adjust as needed
        min_aspect_ratio = 0.0  # Adjust as needed
        max_aspect_ratio = 0.8  # Adjust as needed

        filtered_contours = [
            cnt for cnt in contours
            if min_contour_area < cv2.contourArea(cnt) < max_contour_area
            and min_aspect_ratio < self.aspect_ratio(cnt) < max_aspect_ratio
        ]

        # Draw rectangles around the cones
        result_image = cv_image.copy()
        for contour in filtered_contours:
            x, y, w, h = cv2.boundingRect(contour)
            cv2.rectangle(result_image, (x, y), (x + w, y + h), (0, 255, 0), 2)

        # Publish the result
        self.publisher.publish(self.bridge.cv2_to_imgmsg(result_image, "bgr8"))

    def image_callback(self, msg):
        # Convert the ROS Image message to an OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        # Perform color-based detection
        self.perform_color_based_detection(cv_image)

        # Perform contour-based detection
        self.perform_contour_based_detection(cv_image)

def main(args=None):
    rclpy.init(args=args)
    node = ConeDetectionNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
