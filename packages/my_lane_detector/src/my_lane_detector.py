#!/usr/bin/env python3

# Python libraries
import numpy as np
import cv2

# ROS libraries
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage


class Lane_Detector:
    def __init__(self):
        self.cv_bridge = CvBridge()

        # This topic comes from your rosbag file.
        # Your rostopic list showed:
        # /akandb/camera_node/image/compressed
        self.image_topic = "/akandb/camera_node/image/compressed"

        rospy.init_node("my_lane_detector", anonymous=True)

        self.image_sub = rospy.Subscriber(
            self.image_topic,
            CompressedImage,
            self.image_callback,
            queue_size=1
        )

        rospy.loginfo("Lane detector node started.")
        rospy.loginfo("Subscribed to image topic: %s", self.image_topic)

    def draw_lines(self, image, lines, color, thickness=3):
        """
        Draws Hough line segments on an image.
        """
        output = image.copy()

        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]

                # Draw the line
                cv2.line(output, (x1, y1), (x2, y2), color, thickness, cv2.LINE_AA)

                # Draw start and end points for better visualisation
                cv2.circle(output, (x1, y1), 4, (0, 255, 0), -1)
                cv2.circle(output, (x2, y2), 4, (0, 0, 255), -1)

        return output

    def image_callback(self, msg):
        """
        This function runs every time a new camera image arrives from the bag file.
        """

        # Convert ROS compressed image message to OpenCV BGR image
        img = self.cv_bridge.compressed_imgmsg_to_cv2(msg, "bgr8")

        # to convert upside down, comment this line out and test again.
        #img = cv2.flip(img, 0)

        height, width, channels = img.shape

        # ---------------------------------------------------------
        # 1. Crop image so that mostly the road is visible
        # ---------------------------------------------------------
        # The road/lane area is usually in the lower part of the image.
        crop_start = int(height * 0.45)
        cropped = img[crop_start:height, 0:width]

        # ---------------------------------------------------------
        # 2. Convert cropped image to HSV colour space
        # ---------------------------------------------------------
        hsv = cv2.cvtColor(cropped, cv2.COLOR_BGR2HSV)

        # ---------------------------------------------------------
        # 3. White colour filtering
        # ---------------------------------------------------------
        # White has low saturation and high brightness/value.
        lower_white = np.array([0, 0, 160])
        upper_white = np.array([180, 80, 255])

        white_mask = cv2.inRange(hsv, lower_white, upper_white)
        white_filtered = cv2.bitwise_and(cropped, cropped, mask=white_mask)

        # ---------------------------------------------------------
        # 4. Yellow colour filtering
        # ---------------------------------------------------------
        # Yellow hue is normally around 20-35 in HSV.
        lower_yellow = np.array([20, 70, 70])
        upper_yellow = np.array([40, 255, 255])

        yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        yellow_filtered = cv2.bitwise_and(cropped, cropped, mask=yellow_mask)

        # ---------------------------------------------------------
        # 5. Clean masks using morphology
        # ---------------------------------------------------------
        kernel = np.ones((5, 5), np.uint8)

        white_mask_clean = cv2.morphologyEx(white_mask, cv2.MORPH_OPEN, kernel)
        white_mask_clean = cv2.morphologyEx(white_mask_clean, cv2.MORPH_CLOSE, kernel)

        yellow_mask_clean = cv2.morphologyEx(yellow_mask, cv2.MORPH_OPEN, kernel)
        yellow_mask_clean = cv2.morphologyEx(yellow_mask_clean, cv2.MORPH_CLOSE, kernel)

        white_filtered_clean = cv2.bitwise_and(cropped, cropped, mask=white_mask_clean)
        yellow_filtered_clean = cv2.bitwise_and(cropped, cropped, mask=yellow_mask_clean)

        # ---------------------------------------------------------
        # 6. Apply Canny Edge Detector to the cropped image
        # ---------------------------------------------------------
        gray_cropped = cv2.cvtColor(cropped, cv2.COLOR_BGR2GRAY)
        canny_cropped = cv2.Canny(gray_cropped, 80, 160)

        # ---------------------------------------------------------
        # 7. Apply Canny + Hough Transform to the white-filtered image
        # ---------------------------------------------------------
        white_edges = cv2.Canny(white_mask_clean, 50, 150)

        white_lines = cv2.HoughLinesP(
            white_edges,
            rho=1,
            theta=np.pi / 180,
            threshold=20,
            minLineLength=20,
            maxLineGap=10
        )

        # ---------------------------------------------------------
        # 8. Apply Canny + Hough Transform to the yellow-filtered image
        # ---------------------------------------------------------
        yellow_edges = cv2.Canny(yellow_mask_clean, 50, 150)

        yellow_lines = cv2.HoughLinesP(
            yellow_edges,
            rho=1,
            theta=np.pi / 180,
            threshold=20,
            minLineLength=20,
            maxLineGap=10
        )

        # ---------------------------------------------------------
        # 9. Draw lines from both Hough Transforms on cropped image
        # ---------------------------------------------------------
        hough_output = cropped.copy()

        # Blue lines for white lane detections
        hough_output = self.draw_lines(hough_output, white_lines, (255, 0, 0), 3)

        # Yellow lines for yellow lane detections
        hough_output = self.draw_lines(hough_output, yellow_lines, (0, 255, 255), 3)

        # ---------------------------------------------------------
        # 10. Display required OpenCV outputs
        # ---------------------------------------------------------
        cv2.imshow("01 Original Image", img)
        cv2.imshow("02 Cropped Road Image", cropped)
        cv2.imshow("03 White Filtered Image", white_filtered_clean)
        cv2.imshow("04 Yellow Filtered Image", yellow_filtered_clean)
        cv2.imshow("05 Canny Edge Detector", canny_cropped)
        cv2.imshow("06 White Hough Edges", white_edges)
        cv2.imshow("07 Yellow Hough Edges", yellow_edges)
        cv2.imshow("08 Hough Lines on Cropped Image", hough_output)

        cv2.waitKey(1)

    def run(self):
        rospy.spin()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    try:
        lane_detector_instance = Lane_Detector()
        lane_detector_instance.run()
    except rospy.ROSInterruptException:
        pass
