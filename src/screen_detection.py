# #!/usr/bin/env python3

# import rospy
# import cv2
# import cv_bridge
# from sensor_msgs.msg import Image

# # Global bridge and image buffer
# bridge = cv_bridge.CvBridge()
# ids_image = None

# def ids_image_callback(data):
#     global ids_image
#     try:
#         # Convert ROS image message to OpenCV format
#         ids_image = bridge.imgmsg_to_cv2(data, "bgr8")
#     except cv_bridge.CvBridgeError as e:
#         rospy.logerr(f"Error converting image: {e}")

# def select_roi():
#     """
#     Selects an ROI from the IDS camera feed.

#     Returns:
#     tuple: Coordinates of the selected ROI in the format (x, y, width, height).
#     """
#     global ids_image

#     # Check if there's an image to process
#     if ids_image is None:
#         print("No image available yet.")
#         return None

#     # Resize the image to a consistent size for display
#     resized_image = cv2.resize(ids_image, (640, 480))

#     # Show the image and allow ROI selection
#     roi = cv2.selectROI("Select ROI", resized_image, fromCenter=False, showCrosshair=True)

#     # Close the ROI selection window
#     cv2.destroyWindow("Select ROI")

#     # If the ROI selection is valid, return it
#     if roi != (0, 0, 0, 0):
#         (x, y, w, h) = roi
#         return (x, y, w, h)
#     else:
#         print("ROI was not selected.")
#         return None

# def main():
#     rospy.init_node('ids_cam_display_node', anonymous=True)
    
#     # Subscribe to the IDS camera topic
#     rospy.Subscriber("/ids/rgb", Image, ids_image_callback)

#     # Create a window to display the image
#     window_name = "IDS Camera Feed"
#     cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
#     cv2.resizeWindow(window_name, 640, 480)

#     rate = rospy.Rate(10)  # Set the rate to 10Hz (10 frames per second)

#     while not rospy.is_shutdown():
#         if ids_image is not None:
#             # Resize the image to a consistent size for display
#             resized_image = cv2.resize(ids_image, (640, 480))

#             # Display the image in the OpenCV window
#             cv2.imshow(window_name, resized_image)

#             # Exit if 's' is pressed, and select ROI
#             key = cv2.waitKey(1) & 0xFF
#             if key == ord('s'):
#                 coordinates = select_roi()
#                 if coordinates:
#                     print(f"Selected ROI Coordinates: (x, y, width, height) = {coordinates}")
#                 break
#             elif key == ord('q'):
#                 break

#         rate.sleep()

#     # Cleanup and close window
#     cv2.destroyAllWindows()

# if __name__ == '__main__':
#     try:
#         main()
#     except rospy.ROSInterruptException:
#         pass


#!/usr/bin/env python

import rospy
import cv2
import os
import cv_bridge
from sensor_msgs.msg import Image

class VideoRecorder:
    def __init__(self):
        rospy.init_node('ids_video_recorder', anonymous=True)

        # Initialize variables
        self.recording = False
        self.bridge = cv_bridge.CvBridge()
        self.video_writer = None
        self.ids_image = None

        # Set up the subscriber for the IDS camera feed
        rospy.Subscriber("/camera/color/image_raw", Image, self.ids_image_callback)

        # Set up the window for the video feed, resized to 640x480
        cv2.namedWindow("IDS Camera Feed", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("IDS Camera Feed", 640, 480)

        # Set up the path to save the video on the desktop
        home_dir = os.path.expanduser("~")
        self.video_path = os.path.join(home_dir, "Desktop", "output_video1.avi")

    def ids_image_callback(self, data):
        # Convert the incoming ROS image message to an OpenCV format
        try:
            self.ids_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
        except cv_bridge.CvBridgeError as e:
            rospy.logerr("Could not convert image: %s", e)

    def start_recording(self):
        # Define the video codec and create a VideoWriter object
        fourcc = cv2.VideoWriter_fourcc(*'XVID')
        self.video_writer = cv2.VideoWriter(self.video_path, fourcc, 10.0, (self.ids_image.shape[1], self.ids_image.shape[0]))

        rospy.loginfo("Recording started... Press 'r' to stop.")
        rospy.loginfo(f"Saving video to: {self.video_path}")

    def stop_recording(self):
        self.recording = False
        if self.video_writer:
            self.video_writer.release()
            self.video_writer = None
        rospy.loginfo(f"Recording stopped. Video saved as '{self.video_path}'.")

    def run(self):
        rate = rospy.Rate(10)  # 10 Hz

        while not rospy.is_shutdown():
            if self.ids_image is not None:
                # Resize the image for display
                resized_image = cv2.resize(self.ids_image, (640, 480))

                # Display the resized frame
                cv2.imshow("IDS Camera Feed", resized_image)

                # Check for keyboard input
                key = cv2.waitKey(1) & 0xFF

                if key == ord('r'):
                    if not self.recording:
                        self.recording = True
                        self.start_recording()
                    else:
                        self.stop_recording()

                # If recording, save frames to the video
                if self.recording and self.video_writer:
                    self.video_writer.write(self.ids_image)

            rate.sleep()

        # Clean up
        if self.recording:
            self.stop_recording()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    video_recorder = VideoRecorder()
    video_recorder.run()
