# import cv2
# import time

# class ScreenMonitor:
#     def __init__(self, video_source=0, brightness_threshold=10):
#         self.video_source = video_source
#         self.cap = cv2.VideoCapture(self.video_source)
#         if not self.cap.isOpened():
#             raise Exception(f"Could not open video source: {self.video_source}")

#         self.roi = None
#         self.initial_brightness = None
#         self.changed_frame = None
#         self.brightness_threshold = brightness_threshold
#         self.frame_width = 640
#         self.frame_height = 480

#     def resize_frame(self, frame):
#         return cv2.resize(frame, (self.frame_width, self.frame_height))

#     def select_roi(self):
#         print("Select the phone screen area.")
#         while True:
#             ret, frame = self.cap.read()
#             if not ret:
#                 raise Exception("Failed to capture frame for ROI selection")

#             frame = self.resize_frame(frame)
#             self.roi = cv2.selectROI("Select ROI", frame, fromCenter=False, showCrosshair=True)
#             cv2.destroyWindow("Select ROI")

#             if self.roi != (0, 0, 0, 0):
#                 break
#             print("No ROI selected, please select again.")

#         print(f"Selected ROI: {self.roi}")

#     def get_brightness(self, frame):
#         if self.roi is None:
#             raise Exception("ROI not set. Please select the ROI first.")

#         x, y, w, h = self.roi
#         roi_frame = frame[y:y+h, x:x+w]
#         gray = cv2.cvtColor(roi_frame, cv2.COLOR_BGR2GRAY)
#         return gray.mean()

#     def capture_initial_brightness(self):
#         self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)  # Restart video
#         ret, frame = self.cap.read()
#         if not ret:
#             raise Exception("Failed to capture frame")

#         frame = self.resize_frame(frame)
#         self.initial_brightness = self.get_brightness(frame)
#         print(f"Initial Brightness: {self.initial_brightness}")
#         return frame

#     def check_brightness_change(self):
#         print("Press 's' to start checking for brightness change...")
#         while True:
#             ret, frame = self.cap.read()
#             if not ret:
#                 print("Restarting video...")
#                 self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)  # Restart video
#                 continue

#             frame = self.resize_frame(frame)

#             x, y, w, h = self.roi
#             roi_frame = frame[y:y+h, x:x+w]
#             cv2.imshow("Current Frame", roi_frame)

#             key = cv2.waitKey(1) & 0xFF
#             if key == ord('s'):
#                 break
#             if key == ord('q'):
#                 self.release()
#                 exit()

#         print("Checking screen brightness change started...")
#         wait_time = 10
#         # time.sleep(wait_time)

#         while True:
#             ret, frame = self.cap.read()
#             if not ret:
#                 print("Restarting video...")
#                 self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
#                 continue

#             frame = self.resize_frame(frame)
#             roi_frame = frame[y:y+h, x:x+w]
#             cv2.imshow("Current Frame", roi_frame)
#             if cv2.waitKey(1) & 0xFF == ord('q'):
#                 break

#             current_brightness = self.get_brightness(frame)
#             print(f"Current Brightness: {current_brightness}")

#             if current_brightness - self.initial_brightness > self.brightness_threshold:
#                 print("Screen turned on!")
#                 print("BRIGHTNESS VALUE: ", current_brightness)
#                 print("INITIAL BRIGHTNESS: ", self.initial_brightness)
#                 self.changed_frame = frame
#                 return frame

#         print("No brightness change detected.")
#         return None

#     def release(self):
#         self.cap.release()
#         cv2.destroyAllWindows()

# if __name__ == "__main__":
#     video_path = "/home/robothon/Desktop/output_video1.avi"  # Replace with the actual video file path
#     screen_monitor = ScreenMonitor(video_source=video_path, brightness_threshold=20)

#     try:
#         screen_monitor.select_roi()
#         screen_monitor.capture_initial_brightness()
#         changed_frame = screen_monitor.check_brightness_change()

#         if changed_frame is not None:
#             x, y, w, h = screen_monitor.roi
#             roi_frame = changed_frame[y:y+h, x:x+w]
#             cv2.imshow('Screen On', roi_frame)
#             cv2.waitKey(0)

#     finally:
#         screen_monitor.release()


import cv2
import numpy as np

# Open the video file or webcam feed
cap = cv2.VideoCapture('/home/robothon/Desktop/output_video1.avi')  # Replace with 0 to use the webcam

if not cap.isOpened():
    print("Error: Could not open video.")
    exit()

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    # Convert the frame from BGR to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Create the mask
    mask1 = cv2.inRange(hsv, (146, 94, 187), (180, 115, 255))

    # Apply the mask
    result = cv2.bitwise_and(frame, frame, mask=mask1)

    # Show the original frame and the masked output
    cv2.imshow('Original Frame', frame)
    cv2.imshow('Masked Output', result)

    # Wait for a short period to display frames
    if cv2.waitKey(30) & 0xFF == ord('q'):
        break

# Release the video capture and close windows
cap.release()
cv2.destroyAllWindows()
