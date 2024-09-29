
#!/usr/bin/env python3

import rospy
import cv2
import cv_bridge
from sensor_msgs.msg import Image

# Global bridge and image buffer
bridge = cv_bridge.CvBridge()
ids_image = None

def ids_image_callback(data):
    global ids_image
    try:
        # Convert ROS image message to OpenCV format
        ids_image = bridge.imgmsg_to_cv2(data, "bgr8")
    except cv_bridge.CvBridgeError as e:
        rospy.logerr(f"Error converting image: {e}")

def select_roi():
    """
    Selects an ROI from the IDS camera feed.

    Returns:
    tuple: Coordinates of the selected ROI in the format (x, y, width, height).
    """
    global ids_image

    # Check if there's an image to process
    if ids_image is None:
        print("No image available yet.")
        return None

    # Resize the image to a consistent size for display
    resized_image = cv2.resize(ids_image, (640, 480))

    # Show the image and allow ROI selection
    roi = cv2.selectROI("Select ROI", resized_image, fromCenter=False, showCrosshair=True)

    # Close the ROI selection window
    cv2.destroyWindow("Select ROI")

    # If the ROI selection is valid, return it
    if roi != (0, 0, 0, 0):
        (x, y, w, h) = roi
        return (x, y, w, h)
    else:
        print("ROI was not selected.")
        return None

def main():
    rospy.init_node('ids_cam_display_node', anonymous=True)

    # Subscribe to the IDS camera topic
    rospy.Subscriber("/ids/rgb", Image, ids_image_callback)

    # Create a window to display the image
    window_name = "IDS Camera Feed"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window_name, 640, 480)

    rate = rospy.Rate(10)  # Set the rate to 10Hz (10 frames per second)

    while not rospy.is_shutdown():
        if ids_image is not None:
            # Resize the image to a consistent size for display
            resized_image = cv2.resize(ids_image, (640, 480))

            # Display the image in the OpenCV window
            cv2.imshow(window_name, resized_image)

            # Exit if 's' is pressed, and select ROI
            key = cv2.waitKey(1) & 0xFF
            if key == ord('s'):
                coordinates = select_roi()
                if coordinates:
                    print(f"Selected ROI Coordinates: (x, y, width, height) = {coordinates}")
                break
            elif key == ord('q'):
                break

        rate.sleep()

    # Cleanup and close window
    cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
