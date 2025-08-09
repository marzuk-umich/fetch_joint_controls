#!/usr/bin/env python3

import rospy
import cv2
import os
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer, Subscriber
import numpy as np

# Global flag to track if we've already saved
saved = False

def callback(rgb_msg, depth_msg):
    global saved
    if saved:
        return  # Already saved once, ignore further callbacks

    bridge = CvBridge()

    # Convert the ROS Image messages to OpenCV images
    rgb_image = bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
    depth_image = bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
    print(f"Depth image dtype: {depth_image.dtype}, shape: {depth_image.shape}")

    # Create a directory to save images
    save_dir = os.path.expanduser("~/fetch_images/rgd_depth")
    os.makedirs(save_dir, exist_ok=True)

    # Use timestamp for unique filenames
    timestamp = rospy.Time.now()
    rgb_filename = os.path.join(save_dir, f"rgb_{timestamp.secs}_{timestamp.nsecs}.png")
    depth_filename = os.path.join(save_dir, f"depth_{timestamp.secs}_{timestamp.nsecs}.png")
    depth_npy_filename = os.path.join(save_dir, f"depth_{timestamp.secs}_{timestamp.nsecs}.npy")

    # Save RGB image
    cv2.imwrite(rgb_filename, rgb_image)

    # Save Depth image
    if depth_image.dtype == np.float32 or depth_image.dtype == np.float64:
        # Depth is in meters -> Convert to millimeters and save as uint16
        depth_mm = (depth_image * 1000).astype(np.uint16)
        cv2.imwrite(depth_filename, depth_mm)

        # Also save original meters version for perfect precision
        np.save(depth_npy_filename, depth_image)
    elif depth_image.dtype == np.uint16:
        # Depth already in millimeters
        cv2.imwrite(depth_filename, depth_image)
    else:
        rospy.logwarn(f"Unsupported depth image dtype: {depth_image.dtype}. Saving as numpy only.")
        np.save(depth_npy_filename, depth_image)

    rospy.loginfo(f"✅ Saved images: {rgb_filename}, {depth_filename}, {depth_npy_filename}")

    saved = True  # Mark that we saved
    rospy.signal_shutdown('Saved one RGB and Depth image')

def main():
    rospy.init_node('save_rgb_depth_images')

    # Subscribers
    rgb_sub = Subscriber('/head_camera/rgb/image_rect_color', Image)
    depth_sub = Subscriber('/head_camera/depth_registered/image_raw', Image)

    # Synchronize the topics
    ats = ApproximateTimeSynchronizer([rgb_sub, depth_sub], queue_size=10, slop=0.1)
    ats.registerCallback(callback)

    rospy.loginfo("Saving one synchronized RGB and Depth image...")
    rospy.spin()

if __name__ == "__main__":
    main()
