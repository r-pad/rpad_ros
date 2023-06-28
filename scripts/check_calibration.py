import rospy
import math
import tf
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, PoseStamped
from scipy.spatial.transform import Rotation as R
import numpy as np

"""
This is a test script that tests the precision and quality of calibration.

We define a goal via a goal pose.
The goal pose is curr_ee_pose + delta
"""

def main():
    rospy.init_node("calibration_test")

    # Look up where the marker is
    listener = tf.TransformListener()

    rate = rospy.Rate(10.0)

    # Getting marker transformation
    while not rospy.is_shutdown():
        try:
            (marker_trans, marker_rot) = listener.lookupTransform(
                "/world", "/aruco_581", rospy.Time(0)
            )
            (tagframe_trans, tagframe_rot) = listener.lookupTransform(
                "/world", "/tag_gt_frame", rospy.Time(0)
            )
            print("Marker translation: ", marker_trans)
            print("Marker rotation: ", marker_rot)
            print("Tagframe trans: ", tagframe_trans)
            print("Tagframe rot: ", tagframe_rot)
            break
        except (
            tf.LookupException,
            tf.ConnectivityException,
            tf.ExtrapolationException,
        ):
            continue

    t_err = np.asarray(tagframe_trans) - np.asarray(marker_trans)
    t_abs_err = np.linalg.norm(t_err)
    print("Translation error: ", t_err)
    print("Absolute translation error: ", t_abs_err)

    # Compute the angular distance between marker_rot and tagframe_rot
    r_err = (R.from_quat(marker_rot).inv() * R.from_quat(tagframe_rot)).as_matrix()
    th = np.arccos((np.trace(r_err) - 1) / 2)

    print("Rotation error in deg: ", np.rad2deg(th))




if __name__ == "__main__":
    main()
