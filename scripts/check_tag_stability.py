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
import time

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
    ts = []
    Rs = []
    for i in range(100):
        while not rospy.is_shutdown():

            try:
                (marker_trans, marker_rot) = listener.lookupTransform(
                    "/world", "/aruco_581", rospy.Time(0)
                )
                print("Marker translation: ", marker_trans)
                print("Marker rotation: ", marker_rot)
                ts.append(marker_trans)
                Rs.append(marker_rot)
                break

            except (
                tf.LookupException,
                tf.ConnectivityException,
                tf.ExtrapolationException,
            ):
                continue
        time.sleep(.2)
            

    # Compute the angular distance between marker_rot and tagframe_rot
    ths = []
    for marker_rot in Rs:
        th = np.arccos((np.trace(R.from_quat(marker_rot).as_matrix()) - 1) / 2)
        ths.append(th)

    print("Trans varience: ", np.std(ts, axis=0))
    print("Rot variance: ", np.std(np.rad2deg(ths)))





if __name__ == "__main__":
    main()
