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
import yaml
"""
This is a test script that tests the precision and quality of calibration.

We define a goal via a goal pose.
The goal pose is curr_ee_pose + delta
"""

def main(args):
    rospy.init_node("compute_calibration")

    # Look up where the marker is
    listener = tf.TransformListener()

    # ix = rospy.get_param("camera_id")
    ix = args.camera_id
    print("Using camera {}".format(ix))

    rate = rospy.Rate(10.0)

    # Getting marker transformation
    Rs = []
    ts = []
    for i in range(10):
        while not rospy.is_shutdown():
            try:
                input("Press enter to capture a frame")
                (marker_trans, marker_rot) = listener.lookupTransform(
                    f"/k4a_{ix}/camera_base", "/aruco_581", rospy.Time(0)
                )
                (tagframe_trans, tagframe_rot) = listener.lookupTransform(
                    "/world", "/tag_gt_frame", rospy.Time(0)
                )

                T_world_aruco = np.eye(4)
                T_world_aruco[:3, :3] = R.from_quat(tagframe_rot).as_matrix()
                T_world_aruco[:3, 3] = tagframe_trans

                T_cam1_aruco = np.eye(4)
                T_cam1_aruco[:3, :3] = R.from_quat(marker_rot).as_matrix()
                T_cam1_aruco[:3, 3] = marker_trans

                T_world_cam1 = T_world_aruco @ np.linalg.inv(T_cam1_aruco)
                cam_trans = T_world_cam1[:3, 3]
                cam_R = T_world_cam1[:3, :3]
                cam_rot = R.from_matrix(T_world_cam1[:3, :3]).as_quat()

                ts.append(cam_trans)
                Rs.append(cam_R)


                print("Camera translation: ", cam_trans)
                print("Camera rotation: ", cam_rot)
                

                break
            except (
                tf.LookupException,
                tf.ConnectivityException,
                tf.ExtrapolationException,
            ):
                continue
        time.sleep(.2)

    # Compute the average translation and rotation
    ts = np.asarray(ts)
    Rs = np.asarray(Rs)

    cam_t_avg = np.mean(ts, axis=0)

    # Compute the average rotation by solving a procrustes problem
    # https://en.wikipedia.org/wiki/Orthogonal_Procrustes_problem
    R_avg = np.mean(Rs, axis=0)
    U, _, Vt = np.linalg.svd(R_avg)
    R_avg = U @ Vt

    cam_rot_avg = R.from_matrix(R_avg).as_quat()

    


    print(f"  x: {cam_t_avg[0]}")
    print(f"  y: {cam_t_avg[1]}")
    print(f"  z: {cam_t_avg[2]}")
    print(f"  qx: {cam_rot_avg[0]}")
    print(f"  qy: {cam_rot_avg[1]}")
    print(f"  qz: {cam_rot_avg[2]}")
    print(f"  qw: {cam_rot_avg[3]}")

    fn = f"/home/beisner/.ros/easy_handeye/k4a_{ix}_eye_on_base.yaml"
    # Open the file as a yaml file, and put each of these under the transformation key:
    # translation: [x, y, z]
    # rotation: [qx, qy, qz, qw]
    with open(fn, "r") as f:
        contents = yaml.load(f, Loader=yaml.FullLoader)

    contents["transformation"]["x"] = float(cam_t_avg[0])
    contents["transformation"]["y"] = float(cam_t_avg[1])
    contents["transformation"]["z"] = float(cam_t_avg[2])
    contents["transformation"]["qx"] = float(cam_rot_avg[0])
    contents["transformation"]["qy"] = float(cam_rot_avg[1])
    contents["transformation"]["qz"] = float(cam_rot_avg[2])
    contents["transformation"]["qw"] = float(cam_rot_avg[3])

    with open(fn, "w") as f:
        yaml.dump(contents, f)

    




if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("--camera_id", type=int, default=0)
    args = parser.parse_args()

    main(args)
