import rospy
from sensor_msgs.msg import PointCloud2, CameraInfo
img_dict = {
    "0": None,
    "1": None,
    "2": None,
    "3": None,
}

def camera_callback(msg, cam_id):
    print("Received pointcloud from camera {}".format(cam_id))

def main():
    topic = "/k4a_0/depth_to_rgb/filtered_points"
    # topic = "/k4a_0/depth/camera_info"
    rospy.init_node("record_pointclouds")
    rospy.Subscriber(topic, PointCloud2, camera_callback, callback_args="0")

    rospy.spin()

    # a rosbag command which records the pointclouds
    # rosbag record -O pointclouds /k4a_0/depth_to_rgb/filtered_points /k4a_1/depth_to_rgb/filtered_points /k4a_2/depth_to_rgb/filtered_points /k4a_3/depth_to_rgb/filtered_points

if __name__ == "__main__":
    main()