import numpy as np
import yaml
from scipy.spatial.transform import Rotation as R
def main(transform_folder, handeye_folder):
    for i in range(4):
        # Load the transform as a numpy array
        transform_fname = transform_folder + f"/transform_{i}.npy"
        T_CW = np.load(transform_fname)

        # Load the original yaml file:
        handeye_fname = handeye_folder + f"/k4a_{i}_eye_on_base.yaml"
        with open(handeye_fname, "r") as f:
            handeye_dict = yaml.safe_load(f)
        
        # Convert the handeye transform to a numpy array
        x = handeye_dict["transformation"]["x"]
        y = handeye_dict["transformation"]["y"]
        z = handeye_dict["transformation"]["z"]
        qx = handeye_dict["transformation"]["qx"]
        qy = handeye_dict["transformation"]["qy"]
        qz = handeye_dict["transformation"]["qz"]
        qw = handeye_dict["transformation"]["qw"]

        # Convert the handeye transform to a numpy array
        T_WC = np.eye(4)
        T_WC[:3, 3] = np.array([x, y, z])
        T_WC[:3, :3] = R.from_quat([qx, qy, qz, qw]).as_matrix()

        # Compute the new transform
        T_BW = T_CW @ T_WC

        # Convert the new transform to a dictionary
        P_BW = T_BW[:3, 3]
        R_BW = T_BW[:3, :3]
        Q_BW = R.from_matrix(R_BW).as_quat()
        PQ_BW = np.concatenate((P_BW, Q_BW))

        # Save the new transform as a yaml file
        handeye_dict["transformation"]["x"] = float(PQ_BW[0])
        handeye_dict["transformation"]["y"] = float(PQ_BW[1])
        handeye_dict["transformation"]["z"] = float(PQ_BW[2])
        handeye_dict["transformation"]["qx"] = float(PQ_BW[3])
        handeye_dict["transformation"]["qy"] = float(PQ_BW[4])
        handeye_dict["transformation"]["qz"] = float(PQ_BW[5])
        handeye_dict["transformation"]["qw"] = float(PQ_BW[6])

        # Put it in a new yaml file
        handeye_new_fname = handeye_folder + f"/k4a_{i}_eye_on_base_snapped.yaml"

        with open(handeye_new_fname, "w") as f:
            yaml.dump(handeye_dict, f)
        


if __name__ == "__main__":
    transform_folder = "/home/beisner/catkin_ws/src/rpad_ros/scripts"
    handeye_folder = "/home/beisner/.ros/easy_handeye"
    main(transform_folder=transform_folder, handeye_folder=handeye_folder)
