from pyk4a import PyK4A
from matplotlib import pyplot as plt
from pyk4a.calibration import CalibrationType
import open3d as o3d
import numpy as np
def main():
    # Capture a depth image and an rgb image from the camera.

    # Load camera with the default config
    k4a = PyK4A(
        device_id=1,
    )
    k4a.start()

    # Get the next capture (blocking function)
    capture = k4a.get_capture()
    img_color = capture.color
    img_depth = capture.depth
    capture_pts = capture.depth_point_cloud

    # # Display with pyplot
    # plt.imshow(img_color[:, :, 2::-1]) # BGRA to RGB
    # plt.show()

    # get the transform from the rgb frame to the depth frame.
    R, t = k4a.calibration.get_extrinsic_parameters(CalibrationType.DEPTH, CalibrationType.COLOR)
    print(R, t)

    # get the intrinsics from the depth camera.
    K = k4a.calibration.get_camera_matrix(CalibrationType.DEPTH)
    print(K)

    # Backproject the depth image into 3D space.
    # This is a 2D array of 3D points.
    # Do it manually, without calling the k4a api.
    # Do it vectorized.
    # Do it with numpy.

    # Get a numpy meshgrid of the pixel coordinates.
    # This is a 2D array of 2D points.
    xx, yy = np.meshgrid(np.arange(img_depth.shape[1]), np.arange(img_depth.shape[0]))

    # Reshape the meshgrid into a 2D array of 2D points.
    # This is a 2D array of 2D points.
    xx = xx.reshape(-1)
    yy = yy.reshape(-1)

    # Get the depth values.
    # This is a 1D array of depth values.
    zz = img_depth.reshape(-1)

    # Get the camera intrinsics.
    # This is a 2D array of camera intrinsics.
    KK = np.array(K).reshape(3, 3)

    # Get the inverse of the camera intrinsics.
    # This is a 2D array of camera intrinsics.
    KK_inv = np.linalg.inv(KK)

    # Get the 2D points in the camera frame.
    # This is a 2D array of 2D points.
    points_2d = np.stack([xx, yy, np.ones_like(xx)], axis=0)

    # Get the 3D points in the camera frame.
    # This is a 2D array of 3D points.
    points_3d = KK_inv @ points_2d

    # Scale the 3D points by the depth values.
    # This is a 2D array of 3D points.
    points_3d = points_3d * zz

    # breakpoint()



    # # Visualize the 3D point cloud.
    # pcd2 = o3d.geometry.PointCloud()
    # pcd2.points = o3d.utility.Vector3dVector(points_3d.T)
    # pcd2.paint_uniform_color([1, 0.706, 0])
    # # o3d.visualization.draw_geometries([pcd3])

    # pcd = o3d.geometry.PointCloud()
    # pcd.points = o3d.utility.Vector3dVector(capture_pts.reshape(-1, 3))
    # pcd.paint_uniform_color([0, 0.651, 0.929])
    # o3d.visualization.draw_geometries([pcd, pcd2])

    # Visualize color to depth points
    color_3d = capture.transformed_color
    color = capture.color
    plt.subplot(1, 2, 1)
    plt.imshow(color[:, :, 2::-1])
    plt.subplot(1, 2, 2)
    plt.imshow(color_3d[:, :, 2::-1])
    plt.show()

    # color_to_depth = k4a.calibration.color_to_depth_3d()
    # pcd3 = o3d.geometry.PointCloud()
    # pcd3.points = o3d.utility.Vector3dVector(color_to_depth @ color_3d.T)






if __name__ == "__main__":
    main()