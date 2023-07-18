# ----------------------------------------------------------------------------
# -                        Open3D: www.open3d.org                            -
# ----------------------------------------------------------------------------
# Copyright (c) 2018-2023 www.open3d.org
# SPDX-License-Identifier: MIT
# ----------------------------------------------------------------------------
"""Align multiple pieces of geometry in a global space"""

import open3d as o3d
import numpy as np
import rosbag
from sensor_msgs.msg import PointCloud
import os
# import yaml

def get_viewpoint(path):
    with open(path) as input_file:
        head = [next(input_file) for _ in range(10)]
    vp = [line for line in head if "VIEWPOINT" in line][0]
    vp = vp.split(" ")[1:]
    vp = [float(x) for x in vp]

    # The viewpoint is x y z qx qy qz qw
    # Return a transformation matrix.
    T = np.eye(4)
    T[:3, 3] = vp[:3]
    T[:3, :3] = o3d.geometry.get_rotation_matrix_from_quaternion(vp[3:])
    return T


def get_ros_point_clouds(bagdir, voxel_size):
    
    paths = [os.path.join(bagdir, str(ix), os.listdir(f"{bagdir}/{ix}")[0]) for ix in range(4)]


    pcds = []
    for i, path in enumerate(paths):
        vp = get_viewpoint(path)
        pcd = o3d.io.read_point_cloud(path)
        print(f"Loaded point cloud {i} with {len(pcd.points)} points.")
        pcd.remove_non_finite_points()  # Lots of nans.
        # breakpoint()
        pcd.transform(vp)

        # Remove everything in a bounding box of 2m x 2m x 1.1m
        pcd = pcd.crop(
            o3d.geometry.AxisAlignedBoundingBox(
                min_bound=(-1, -1, -.1), max_bound=(1, 1, 1)
            )
        )
        

        

        pcd_down = pcd.voxel_down_sample(voxel_size=voxel_size)

        # Estimate normals
        pcd_down.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(
                radius=voxel_size * 2, max_nn=30
            )
        )

        pcds.append(pcd_down)

    return pcds


def load_point_clouds(voxel_size=0.0):
    pcd_data = o3d.data.DemoICPPointClouds()
    pcds = []
    for i in range(3):
        pcd = o3d.io.read_point_cloud(pcd_data.paths[i])
        pcd_down = pcd.voxel_down_sample(voxel_size=voxel_size)
        pcds.append(pcd_down)
    return pcds


def pairwise_registration(source, target, max_correspondence_distance_coarse,
                          max_correspondence_distance_fine):
    print("Apply point-to-plane ICP")
    icp_coarse = o3d.pipelines.registration.registration_icp(
        source, target, max_correspondence_distance_coarse, np.identity(4),
        o3d.pipelines.registration.TransformationEstimationPointToPlane())
    icp_fine = o3d.pipelines.registration.registration_icp(
        source, target, max_correspondence_distance_fine,
        icp_coarse.transformation,
        o3d.pipelines.registration.TransformationEstimationPointToPlane())
    transformation_icp = icp_fine.transformation
    information_icp = o3d.pipelines.registration.get_information_matrix_from_point_clouds(
        source, target, max_correspondence_distance_fine,
        icp_fine.transformation)
    return transformation_icp, information_icp


def pairwise_registration_color(source, target):
    # Colored pointcloud registration.
    # This is implementation of following paper:
    # J. Park, Q.-Y. Zhou, V. Koltun,
    # Colored Point Cloud Registration Revisited, ICCV 2017.
    voxel_radius = [0.004, 0.002, 0.0015]
    max_iter = [50, 30, 10]
    current_transformation = np.identity(4)
    print("Colored point cloud registration ...\n")
    for scale in range(3):
        iter = max_iter[scale]
        radius = voxel_radius[scale]
        print([iter, radius, scale])

        print("1. Downsample with a voxel size %.2f" % radius)
        source_down = source.voxel_down_sample(radius)
        target_down = target.voxel_down_sample(radius)

        print("2. Estimate normal")
        source_down.estimate_normals(
            o3d.geometry.KDTreeSearchParamHybrid(radius=radius * 2, max_nn=30))
        target_down.estimate_normals(
            o3d.geometry.KDTreeSearchParamHybrid(radius=radius * 2, max_nn=30))

        print("3. Applying colored point cloud registration")
        result_icp = o3d.pipelines.registration.registration_colored_icp(
            source_down, target_down, radius, current_transformation,
            o3d.pipelines.registration.TransformationEstimationForColoredICP(),
            o3d.pipelines.registration.ICPConvergenceCriteria(
                relative_fitness=1e-6, relative_rmse=1e-6, max_iteration=iter))
        current_transformation = result_icp.transformation
        print(result_icp, "\n")
    # draw_registration_result(source, target, result_icp.transformation)
    information_icp = o3d.pipelines.registration.get_information_matrix_from_point_clouds(
        source, target, max_correspondence_distance_fine,
        result_icp.transformation)
    return current_transformation, information_icp


def full_registration(pcds, max_correspondence_distance_coarse,
                      max_correspondence_distance_fine):
    pose_graph = o3d.pipelines.registration.PoseGraph()
    odometry = np.identity(4)
    pose_graph.nodes.append(o3d.pipelines.registration.PoseGraphNode(odometry))
    n_pcds = len(pcds)
    for source_id in range(n_pcds):
        for target_id in range(source_id + 1, n_pcds):
            transformation_icp, information_icp = pairwise_registration(
                pcds[source_id], pcds[target_id],
                max_correspondence_distance_coarse,
                max_correspondence_distance_fine)
            # transformation_icp, information_icp = pairwise_registration_color(
            #     pcds[source_id], pcds[target_id])
            print("Build o3d.pipelines.registration.PoseGraph")
            if target_id == source_id + 1:  # odometry case
                odometry = np.dot(transformation_icp, odometry)
                pose_graph.nodes.append(
                    o3d.pipelines.registration.PoseGraphNode(
                        np.linalg.inv(odometry)))
                pose_graph.edges.append(
                    o3d.pipelines.registration.PoseGraphEdge(source_id,
                                                             target_id,
                                                             transformation_icp,
                                                             information_icp,
                                                             uncertain=False))
            else:  # loop closure case
                pose_graph.edges.append(
                    o3d.pipelines.registration.PoseGraphEdge(source_id,
                                                             target_id,
                                                             transformation_icp,
                                                             information_icp,
                                                             uncertain=True))
    return pose_graph


if __name__ == "__main__":
    voxel_size = 0.001

    # These are the demo point clouds.
    # pcds_down = load_point_clouds(voxel_size)

    # These are the point clouds from the rpad_ros package.
    bagdir = "/home/beisner/catkin_ws/src/rpad_ros/scripts/pointclouds2"
    pcds_down = get_ros_point_clouds(bagdir, voxel_size)

    o3d.visualization.draw(pcds_down)

    print("Full registration ...")
    max_correspondence_distance_coarse = voxel_size * 15
    max_correspondence_distance_fine = voxel_size * 1.5
    with o3d.utility.VerbosityContextManager(
            o3d.utility.VerbosityLevel.Debug) as cm:
        pose_graph = full_registration(pcds_down,
                                       max_correspondence_distance_coarse,
                                       max_correspondence_distance_fine)

    print("Optimizing PoseGraph ...")
    option = o3d.pipelines.registration.GlobalOptimizationOption(
        max_correspondence_distance=max_correspondence_distance_fine,
        edge_prune_threshold=0.25,
        reference_node=0)
    with o3d.utility.VerbosityContextManager(
            o3d.utility.VerbosityLevel.Debug) as cm:
        o3d.pipelines.registration.global_optimization(
            pose_graph,
            o3d.pipelines.registration.GlobalOptimizationLevenbergMarquardt(),
            o3d.pipelines.registration.GlobalOptimizationConvergenceCriteria(),
            option)

    print("Transform points and display")
    for point_id in range(len(pcds_down)):
        print(pose_graph.nodes[point_id].pose)
        pcds_down[point_id].transform(pose_graph.nodes[point_id].pose)
    o3d.visualization.draw(pcds_down)

    # Save the transforms in a numpy file.
    for point_id in range(len(pcds_down)):
        np.save("transform_" + str(point_id) + ".npy",
                pose_graph.nodes[point_id].pose)
