import numpy as np
from typing import List

from nuscenes import nuscenes
from sklearn.neighbors import kneighbors_graph

from gnnradarobjectdetection.graph_constructor.graph import GeometricGraph
from gnnradarobjectdetection.preprocessor.bounding_box import BoundingBox
from gnnradarobjectdetection.preprocessor.configs import GraphConstructionConfiguration
from gnnradarobjectdetection.preprocessor.radar_point_cloud import RadarPointCloud
from gnnradarobjectdetection.preprocessor.VOD import utils
from gnnradarobjectdetection.preprocessor.VOD.configs import VODDatasetConfiguration


def convert_point_cloud(points: np.ndarray, labels: np.ndarray) -> RadarPointCloud:
    """ Converts the radar points from the VOD to the RadarPointCloud format.

    nuScenes radar points are given in the following format:
    0: x
    1: y
    2: z
    3: dyn_prop
    4: id
    5: rcs
    6: vx
    7: vy
    8: vx_comp
    9: vy_comp
    10: is_quality_valid
    11: ambig_state
    12: x_rms
    13: y_rms
    14: invalid_state
    15: pdh0
    16: vx_rms
    17: vy_rms
    18: timestamp


    VOD radar points are given in the following format:
    0: x
    1: y
    2: z
    3: rcs
    4: v_r
    5: v_r_compensated
    6: time


The radar point clouds are stored in bin files.
Each bin file contains a set of points in the form of a Nx7 array, where N is the number of points, and 7 is the number of features:

[x, y, z, RCS, v_r, v_r_compensated, time]

where v_r is the relative radial velocity, and v_r_compensated is the absolute (i.e. ego motion compensated) radial velocity of the point.

time is the time id of the point, indicating which scan it originates from. E.g., a point from the current scan has a t = 0, while a point from the third most recent scan has a t = −2.

    For more information see radar data definition:
    https://github.com/tudelft-iv/view-of-delft-dataset/blob/main/docs/SENSORS_AND_DATA.md

    Arguments:
        points: Radar points in the nuScenes format (19, N).
        labels: Class labels for every radar point (N,).

    Returns:
        point_cloud: Radar point cloud.
    """
    # Initialize a RadarPointCloud instance
    point_cloud = RadarPointCloud()

    # Assign nuscenes radar channels
    #point_cloud.X_cc = np.vstack([points[0], points[1]]).T
    # point_cloud.X_seq = np.empty([num_points, 2])
    #point_cloud.V_cc = np.vstack([points[6], points[7]]).T
    #point_cloud.V_cc_compensated = np.vstack([points[8], points[9]]).T
    # point_cloud.range_sc = np.empty([num_points, 1]).T
    # point_cloud.azimuth_sc = np.empty([num_points, 1]).T
    #point_cloud.rcs = np.atleast_2d(points[5]).T
    # point_cloud.vr = np.empty([num_points, 1])
    # point_cloud.vr_compensated = np.empty([num_points, 1])
    #point_cloud.timestamp = np.atleast_2d(points[18]).T
    # point_cloud.sensor_id = np.empty([num_points, 1])
    # point_cloud.uuid = []
    # point_cloud.track_id = []
    #point_cloud.label_id = np.atleast_2d(labels).T

    # Assign VOD radar channels
    point_cloud.X_cc = np.vstack([points[0], points[1], points[2]]).T
    point_cloud.vr = np.vstack(points[3])
    point_cloud.vr_compensated = np.vstack(points[4])
    point_cloud.rcs = np.vstack(points[5])
    point_cloud.timestamp = np.atleast_2d(points[6]).T
    point_cloud.label_id = np.atleast_2d(labels).T

    return point_cloud


def build_geometric_graph(config: GraphConstructionConfiguration,
                          point_cloud: RadarPointCloud) -> GeometricGraph:
    """Builds a geometric graph from the given point cloud and configuration.

    Arguments:
        config: Graph construction configuration.
        point_cloud: Radar point cloud.

    Returns:
        geometric_graph: Geometric graph.
    """

    # build the geometric_graph
    if config.distance_definition == "X":
        distance_basis = point_cloud.X_cc
    elif config.distance_definition == "XV":
        distance_basis = np.concatenate((point_cloud.X_cc, point_cloud.V_cc_compensated), axis=1)

    geometric_graph = GeometricGraph()
    geometric_graph.X = point_cloud.X_cc
    #geometric_graph.V = point_cloud.V_cc_compensated
    geometric_graph.V = point_cloud.vr_compensated
    geometric_graph.F = {"rcs": point_cloud.rcs}

    # add time-index as node feature
    if "time_index" in config.node_features:
        timestamps = np.unique(point_cloud.timestamp)
        t_idx = np.zeros_like(point_cloud.timestamp)

        for i, _ in enumerate(timestamps):
            idx = np.where(point_cloud.timestamp == timestamps[i])[0]
            t_idx[idx] = int(i)

        # add time index to invariant geometric_graph features
        geometric_graph.add_invariant_feature("time_index", t_idx)

    geometric_graph.build(distance_basis, config.graph_construction_algorithm, k=config.k, r=config.r)
    geometric_graph.extract_node_pair_features(config.edge_features, config.edge_mode)#添加点之间的特征到graph数据
    geometric_graph.extract_single_node_features(config.node_features)#添加单点的特征到graph数据

    return geometric_graph


def convert_bounding_boxes(config: VODDatasetConfiguration,
                           point_cloud: RadarPointCloud,
                           boxes: List[nuscenes.Box],
                           transMatrix: np.ndarray,
                           wlh_factor: float = 1.0,
                           wlh_offset: float = 0.0) -> np.ndarray:
    """ Converts bounding boxes from the nuScenes format to the one configured.

    Arguments:
        config: Dataset configuration with bounding box specifications.
        point_cloud: Radar point cloud.
        boxes: Bounding boxes in the nuScenes format.
        wlh_factor: Factor to inflate or deflate the box (1.1 makes it 10% larger in all dimensions).
        wlh_offset: Offset to inflate or deflate the box (1.0 makes it 1 m larger in all dimensions, on both sides).

    Returns:
        bounding_boxes: Bounding boxes as defined in config.
    """
    # iterate through all objects and create their bounding box
    bounding_boxes = np.zeros([point_cloud.X_cc.shape[0], 5])
    bounding_boxes[:] = np.nan

    if config.bb_invariance == "en":
        # get nearest neighbor of each point in the point cloud
        A_sparse = kneighbors_graph(point_cloud.X_cc, 1, mode='connectivity', include_self=False)
        A_full = A_sparse.toarray()
        X_cc_nn = point_cloud.X_cc[np.where(A_full == 1)[1]]

    for box in boxes:
        # Get the indices of the points within the box
        points_3d = np.vstack([point_cloud.X_cc.T, np.zeros_like(point_cloud.X_cc.T[0])])
        #object_idx = utils.extended_points_in_box(box=box, points=points_3d, wlh_factor=wlh_factor, wlh_offset=wlh_offset, use_z=False)  #得到点云在包围框中的mask标识
        object_idx, BBox3D = utils.extended_points_in_3Dbox(box=box, points=points_3d[:3,], transMatrix=transMatrix, wlh_factor=wlh_factor, wlh_offset = wlh_offset, use_z = True)  # 得到点云在包围框中的mask标识

        #返回扁平化后矩阵中非零元素的位置（index）
        object_idx = np.flatnonzero(object_idx)

        # Get the bounding box bottom corners (2D)，4个3D坐标，3x4
        #corners = box.bottom_corners()
        corners_3D = utils.compute_3d_box_cam2(box[0], box[1], box[2], box[3], box[4], box[5], box[6], transMatrix)
        #按z轴坐标进行大小排列，默认情况下前四个是底部的矩形框。
        corners = corners_3D[:, 0:4]

        # Initialize a bounding box instance (2D)
        bb = BoundingBox(corners[:2, :].T, False)

        # Iterate through all points within the box
        for idx in object_idx:
            # Get relative rotated bounding box
            point_coord = point_cloud.X_cc[idx, :]
            x = point_coord[0]
            y = point_coord[1]
            relative_bb = bb.get_relative_bounding_box(x, y)

            if config.bb_invariance == "en":
                nn_coord = X_cc_nn[idx, :]
                relative_bb_rot_inv = relative_bb.relative_rotated_bb_to_rotation_invariant_representation(point_coord, nn_coord)
                bb_array = np.array([relative_bb_rot_inv.d, relative_bb_rot_inv.theta_v_p_nn_v_p_c,
                                     relative_bb_rot_inv.l, relative_bb_rot_inv.w, relative_bb_rot_inv.theta_v_p_nn_v_dir]).reshape(1, 5)

            elif config.bb_invariance == "none":
                x_c = point_coord[0] + relative_bb.x_center
                y_c = point_coord[1] + relative_bb.y_center
                bb_array = np.array([x_c, y_c, relative_bb.l, relative_bb.w, relative_bb.theta]).reshape(1, 5)

            elif config.bb_invariance == "translation":
                bb_array = np.array([relative_bb.x_center, relative_bb.y_center,
                                     relative_bb.l, relative_bb.w, relative_bb.theta]).reshape(1, 5)
            else:
                raise ValueError("Wrong invariance for bounding box selection")

            # Convert from degree to rad
            # TODO add option in configuration whether using degree or radians for the angle representations
            if config.bb_invariance == "en":
                bb_array[0, 1] = (bb_array[0, 1] * np.pi) / 180
                bb_array[0, 4] = (bb_array[0, 4] * np.pi) / 180
            else:
                bb_array[0, 4] = (bb_array[0, 4] * np.pi) / 180

            bounding_boxes[idx, :] = bb_array

    return bounding_boxes
