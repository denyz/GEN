import numpy as np

from nuscenes import nuscenes

# for show corners
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import numpy as np

# for judge point in box
import open3d as o3d

# for coordinate systems transformations
from vod.frame import homogeneous_transformation


def extended_points_in_box(box: nuscenes.Box, points: np.ndarray, wlh_factor: float = 1.0,
                           wlh_offset: float = 0.0, use_z: bool = True) -> np.ndarray:
    """ Returns a mask indicating whether or not each point is inside the bounding box.

    Inspired by NuScenes points_in_box.

    Arguments:
        box: Bounding box in nuScenes format.
        points: Radar points (3, N).
        wlh_factor: Factor to inflate or deflate the box (1.1 makes it 10% larger in all dimensions).
        wlh_offset: Offset to inflate or deflate the box (1.0 makes it 1 m larger in all dimensions, on both sides).
        use_z: Whether the z coordinate is taken into account.

    Returns:
        mask: Mask indicating whether or not each point is inside the bounding box (N,).
    """
    corners = box.corners(wlh_factor=wlh_factor)

    p1 = corners[:, 0]
    p_x = corners[:, 4]
    p_y = corners[:, 1]
    p_z = corners[:, 3]

    i = p_x - p1
    j = p_y - p1
    k = p_z - p1

    v = points - p1.reshape((-1, 1))

    iv = np.dot(i, v) / np.linalg.norm(i)
    jv = np.dot(j, v) / np.linalg.norm(j)
    kv = np.dot(k, v) / np.linalg.norm(k)

    mask_x = np.logical_and(0 - wlh_offset <= iv, iv <= np.linalg.norm(i) + wlh_offset)
    mask_y = np.logical_and(0 - wlh_offset <= jv, jv <= np.linalg.norm(j) + wlh_offset)

    if use_z:
        mask_z = np.logical_and(0 - wlh_offset <= kv, kv <= np.linalg.norm(k) + wlh_offset)
        mask = np.logical_and(np.logical_and(mask_x, mask_y), mask_z)
    else:
        mask = np.logical_and(mask_x, mask_y)

    return mask


def extended_points_in_box(box: nuscenes.Box, points: np.ndarray, wlh_factor: float = 1.0,
                           wlh_offset: float = 0.0, use_z: bool = True) -> np.ndarray:
    """ Returns a mask indicating whether or not each point is inside the bounding box.

    Inspired by NuScenes points_in_box.

    Arguments:
        box: Bounding box in nuScenes format.
        points: Radar points (3, N).
        wlh_factor: Factor to inflate or deflate the box (1.1 makes it 10% larger in all dimensions).
        wlh_offset: Offset to inflate or deflate the box (1.0 makes it 1 m larger in all dimensions, on both sides).
        use_z: Whether the z coordinate is taken into account.

    Returns:
        mask: Mask indicating whether or not each point is inside the bounding box (N,).
    """
    corners = box.corners(wlh_factor=wlh_factor)

    p1 = corners[:, 0]
    p_x = corners[:, 4]
    p_y = corners[:, 1]
    p_z = corners[:, 3]

    i = p_x - p1
    j = p_y - p1
    k = p_z - p1

    v = points - p1.reshape((-1, 1))

    iv = np.dot(i, v) / np.linalg.norm(i)
    jv = np.dot(j, v) / np.linalg.norm(j)
    kv = np.dot(k, v) / np.linalg.norm(k)

    mask_x = np.logical_and(0 - wlh_offset <= iv, iv <= np.linalg.norm(i) + wlh_offset)
    mask_y = np.logical_and(0 - wlh_offset <= jv, jv <= np.linalg.norm(j) + wlh_offset)

    if use_z:
        mask_z = np.logical_and(0 - wlh_offset <= kv, kv <= np.linalg.norm(k) + wlh_offset)
        mask = np.logical_and(np.logical_and(mask_x, mask_y), mask_z)
    else:
        mask = np.logical_and(mask_x, mask_y)

    return mask


def array_to_pointcloud(np_array):
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(np_array)
    return pcd


def _data_trans(data):
    lst = []
    for num in data:
        num_list = num.split()
        lst.append([eval(i) for i in num_list])
    lst.pop()
    return lst


def conv_hull(points: np.ndarray):
    """
    生成凸包 参考文档：https://blog.csdn.net/io569417668/article/details/106274172
    :param points: 待生成凸包的点集
    :return: 索引 lists
    """
    pcl = array_to_pointcloud(points)
    hull, lst = pcl.compute_convex_hull()
    return lst
    '''
	这里的load_data_txt是我自己写的函数，主要是读入三维点坐标，返回list
	array_to_point_cloud是用来把NdArray类型的点坐标转换成o3d.geometry.PointCloud类型的函数
    '''


def in_convex_polyhedron(points_set: np.ndarray, test_points: np.ndarray):
    """
    检测test_points点是否在凸包的角点points_set内
    :param points_set: 凸包，需要对分区的点进行凸包生成 具体见conv_hull函数
    :param test_points: 需要检测的点 可以是多个点
    :return: bool类型
    """
    assert type(points_set) == np.ndarray
    assert type(points_set) == np.ndarray
    bol = np.zeros((test_points.shape[0], 1), dtype=bool)
    ori_set = points_set
    ori_edge_index = conv_hull(ori_set)
    ori_edge_index = np.sort(np.unique(ori_edge_index))
    for i in range(test_points.shape[0]):
        new_set = np.concatenate((points_set, test_points[i, np.newaxis]), axis=0)
        new_edge_index = conv_hull(new_set)
        new_edge_index = np.sort(np.unique(new_edge_index))
        bol[i] = (new_edge_index.tolist() == ori_edge_index.tolist())
    return bol


# For VOD dataset

# KITTI offer
# def compute_3d_box_cam2(h, w, l, x, y, z, yaw):
#     """
#     Return : 3xn in cam2 coordinate
#     """
#     R = np.array([[np.cos(yaw), 0, np.sin(yaw)], [0, 1, 0], [-np.sin(yaw), 0, np.cos(yaw)]])
#     x_corners = [l / 2, l / 2, -l / 2, -l / 2, l / 2, l / 2, -l / 2, -l / 2]
#     y_corners = [0, 0, 0, 0, -h, -h, -h, -h]
#     z_corners = [w / 2, -w / 2, -w / 2, w / 2, w / 2, -w / 2, -w / 2, w / 2]
#     corners_3d_cam2 = np.dot(R, np.vstack([x_corners, y_corners, z_corners]))
#     corners_3d_cam2[0, :] += x
#     corners_3d_cam2[1, :] += y
#     corners_3d_cam2[2, :] += z
#     return corners_3d_cam2


def draw_point_cloud(ax, points, axes=[0, 1, 2], point_size=10, xlim3d=None, ylim3d=None, zlim3d=None):
    """
    Convenient method for drawing various point cloud projections as a part of frame statistics.
        """
    #point_size, lidar:0.003, radar:10
    axes_limits = [
        [0, 40],
        [-20, 20],
        [-20, 20]
    ]
    axes_str = ['X', 'Y', 'Z']
    ax.grid(False)

    # ax.scatter(*np.transpose(points[:, axes]), s=point_size, c=points[:,3], cmap='gray')
    # ax.scatter(points[0, :], points[1,:], points[2,:], s=point_size, c='r', cmap='gray')
    ax.scatter(*np.transpose(points[:, axes]), s=point_size, c='b', cmap='gray')
    ax.set_xlabel('{} axis'.format(axes_str[axes[0]]))
    ax.set_ylabel('{} axis'.format(axes_str[axes[1]]))
    if len(axes) > 2:
        ax.set_xlim3d(*axes_limits[axes[0]])
        ax.set_ylim3d(*axes_limits[axes[1]])
        ax.set_zlim3d(*axes_limits[axes[2]])
        ax.xaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))
        ax.yaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))
        ax.zaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))
        ax.set_zlabel('{} axis'.format(axes_str[axes[2]]))
    else:
        ax.set_xlim(*axes_limits[axes[0]])
        ax.set_ylim(*axes_limits[axes[1]])
    # User specified limits
    if xlim3d != None:
        ax.set_xlim3d(xlim3d)
    if ylim3d != None:
        ax.set_ylim3d(ylim3d)
    if zlim3d != None:
        ax.set_zlim3d(zlim3d)

def draw_Mask(ax, points, axes=[0, 1, 2], point_size=11, xlim3d=None, ylim3d=None, zlim3d=None):
        """
        Convenient method for drawing various point cloud projections as a part of frame statistics.
            """
        # point_size, lidar:0.003, radar:10
        axes_limits = [
            [-20, 20],
            [-20, 20],
            [-20, 20]
        ]
        axes_str = ['X', 'Y', 'Z']
        ax.grid(False)

        # ax.scatter(*np.transpose(points[:, axes]), s=point_size, c=points[:,3], cmap='gray')
        # ax.scatter(points[0, :], points[1,:], points[2,:], s=point_size, c='r', cmap='gray')
        ax.scatter(*np.transpose(points[:, axes]), s=point_size, c='r', cmap='gray')
        ax.set_xlabel('{} axis'.format(axes_str[axes[0]]))
        ax.set_ylabel('{} axis'.format(axes_str[axes[1]]))
        if len(axes) > 2:
            ax.set_xlim3d(*axes_limits[axes[0]])
            ax.set_ylim3d(*axes_limits[axes[1]])
            ax.set_zlim3d(*axes_limits[axes[2]])
            ax.xaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))
            ax.yaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))
            ax.zaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))
            ax.set_zlabel('{} axis'.format(axes_str[axes[2]]))
        else:
            ax.set_xlim(*axes_limits[axes[0]])
            ax.set_ylim(*axes_limits[axes[1]])
        # User specified limits
        if xlim3d != None:
            ax.set_xlim3d(xlim3d)
        if ylim3d != None:
            ax.set_ylim3d(ylim3d)
        if zlim3d != None:
            ax.set_zlim3d(zlim3d)

def draw_box(pyplot_axis, vertices, axes=[0, 1, 2], color='green'):
    """
    Draws a bounding 3D box in a pyplot axis.

    Parameters
    ----------
    pyplot_axis : Pyplot axis to draw in.
    vertices    : Array 8 box vertices containing x, y, z coordinates.
    axes        : Axes to use. Defaults to `[0, 1, 2]`, e.g. x, y and z axes.
    color       : Drawing color. Defaults to `black`.
    """
    pyplot_axis.grid(False)

    axes_str = ['X', 'Y', 'Z']

    pyplot_axis.set_xlabel('{} axis'.format(axes_str[axes[0]]))
    pyplot_axis.set_ylabel('{} axis'.format(axes_str[axes[1]]))
    if len(axes) > 2:
        pyplot_axis.set_zlabel('{} axis'.format(axes_str[axes[2]]))

    vertices = vertices[axes, :]
    connections = [
        [0, 1], [1, 2], [2, 3], [3, 0],  # Lower plane parallel to Z=0 plane
        [4, 5], [5, 6], [6, 7], [7, 4],  # Upper plane parallel to Z=0 plane
        [0, 4], [1, 5], [2, 6], [3, 7]  # Connections between upper and lower planes
    ]
    for connection in connections:
        pyplot_axis.plot(*vertices[:, connection], c=color, lw=0.5)


# Rotation around x-axis
def rotx(r):
    c = np.cos(r)
    s = np.sin(r)
    return np.array([[1, 0, 0],
                     [0, c, -s],
                     [0, s, c]])


# Rotation around y-axis
def roty(r):
    c = np.cos(r)
    s = np.sin(r)
    return np.array([[c, 0, s],
                     [0, 1, 0],
                     [-s, 0, c]])


# Rotation around z-axis
def rotz(r):
    c = np.cos(r)
    s = np.sin(r)
    return np.array([[c, -s, 0],
                     [s, c, 0],
                     [0, 0, 1]])


# rot_matrix = np.array([[np.cos(rotation), -np.sin(rotation), 0],
#                        [np.sin(rotation), np.cos(rotation), 0],
#                        [0, 0, 1]])


# from nuscenes
def center2corner_3d(x, y, z, w, l, h, r):
    x_corners = [-l / 2, -l / 2, l / 2, l / 2, -l / 2, -l / 2, l / 2, l / 2]
    y_corners = [w / 2, -w / 2, -w / 2, w / 2, w / 2, -w / 2, -w / 2, w / 2]
    z_corners = [0, 0, 0, 0, h, h, h, h]  # for kitti3d dataset
    # z_corners = [-h/2,-h/2,-h/2,-h/2,h/2,h/2,h/2,h/2]  #for our lidar-coordination-based dataset
    R = rotz(r)
    corners_3d = np.dot(R, np.vstack([x_corners, y_corners, z_corners])) + np.vstack([x, y, z])

    # 交换y与轴坐标
    # corners_3d = np.transpose(corners_3d,(0,2,1))
    return corners_3d


# from KITTI 在通过3D标注框信息计算出8个角点坐标
def compute_3d_box_cam2(h, w, l, x, y, z, yaw, t_camera_lidar):
    """
    Return : 3xn in cam2 coordinate
    """
    # R = np.array([[np.cos(yaw), 0, np.sin(yaw)], [0, 1, 0], [-np.sin(yaw), 0, np.cos(yaw)]])
    yaw = -(yaw + np.pi / 2)

    R = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                  [np.sin(yaw), np.cos(yaw), 0],
                  [0, 0, 1]])

    x_corners = [l / 2, l / 2, -l / 2, -l / 2, l / 2, l / 2, -l / 2, -l / 2]
    y_corners = [w / 2, -w / 2, -w / 2, w / 2, w / 2, -w / 2, -w / 2, w / 2]
    # y_corners = [0,0,0,0,-h,-h,-h,-h]
    # z_corners = [w/2,-w/2,-w/2,w/2,w/2,-w/2,-w/2,w/2]
    z_corners = [0, 0, 0, 0, h, h, h, h]

    center = (np.linalg.inv(t_camera_lidar) @ np.array([x, y, z, 1]))[:3]

    corners_3d_cam2 = np.dot(R, np.vstack([x_corners, y_corners, z_corners])) + center.reshape(3, 1)

    # corners_3d_cam2[0,:] += x
    # corners_3d_cam2[1,:] += y
    # corners_3d_cam2[2,:] += z
    return corners_3d_cam2


def show_corners(corners_3d: np.ndarray):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(corners_3d[0], corners_3d[1], corners_3d[2])
    ax.set_xlabel('X Label')
    ax.set_ylabel('Y Label')
    ax.set_zlabel('Z Label')
    plt.show()


def extended_points_in_3Dbox(box: nuscenes.Box, points: np.ndarray, transMatrix: np.ndarray, wlh_factor: float = 1.0,
                             wlh_offset: float = 0.0, use_z: bool = True) -> np.ndarray:
    # for VOD 3D BBox label
    """ Returns a mask indicating whether or not each point is inside the bounding box.

    Inspired by NuScenes points_in_box.

    Arguments:
        box: Bounding box in nuScenes format.
        points: Radar points (3, N).
        wlh_factor: Factor to inflate or deflate the box (1.1 makes it 10% larger in all dimensions).
        wlh_offset: Offset to inflate or deflate the box (1.0 makes it 1 m larger in all dimensions, on both sides).
        use_z: Whether the z coordinate is taken into account.

    Returns:
        mask: Mask indicating whether or not each point is inside the bounding box (N,).
    """

    # nuscenes corners : (3,8) array，也就是3D包围框的角点。
    # https://blog.csdn.net/weixin_38705903/article/details/89301383
    # corners = box.corners(wlh_factor=wlh_factor)

    # For VOD 3D bbox, box前三位保存了物体的3D中心坐标。
    # corners = center2corner_3d(box[3], box[4], box[5], box[0], box[1], box[2], box[6])
    # corners = center2corner_3d(box[3], box[4], box[5], trans_box[0, 2], trans_box[0, 0], trans_box[0, 1], box[6])
    # corners = compute_3d_box_cam2(box[0],box[1],box[2],box[3],box[4],box[5], box[6])
    # corners = compute_3d_box_cam2(trans_box[0, 0], trans_box[0, 1], trans_box[0, 2], box[3], box[4], box[5], box[6])

    # box坐标转换到radar坐标系
    # trans_box = homogeneous_transformation(box.T[0:4].reshape(1, 4), transMatrix)
    # trans_corners = homogeneous_transformation(np.concatenate((corners, np.ones((1, 8))), axis=0).T, transMatrix)

    corners = compute_3d_box_cam2(box[0], box[1], box[2], box[3], box[4], box[5], box[6], transMatrix)

    #转换后的包围框中心坐标
    BBox_new = (np.linalg.inv(transMatrix) @ np.array([box[3], box[4], box[5], 1]))[:3]

    #draw
    fig = plt.figure(figsize=(20, 10))
    AX = fig.add_subplot(111, projection='3d')
    AX.view_init(40, 150)

    # draw 3D box
    # draw_box(AX, trans_corners.T)
    #根据角点画出3D包围框
    draw_box(AX, corners)
    # draw_box(AX, homogeneous_transformation(k.T, transMatrix).T[0:3, :])

    # radar 散点3D图
    draw_point_cloud(AX, points.T)


    # for debug show the corners
    # show_corners(corners)

    # 判断雷达点是否在八个角点形成的立方体中，输出mask
    #mask = in_convex_polyhedron(corners.T, points.T)


    # 鸟瞰图
    #draw_point_cloud(AX, points.T, axes=[0,1])
    #plt.show()

    p1 = corners[:, 0]
    p_x = corners[:, 4]
    p_y = corners[:, 1]
    p_z = corners[:, 3]

    i = p_x - p1
    j = p_y - p1
    k = p_z - p1

    v = points - p1.reshape((-1, 1))

    iv = np.dot(i, v) / np.linalg.norm(i)
    jv = np.dot(j, v) / np.linalg.norm(j)
    kv = np.dot(k, v) / np.linalg.norm(k)

    mask_x = np.logical_and(0 - wlh_offset <= iv, iv <= np.linalg.norm(i) + wlh_offset)
    mask_y = np.logical_and(0 - wlh_offset <= jv, jv <= np.linalg.norm(j) + wlh_offset)

    if use_z:
        mask_z = np.logical_and(0 - wlh_offset <= kv, kv <= np.linalg.norm(k) + wlh_offset)
        mask = np.logical_and(np.logical_and(mask_x, mask_y), mask_z)
    else:
        mask = np.logical_and(mask_x, mask_y)

    #for debug
    object_idx = np.flatnonzero(mask)
    #print(object_idx)
    #draw_Mask(AX, points[:,196].reshape(1,3))
    #draw_Mask(AX, BBox_new.reshape(1,3))


    return mask, BBox_new
