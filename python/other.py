import open3d as o3d
import numpy as np


def comp_ratio(rem_no, raw_no):
    v = float(rem_no) / float(raw_no)
    return v


def bbox_cut(cloud, bbox): # cloud full OR ds!
    vol = o3d. visualization.read_selection_polygon_volume(bbox)
    cloud_cut = vol.crop_point_loud(cloud)

    return cloud_cut


def save_cloud(cloud, path_within_data, data_path):
    print('you did nothing')


def label_retriever_cloud(cloud_xyz, cloud_xyzl): # cloud should be ds..
    cloud_relabeled = np.zeros([cloud_xyz.shape[0], 4])
    cloud_relabeled[:, :-1] = cloud_xyz

    for point in cloud_xyz:
        ind = np.where(
            (cloud_xyzl[:, 0] == point[0]) &
            (cloud_xyzl[:, 1] == point[1]) &
            (cloud_xyzl[:, 2] == point[2])
        )
        # label = cloud_xyzl[ind, -1]
        cloud_relabeled[ind, -1] = cloud_xyzl[ind, -1]

    return cloud_relabeled


# def label_retriever_point(point, cloud_xyzl): # cloud should be ds..
#
#     ind = np.where(
#         (cloud_xyzl[:, 0] == point[0]) &
#         (cloud_xyzl[:, 1] == point[1]) &
#         (cloud_xyzl[:, 2] == point[2])
#     )
#
#     label = cloud_xyzl[ind, -1]
#     return label

def to_xyz(cloud):
    xyz_cloud = cloud[:, 0:3]
    return xyz_cloud

