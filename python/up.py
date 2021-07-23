import numpy as np
import open3d as o3d
# from sklearn.neighbors import KDTree
# from bbox import points_from_box


def near_one(cloud_down_xyzl, cloud_full_xyz, dead_end):

    cloud_full_nulabels = np.zeros([cloud_full_xyz.shape[0], cloud_full_xyz.shape[1] + 1])
    cloud_full_nulabels[:, :-1] = cloud_full_xyz
    # cloud_full_tbl = np.array(cloud_full_blind, copy=True)
    cloud_down_xyz = cloud_down_xyzl[:, :-1]

    pc_down = o3d.geometry.PointCloud()
    pc_down.points = o3d.utility.Vector3dVector(cloud_down_xyz)
    tree_down = o3d.geometry.KDTreeFlann(pc_down)

    for i, point in enumerate(cloud_full_nulabels):
        # best_friend = nearest neighbor in down
        point = point[:-1]
        [not_k, idx, _] = tree_down.search_knn_vector_3d(point, 1)
        label = cloud_down_xyzl[idx, -1]
        cloud_full_nulabels[i, -1] = label

    return cloud_full_nulabels


def near_k(cloud_down_xyzl, cloud_full_xyz, k):

    cloud_full_nulabels = np.zeros([cloud_full_xyz.shape[0], cloud_full_xyz.shape[1] + 1])
    cloud_full_nulabels[:, :-1] = cloud_full_xyz
    cloud_down_xyz = cloud_down_xyzl[:, :-1]

    pc_down = o3d.geometry.PointCloud()
    pc_down.points = o3d.utility.Vector3dVector(cloud_down_xyz)
    tree_down = o3d.geometry.KDTreeFlann(pc_down)

    for i, point in enumerate(cloud_full_nulabels):

        point = point[:-1]
        [not_k, idx, _] = tree_down.search_knn_vector_3d(point, k)
        labels = cloud_down_xyzl[idx, -1].astype(int)
        winner = np.bincount(labels).argmax()
        cloud_full_nulabels[i, -1] = float(winner)

    return cloud_full_nulabels


def near_one_OLD(cloud_down_xyzl, cloud_full_blind, raw_no, trial_name, data_path):

    print('   ->nearest neighbor label adoption')

    cloud_full_tbl = np.zeros([cloud_full_blind.shape[0], cloud_full_blind.shape[1] + 1])
    cloud_full_tbl[:, :-1] = cloud_full_blind
    # cloud_full_tbl = np.array(cloud_full_blind, copy=True)
    cloud_down_xyz = cloud_down_xyzl[:, :-1]

    pc_down = o3d.geometry.PointCloud()
    pc_down.points = o3d.utility.Vector3dVector(cloud_down_xyz)
    tree_down = o3d.geometry.KDTreeFlann(pc_down)

    for i, point in enumerate(cloud_full_tbl):
        # best_friend = nearest neighbor in down
        point = point[:-1]
        [not_k, idx, _] = tree_down.search_knn_vector_3d(point, 1)
        label = cloud_down_xyzl[idx, -1]
        cloud_full_tbl[i, -1] = label

    to_file_txt = data_path + '/out/up__' + trial_name + '_nearest_one.txt'
    np.savetxt(to_file_txt, cloud_full_tbl)

    # print('OK - relabeled: ' + str(cloud_full_tbl.shape[0]) + '/' + str(raw_no))

    return cloud_full_tbl


def near_k_OLD(cloud_down_xyzl, cloud_full_blind, raw_no, trial_name, k, data_path):

    print('   ->' + str(k) + ' nearest neighbors vote for label')

    cloud_full_tbl = np.zeros([cloud_full_blind.shape[0], cloud_full_blind.shape[1] + 1])
    cloud_full_tbl[:, :-1] = cloud_full_blind
    cloud_down_xyz = cloud_down_xyzl[:, :-1]

    pc_down = o3d.geometry.PointCloud()
    pc_down.points = o3d.utility.Vector3dVector(cloud_down_xyz)
    tree_down = o3d.geometry.KDTreeFlann(pc_down)

    for i, point in enumerate(cloud_full_tbl):

        point = point[:-1]
        [not_k, idx, _] = tree_down.search_knn_vector_3d(point, k)
        labels = cloud_down_xyzl[idx, -1].astype(int)
        winner = np.bincount(labels).argmax()
        cloud_full_tbl[i, -1] = float(winner)

    to_file_txt = data_path + '/out/up__' + trial_name + '_nearest_k.txt'
    np.savetxt(to_file_txt, cloud_full_tbl)

    # print('OK - relabeled: ' + str(cloud_full_tbl.shape[0]) + '/' + str(raw_no))

    return cloud_full_tbl
