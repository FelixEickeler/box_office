import numpy as np
from other import comp_ratio, to_xyz, label_retriever_cloud
from sklearn.neighbors import KDTree

def stat_out(cloud, raw_no, stat_out_mean_k, std_dev_mul_thresh, trial_name):
    print('   ->statistical outlier filter')
    filt = cloud.make_statistical_outlier_filter()
    filt.set_mean_k(stat_out_mean_k)
    filt.set_std_dev_mul_thresh(std_dev_mul_thresh)
    to_file = './data/out/' + trial_name + '_statistical_outliers.pcd'
    rem_no = filt.filter().size
    filt.filter().to_file(to_file)

    v = comp_ratio(rem_no, raw_no)
    print('OK - remaining points: ' + str(rem_no) + ', v=' + str(round(v, 3)))

    return rem_no, v


def rand_samp(cloud, raw_no, rand_ratio, trial_name, active_path):
    print('   ->random sampling')
    np_cloud = cloud # np.asarray(cloud)
    rem_no = int(round(raw_no * rand_ratio, 0))
    rem_ind = np.random.choice(raw_no, size=rem_no, replace=False)
    rem_cloud_np = np_cloud[rem_ind, :]
    to_file_txt = active_path + '/' + 'down__' + trial_name + '_random_sampling_' + str(rand_ratio) + '.txt'
    np.savetxt(to_file_txt, rem_cloud_np, fmt="%1.6f")
    # rem_cloud_pcl = pcl.PointCloud_PointXYZI(rem_cloud_np)
    # to_file = './data/out/' + trial_name + '_random_sampling.pcd'
    # rem_cloud_pcl.to_file(to_file)

    v = comp_ratio(rem_no, raw_no)
    print('OK - remaining points: ' + str(rem_no) + ', v=' + str(round(v, 3)))

    return rem_no, v, rem_cloud_np


def min_dist_samp_np(cloud, raw_no, min_dist, trial_name, active_path):
    print('   ->space sampling')
    np_cloud = cloud
    # np_cloud = np.asarray(cloud)
    tracker = np.zeros((np_cloud.shape[0], np_cloud.shape[1] + 1))
    tracker[:, :-1] = np_cloud

    run_cond = True
    while run_cond:
        dep = np.where(tracker[:, -1] == 0)[0][0]
        poi = tracker[dep, :-1]
        tracker[dep][-1] = 1

        dists = np.sqrt(
            (tracker[:, 0] - poi[0]) ** 2
            + (tracker[:, 1] - poi[1]) ** 2
            + (tracker[:, 2] - poi[2]) ** 2
        )

        dists[dep] = 999  # weak
        tracker = tracker[dists >= min_dist]

        if np.array_equal(poi, tracker[-1, :-1]):
            run_cond = False

    rem_no = tracker.shape[0]
    rem_cloud_np = np.float32(tracker[:, :-1])
    # rem_cloud_pcl = pcl.PointCloud(rem_cloud_np)
    # to_file = './data/out/' + trial_name + '_space_sampling.pcd'
    # rem_cloud_pcl.to_file(to_file)

    to_file_txt = active_path + '/' + 'down__' + trial_name + '_mind_dist_sampling_' + str(min_dist) + '.txt'
    np.savetxt(to_file_txt, rem_cloud_np, fmt="%1.6f")

    v = comp_ratio(rem_no, raw_no)
    print('OK - remaining points: ' + str(rem_no) + ', v=' + str(round(v, 3)))

    return rem_no, v, rem_cloud_np

def min_dist_samp_sk(cloud, raw_no, min_dist, trial_name):
    print('   ->space sampling')
    np_cloud = cloud
    np_cloud = to_xyz(np_cloud)

    tracker = np.zeros((np_cloud.shape[0], np_cloud.shape[1] + 1))
    tracker[:, :-1] = np_cloud

    run_cond = True
    while run_cond:

        dep = np.where(tracker[:, -1] == 0)[0][0]
        poi = tracker[dep, :-1]
        tracker[dep][-1] = 1
        np_cloud_xyz = np_cloud[:, :-1]
        cloud_tree = KDTree(np_cloud_xyz)

        distances, indices = cloud_tree.query(np_cloud_xyz, k=2)
        dists = distances[:, 1]
        # dists = np.sqrt(
        #     (tracker[:, 0] - poi[0]) ** 2
        #     + (tracker[:, 1] - poi[1]) ** 2
        #     + (tracker[:, 2] - poi[2]) ** 2
        # )
        for louis in zip(dists, indices[1]):
            if louis[0] <= min_dist and louis[1] == 8:
                print('sepp')


        dists[dep] = 999  # weak
        tracker = tracker[dists >= min_dist]

        if np.array_equal(poi, tracker[-1, :-1]):
            run_cond = False

    rem_no = tracker.shape[0]
    rem_cloud_np = np.float32(tracker[:, :-1])

    rem_cloud_np_l = label_retriever_cloud(rem_cloud_np, np_cloud)

    # rem_cloud_pcl = pcl.PointCloud(rem_cloud_np)
    # to_file = './data/out/' + trial_name + '_space_sampling.pcd'
    # rem_cloud_pcl.to_file(to_file)
    to_file_txt = './data/out/' + trial_name + '_mind_dist_sampling.txt'
    np.savetxt(to_file_txt, rem_cloud_np)

    v = comp_ratio(rem_no, raw_no)
    print('OK - remaining points: ' + str(rem_no) + ', v=' + str(round(v, 3)))

    return rem_no, v



def vox_grid(cloud, raw_no, vox_leaf_size, trial_name):
    print('   ->voxel grid filter')
    filt = cloud.make_voxel_grid_filter()
    filt.set_leaf_size(vox_leaf_size, vox_leaf_size, vox_leaf_size)
    to_file = './data/out/' + trial_name + '_voxel_grid.pcd'
    rem_no = filt.filter().size
    filt.filter().to_file(to_file)

    v = comp_ratio(rem_no, raw_no)
    print('OK - remaining points: ' + str(rem_no) + ', v=' + str(round(v, 3)))

    return rem_no, v


def bilat_filt(cloud, raw_no, bilat_param, trial_name):
    print('   ->bilateral filtering')

    print('NO - nada')