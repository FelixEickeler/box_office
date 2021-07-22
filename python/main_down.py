import numpy as np
import os

from down import rand_samp, min_dist_samp_np

# MAIN is load and downsample to individual pcds
if __name__ == "__main__":
    ### PARAMETERS ###
    trial_name = 'testing_2'
    # statistical outlier
    stat_out_mean_k = 20
    std_dev_mul_thresh = 1.0
    # random sampling
    rand_ratio = 0.2
    # voxel grid
    vox_leaf_size = 0.02  # type: float
    # distance sampling
    min_dist = 0.02
    # bilateral filtering
    bilat_param = 0

    data_path = '/home/boxy/BoxOffice/data'
    active_path = data_path + '/active/' + trial_name
    if not os.path.exists(active_path):
        os.makedirs(active_path)

    # standard I/O pcd, obj, ply
    raw_cloud_np = np.float32(np.loadtxt(data_path + '/in/' + 'one_pipe_01_cut.txt')) # XYZRGBL
    # raw_cloud_np = np.delete(raw_cloud_np, [3, 4, 5], 1) # XYZL / "I"
    raw_no = raw_cloud_np.shape[0]
    raw_cloud_np_xyz = np.delete(raw_cloud_np, -1, 1)

    # raw_cloud_np_i = raw_cloud_np
    # raw_cloud_np_i[:, -1] = raw_cloud_np_i[:, -1]/100

    # raw_cloud = pcl.PointCloud_PointXYZI
    # raw_cloud = pcl.PointCloud_PointXYZI(raw_cloud_np_i)
    # raw_no = raw_cloud.size

    print('***\nOK - cloud loaded with ' + str(raw_no) + ' points\n***\n')



    # apply filtering
    # stat_out(cloud, raw_no, stat_out_mean_k, std_dev_mul_thresh, trial_name)  # added
    rand_samp = rand_samp(raw_cloud_np, raw_no, rand_ratio, trial_name, active_path)
    rand_samp_cloud = rand_samp[2]
    # label_retriever(rand_samp[2][:, :-1], raw_cloud_np, raw_no)
    min_dist_samp_np(raw_cloud_np, raw_no, min_dist, trial_name, active_path)
    # vox_grid(raw_cloud, raw_no, vox_leaf_size, trial_name)
    # bilat_filt(raw_cloud, raw_no, bilat_param, trial_name)

    print('\n***\nOK - all done\n***')

    # select croup with bboxes

    # upsample
    #near_one(rand_samp_cloud, raw_cloud_np)
