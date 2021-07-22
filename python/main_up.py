import numpy as np
from sklearn.metrics import accuracy_score

from up import near_one, near_k

# MAIN is load and upsample individual downsampled / labeled pcds
def down_data_loop(case, load_string, trial_name, k, raw_path, data_path):

    raw_cloud_np = np.float32(np.loadtxt(raw_path))  # XYZRGBL
    # raw_cloud_np = np.delete(raw_cloud_np, -1, 1)  # XYZL / "I"
    raw_no = raw_cloud_np.shape[0]
    # raw_cloud_np_xyz = np.delete(raw_cloud_np, -1, 1)
    raw_cloud_np_xyz = np.array(raw_cloud_np[:, :3], copy=True)

    down_cloud_np = np.float32(np.loadtxt(load_string))
    down_no = down_cloud_np.shape[0]

    cloud_relabeled_near_one = near_one(down_cloud_np, raw_cloud_np_xyz, raw_no, trial_name, data_path)
    cloud_relabeled_near_k = near_k(down_cloud_np, raw_cloud_np_xyz, raw_no, trial_name, k, data_path)

    # fpr_2_near_one = calc_pure_sampling_error(raw_cloud_np, cloud_relabeled_near_one)
    fpr_2_near_one = 1 - accuracy_score(raw_cloud_np[:, -1], cloud_relabeled_near_one[:, -1])
    fpr_2_near_k = 1 - accuracy_score(raw_cloud_np[:, -1], cloud_relabeled_near_k[:, -1])

    print('down ' + case + ', up nearest one, fpr_2=' + str(round(fpr_2_near_one*100, 4)) + '%')
    print('down ' + case + ', up nearest k=' + str(k) + ', fpr_2=' + str(round(fpr_2_near_k*100, 4)) + '%')


if __name__ == "__main__":
    trial_name = 'testing_2'
    downsampling = ['random_sampling_0.2', 'mind_dist_sampling_0.02']
    k = 5
    data_path = '/home/boxy/BoxOffice/data'
    raw_path = data_path + '/in/' + 'one_pipe_01_cut.txt'
    #down_data = ['./data/out/down__testing_2_random_sampling.txt', './data/out/down__testing_2_mind_dist_sampling.txt']
    for case in downsampling:
        load_string = data_path + '/active/' + trial_name + '/down__' + trial_name + '_' + case + '.txt'
        down_data_loop(case, load_string, trial_name, k, raw_path, data_path)
