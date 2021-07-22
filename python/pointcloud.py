import numpy as np
from down import rand_samp, min_dist_samp_np
from up import near_one, near_k


class Pointcloud:
    def __init__(self):
        self.isOriginal = False
        self.xyz = np.zeros(1)
        self.xyzl = np.zeros(1)
        self.has_labels = False
        self.data_path = 'not loaded from path'
        self.trial = 'undefined'
        self.parent_cloud = None
        self.ID = None

    def load_from_txt(self, txt_cloud_path):
        load = np.float32(np.loadtxt(txt_cloud_path))
        self.xyz = load[:, :3]
        if load.shape[1] > 3:
            self.xyzl = np.zeros([self.xyz.shape[0], 4])
            self.xyzl[:, :3] = self.xyz
            self.xyzl[:, -1] = load[:, -1]
            self.has_labels = True
        self.isOriginal = True
        self.data_path = txt_cloud_path
        self.ID = 'parent'

    def save_to_txt(self, path_to_save):
        # to_file_txt = active_path + '/' + 'down__' + trial_name + '_mind_dist_sampling_' + str(min_dist) + '.txt'
        np.savetxt(path_to_save, self.xyzl, fmt="%1.8f")

    def sample_down(self, method, sampling_parameter, path_to_save):
        """
        available methods: rand_samp, min_dist_samp
        """

        if method == 'rand_samp':
            rand_samp(self.xyzl, self.xyzl.shape[0], sampling_parameter, self.trial, path_to_save)
        elif method == 'min_dist_samp':
            min_dist_samp_np(self.xyzl, self.xyzl.shape[0], sampling_parameter, self.trial, path_to_save)
        else:
            print('method unknown, gtfo')

    def sample_up(self, method, sampling_parameter, path_to_save):
        """
        available methods: near_one, near_k
        """

        if method == 'near_one':
            near_one(self.xyzl, self.parent_cloud.xyz, self.parent_cloud.xyz.shape[0], self.trial, path_to_save)
        elif method == 'near_k':
            near_k(self.xyzl, self.parent_cloud.xyz, self.parent_cloud.xyz.shape[0], self.trial, k, path_to_save)
        else:
            print('method unknown, gtfo')
