import numpy as np
from down import rand_samp, min_dist_samp
from up import near_one, near_k
import BoxOffice as boff


class Pointcloud:
    def __init__(self):
        self.isOriginal = False
        self.xyz = np.zeros(1)
        self.xyzl = np.zeros(1)
        self.has_labels = False
        self.data_path = 'not loaded from path'
        self.trial = 'undefined'
        self.parent_cloud = None
        self.ID = None # can be 'full', 'down', or 'up'
        self.sampling_type = None
        self.sampling_parameter = None

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
        self.ID = 'full'

    def save_to_txt(self, path_to_save):
        # to_file_txt = active_path + '/' + 'down__' + trial_name + '_mind_dist_sampling_' + str(min_dist) + '.txt'
        np.savetxt(path_to_save, self.xyzl, fmt="%1.8f")

    def sample_down(self, method, sampling_parameter):
        """
        available methods: rand_samp, min_dist_samp
        """
        if self.ID != 'full':
            print('please use full clouds for downsampling only')
        else:
            if method == 'rand_samp':
                processed_cloud = rand_samp(self.xyzl, sampling_parameter)
            elif method == 'min_dist_samp':
                processed_cloud = min_dist_samp(self.xyzl, sampling_parameter)
            else:
                raise ValueError('method unknown, gtfo')
            cloud = Pointcloud()
            cloud.xyzl = processed_cloud
            cloud.ID = 'down'
            cloud.sampling_type = method
            cloud.sampling_parameter = sampling_parameter

            return cloud

    def sample_up(self, method, sampling_parameter, path_to_save):
        """
        available methods: near_one, near_k
        """
        if self.ID != 'down':
            print('please use downsampled clouds for upsampling only')
        else:
            if method == 'near_one':
                near_one(self.xyzl, self.parent_cloud.xyz, self.parent_cloud.xyz.shape[0], self.trial, path_to_save)
            elif method == 'near_k':
                near_k(self.xyzl, self.parent_cloud.xyz, self.parent_cloud.xyz.shape[0], self.trial, sampling_parameter, path_to_save)
            else:
                raise ValueError('method unknown, gtfo')

    def get_bbox_inliers(self, decomp_depth, class_of_interest):
        boxoffice = bof.create_scene(point_src, class_src).get_object(class_of_interest)
        a = 'slow down'
