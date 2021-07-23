import numpy as np
import os

from down import rand_samp, min_dist_samp
from up import near_one, near_k
from bbox import points_from_box

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
        self.bboxes = None
        self.history = None

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
            cloud.parent_cloud = self.xyzl
            cloud.sampling_type = method
            cloud.sampling_parameter = sampling_parameter

            return cloud

    def sample_up(self, method, sampling_parameter):
        """
        available methods: near_one, near_k
        """
        if self.ID != 'down':
            print('please use downsampled clouds for upsampling only')
        else:
            if method == 'near_one':
                upsampled_cloud = near_one(self.xyzl, self.parent_cloud[:, :-1], sampling_parameter)
            elif method == 'near_k':
                upsampled_cloud = near_k(self.xyzl, self.parent_cloud[:, :-1], sampling_parameter)
            else:
                raise ValueError('method unknown, gtfo')
            cloud = Pointcloud()
            cloud.xyzl = upsampled_cloud
            cloud.ID = 'up'
            cloud.sampling_type = method
            cloud.sampling_parameter = sampling_parameter
            cloud.history = (self.sampling_type, self.sampling_parameter)
            cloud.parent_cloud = self.parent_cloud

            return cloud

    def get_bbox_inliers(self, class_src, class_of_interest, depth, gain, source_path):
        """
        points_src is txt path- temp storage to txt (save-read-remove)
        """
        points_src = source_path + 'temporary_dump.txt'
        self.save_to_txt(points_src)
        with open(points_src, 'r') as original: data = original.read()
        with open(points_src, 'w') as modified: modified.write("# t0p53cr3t-h45h\n" + data) #TODO: remove this disgusting workaround
        boxes = boff.create_scene(points_src, class_src).get_object(class_of_interest).decompose(depth, gain)
        self.bboxes = boxes
        os.remove(points_src)

        inlier_points = []
        fpr_0s = []

        for box in boxes:
            box_vertices = box.bounding_box.get_vertices()
            [box_inliers, fpr_0] = points_from_box(self.xyzl, box_vertices)
            inlier_points.append(box_inliers)
            fpr_0s.append(fpr_0)

            # box_inliers = points_from_box(self.xyz, box_vertices)

        return box_vertices, inlier_points
