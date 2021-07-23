import BoxOffice as bof
import numpy as np

from pointcloud import Pointcloud
from sklearn.metrics import accuracy_score


print('Welcome to the Box Office!')
# '../data/in/bunny_classy_head.txt' # // '../data/in/bunny_classy_head.ol' #
point_src = '../data/in/bunny_classy_head.txt' #"../data/in/down__testing_2_random_sampling_0.2.txt"  #'../data/in/bunny_classy.off'
class_src = '../data/in/bunny_classy_head.ol' #"../data/in/down__testing_2.ol"  #'../data/in/bunny_classy.ol' #

parent_cloud = Pointcloud()
parent_cloud.load_from_txt(point_src)
# cloudy.save_to_txt('/home/boxy/BoxOffice/data/active/testing_2/intermediate/yo.txt')



###############
# SAMPLE DOWN #
###############

# random sampling:
down_method = 'rand_samp'
rand_ratio = [0.38]#, 0.6, 0.8]
scenario_down = []
for ratio in rand_ratio:
    combo = (down_method, ratio)
    scenario_down.append(combo)

# distance sampling:
down_method = 'min_dist_samp'
min_dist = [0.01]#, 0.02, 0.03]
for dist in min_dist:
    combo = (down_method, dist)
    scenario_down.append(combo)

downsampled = []

for scenario in scenario_down:
    downsampled.append(parent_cloud.sample_down(scenario[0], scenario[1]))
    print('downsampling scenario: ', scenario[0], ':', scenario[1])



#############
# SAMPLE UP #
#############

# TODO: add loop adaptive to method
# nearest one:
scenario_up = [('near_one', 1)]

# nearest k:
k = [5]
for k_ in k:
    combo = ('near_k', k_)
    scenario_up.append(combo)

upsampled = []

for scenario in scenario_up:
    for down_cloud in downsampled:
        print('upsampling scenario: ', scenario[0], ':', scenario[1], 'on', down_cloud.sampling_type, ':', down_cloud.sampling_parameter)
        upsampled.append(down_cloud.sample_up(scenario[0], scenario[1]))


####################
# LABEL BY BOXXING #
####################

class_of_interest = 1
decomp_depth = [0, 1, 2]# 3, 4]
gain_thresh = 0.99

oh_my_path = "../data/active/testing_2/intermediate/bbox_dump_temp/" # belongs to the most digusting of fixes

# TODO: add labeling / boxing for parent cloud as baseline!

list_all = downsampled
list_all.append(parent_cloud)

for cloud_tbl in list_all:
    print('full:', cloud_tbl.isOriginal, 'downsampling:', cloud_tbl.sampling_type, cloud_tbl.sampling_parameter)
    box_dict = dict.fromkeys(decomp_depth)
    for depth in decomp_depth:
        print(':::depth ', depth)

        # box_vertices = []
        # box_inliers = []
        # box_object = []

        [vertices, inliers, box_obj] = cloud_tbl.get_bbox_inliers(class_src, class_of_interest, depth, gain_thresh, oh_my_path)

        # box_vertices.append(vertices)
        # box_inliers.append(inliers)
        box_dict[depth] = {
            'box_vertices': vertices,
            'box_inliers': inliers,
            'box_object': box_obj
        }

        cloud_tbl.bboxes = box_dict

parent_cloud = list_all[-1]
downsampled = list_all[:-1]


# TODO: record all fprs with their clouds, include consistent information on sampling methods and parameters

##### ERROR CALCULATION: E1 PURE SAMPLING ERROR
print('calculating E1: pure sampling error')
E1 = []
for sampled_cloud in upsampled:
    error = 1 - accuracy_score(sampled_cloud.parent_cloud[:, -1], sampled_cloud.xyzl[:, -1])
    now = (sampled_cloud.sampling_type, sampled_cloud.sampling_parameter)
    error_log = (error, now, sampled_cloud.history)
    E1.append(error_log)

##### ERROR CALCULATION: E2 PURE LABELING ERROR
print('calculating E2: pure labeling error')
E2 = []
for depth in decomp_depth:
    i = 0
    for box in range(len(parent_cloud.bboxes[depth]['box_inliers'])):
        if i == 0:
            inliers = parent_cloud.bboxes[depth]['box_inliers'][box]
        else:
            inliers = np.concatenate((inliers, parent_cloud.bboxes[depth]['box_inliers'][box]))
        i += 1
        inliers_unique = np.unique(inliers, axis=0)

        if box == len(parent_cloud.bboxes[depth]['box_inliers'])-1:
            inliers_labeled = np.array(inliers_unique, copy=True)
            inliers_labeled[:, -1] = 1 # TODO lift limitation for multi class test
            error = 1 - accuracy_score(inliers_unique[:, -1], inliers_labeled[:, -1])
            error_log = (error, depth, box+1)
            E2.append(error_log)

print('you are doing great')

##### ERROR CALCULATION: E3 INTRODUCED LABELING ERROR
print('calculating E2: pure labeling error')
E2 = []
