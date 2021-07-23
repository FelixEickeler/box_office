import BoxOffice as bof
from pointcloud import Pointcloud
from sklearn.metrics import accuracy_score


print('Welcome to the Box Office!')

point_src = '../data/in/bunny_classy_head.txt' #"../data/active/testing_2/down__testing_2_random_sampling_0.2.txt" #'../data/in/bunny_classy.off'
class_src = '../data/in/bunny_classy_head.ol' #"../data/active/testing_2/down__testing_2.ol" #'../data/in/bunny_classy.ol' #

parent_cloud = Pointcloud()
parent_cloud.load_from_txt(point_src)
# cloudy.save_to_txt('/home/boxy/BoxOffice/data/active/testing_2/intermediate/yo.txt')



###############
# SAMPLE DOWN #
###############

# random sampling:
down_method = 'rand_samp'
rand_ratio = [0.2]#, 0.6, 0.8]
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


##### ERROR CALCULATION: E1 PURE SAMPLING ERROR
print('calculating E1: pure sampling error')
E1 = []
for sampled_cloud in upsampled:
    error = 1 - accuracy_score(sampled_cloud.parent_cloud[:, -1], sampled_cloud.xyzl[:, -1])
    now = (sampled_cloud.sampling_type, sampled_cloud.sampling_parameter)
    error_log = (error, now, sampled_cloud.history)
    E1.append(error_log)


####################
# LABEL BY BOXXING #
####################

class_of_interest = 1
decomp_depth = [0, 1, 2, 3, 4]
gain_thresh = 0.99

oh_my_path = "../data/active/testing_2/intermediate/bbox_dump_temp/" # belongs to the most digusting of fixes
for depth in decomp_depth:

box_vertices_down = []
box_inliers_down = []
for downsampled_cloud in downsampled:
    [vertices, inliers] = downsampled_cloud.get_bbox_inliers(class_src, class_of_interest, decomp_depth, gain_thresh, oh_my_path)
    box_vertices_down.append(vertices)
    box_inliers_down.append(inliers)

a=0
# TODO: record all fprs with their clouds, include consistent information on sampling methods and parameters





# evaluate

