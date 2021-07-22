import BoxOffice as bof
from pointcloud import Pointcloud


print('Welcome to the Box Office!')

point_src = "../data/active/testing_2/down__testing_2_random_sampling_0.2.txt" #'../data/in/bunny_classy.off'
class_src = "../data/active/testing_2/down__testing_2.ol" #'../data/in/bunny_classy.ol' #

parent_cloud = Pointcloud()
parent_cloud.load_from_txt(point_src)
# cloudy.save_to_txt('/home/boxy/BoxOffice/data/active/testing_2/intermediate/yo.txt')


###############
# SAMPLE DOWN #
###############

# TODO: add loop adaptive to method
# down_method = ['rand_samp', 'min_dist_samp']
# for method in down_method:


# random sampling
rand_ratio = [0.2, 0.6, 0.8]
downsampled = []
# for ratio in rand_ratio:
#     downsampled.append(cloudy.sample_down('rand_samp', ratio))

# distance sampling
min_dist = [0.01, 0.02, 0.03]
for dist in min_dist:
    downsampled.append(parent_cloud.sample_down('min_dist_samp', dist))


###############
# MAKE BBOXES #
###############

class_of_interest = 1
decomp_depth = 0
gain_thresh = 0.99

oh_my_path = "../data/active/testing_2/intermediate/bbox_dump_temp/"
box_vertices = []
box_inliers = []
for downsampled_cloud in downsampled:
    [vertices, inliers] = downsampled_cloud.get_bbox_inliers(class_src, class_of_interest, decomp_depth, gain_thresh, oh_my_path)
    box_vertices.append(vertices)
    box_inliers.append(inliers)

#############
# SAMPLE UP #
#############

# k = [3, 5, 7]
k = 5

# TODO: add loop adaptive to method
# up_method = ['near_one', 'near_k']
# for method in up_method:

upsampled_cloud = []
method = 'near_one'
for downsampled_cloud in downsampled:
    upsampled_cloud.append(downsampled_cloud.sample_up(method, k))

a=0



# evaluate

