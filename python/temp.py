import BoxOffice as bof
from pointcloud import Pointcloud


print('Welcome to the Box Office!')

point_src = "../data/active/testing_2/down__testing_2_random_sampling_0.2.txt" #'../data/in/bunny_classy.off'
class_src = "../data/active/testing_2/down__testing_2.ol" #'../data/in/bunny_classy.ol' #

cloudy = Pointcloud()
cloudy.load_from_txt(point_src)
# cloudy.save_to_txt('/home/boxy/BoxOffice/data/active/testing_2/intermediate/yo.txt')


###############
# SAMPLE DOWN #
###############

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
    downsampled.append(cloudy.sample_down('min_dist_samp', dist))


###############
# MAKE BBOXES #
###############

class_of_interest = 1
decomp_depth = 0
gain_thresh = 0.99

oh_my_path = "../data/active/testing_2/intermediate/bbox_dump_temp/"
box_vertices = []
for in_cloud in downsampled:
    box_vertices.append(cloudy.get_bbox_inliers(class_src, class_of_interest, decomp_depth, gain_thresh, oh_my_path))

szene = bof.create_scene(point_src, class_src)
boxof = szene.get_object(class_of_interest)


#############
# SAMPLE UP #
#############

up_method = ['near_one', 'near_k']
k = 5

# evaluate





bbox = szene.get_object(1).boxof.decompose(2, 0.99)

a= 0