import BoxOffice as bof
from pointcloud import Pointcloud
print('welcome')

point_src = "../data/active/testing_2/down__testing_2_random_sampling_0.2.txt" #'../data/in/bunny_classy.off'
class_src = "../data/active/testing_2/down__testing_2.ol" #'../data/in/bunny_classy.ol' #

cloudy = Pointcloud()
cloudy.load_from_txt(point_src)
# cloudy.save_to_txt('/home/boxy/BoxOffice/data/active/testing_2/intermediate/yo.txt')

###############
# sample down
###############
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
# make bboxes
###############
class_of_interest = 1
decomp_depth = 1
gain_thresh = 0.99

boxes

szene = bof.create_scene(point_src, class_src)
boxof = szene.get_object(class_of_interest)

###############
# sample up
###############
k = 5

# evaluate





bbox = szene.get_object(1).boxof.decompose(2, 0.99)

a= 0