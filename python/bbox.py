import numpy as np
import open3d as o3d
from sklearn.metrics import accuracy_score
import BoxOffice as boff


def o3d_rotation_workaround_box(bbox_t, R):

    bbox_verts = np.asarray(bbox_t.get_box_points())
    bbox_verts_rotated = np.dot(bbox_verts, R)
    bbox_t_rotated = o3d.geometry.OrientedBoundingBox.create_from_points(o3d.utility.Vector3dVector(bbox_verts_rotated))

    return bbox_t_rotated


def o3d_rotation_workaround_cloud(cloud_t, R):

    cloud_points = np.asarray(cloud_t.points)
    cloud_points_rotated = np.dot(cloud_points, R)
    cloud_t_rotated = o3d.geometry.PointCloud()
    cloud_t_rotated.points = o3d.utility.Vector3dVector(cloud_points_rotated)

    return cloud_t_rotated


def points_from_box_OLD(cloud, bbox, data_path, trial_name):
    """workaround for get_point_indices_within_bounding_box (o3d OBB)

    in: bbox and cloud in n x 3 numpy arrays
    out: points of cloud that are within bbox"""

    # nr_boxes = int(bbox.shape[0] / 8)
    # bboxes = []
    # for i in range(nr_boxes):
    # current_box = bbox[i*8: i*8+8, :]

    cloud_np = np.array(cloud[:, :3], copy=True)
    np.savetxt(data_path + '/active/' + trial_name + '/intermediate/cloud_IN.txt', cloud_np)
    bbox_np = np.array(bbox[:, :3], copy=True)
    np.savetxt(data_path + '/active/' + trial_name + '/intermediate/bbox_IN.txt', bbox_np)

    obbox_o3_vec = o3d.utility.Vector3dVector(bbox[:, 0:3])
    obbox_o3 = o3d.geometry.OrientedBoundingBox.create_from_points(obbox_o3_vec)

    key_vert = np.asarray(obbox_o3.get_box_points())[:4, :]
    T = key_vert[0]
    key_vert = key_vert - T

    # to_file_txt = './data/out/beauty_testing/key_vert_original.txt'
    # np.savetxt(to_file_txt, key_vert[:4, :])

    x1 = np.array([1, 0, 0])
    y1 = np.array([0, 1, 0])
    z1 = np.array([0, 0, 1])
    axis_dirs = [x1, y1, z1]

    key_v = [key_vert[i, :] / np.sqrt(key_vert[i, :].dot(key_vert[i, :])) for i in range(1, 4)]

    R = np.array([
        [np.dot(key_v[0], axis_dirs[0]),
         np.dot(key_v[0], axis_dirs[1]),
         np.dot(key_v[0], axis_dirs[2])],
        [np.dot(key_v[1], axis_dirs[0]),
         np.dot(key_v[1], axis_dirs[1]),
         np.dot(key_v[1], axis_dirs[2])],
        [np.dot(key_v[2], axis_dirs[0]),
         np.dot(key_v[2], axis_dirs[1]),
         np.dot(key_v[2], axis_dirs[2])]
    ])

    # new_bbox = [np.dot(R, key_v[i]) for i in range(0, 3)]
    # np.savetxt('./data/out/beauty_testing/bbox_AA.txt', new_bbox)

    # cloud translation
    o3_cloud_3dvec = o3d.utility.Vector3dVector(cloud[:, :3])
    o3_cloud = o3d.geometry.PointCloud()
    o3_cloud.points = o3_cloud_3dvec

    np.savetxt(data_path + '/active/' + trial_name + '/intermediate/bbox_0.txt', np.asarray(obbox_o3.get_box_points()))
    bbox_t = obbox_o3.translate(-T)
    ### numpy o3d rotation workaround #1
    # bbox_t = o3d_rotation_workaround_box(bbox_t, R)
    bbox_t = bbox_t.rotate(R, center=(0,0,0))
    np.savetxt(data_path + '/active/' + trial_name + '/intermediate/bbox_AA.txt', np.asarray(bbox_t.get_box_points()))

    np.savetxt(data_path + '/active/' + trial_name + '/intermediate/cloud_0.txt', np.asarray(o3_cloud.points))
    cloud_t = o3d.geometry.PointCloud()
    cloud_t.points = o3_cloud.points
    cloud_t.translate(-T)
    ### numpy o3d rotation workaround #2
    # cloud_t = o3d_rotation_workaround_cloud(cloud_t, R)
    cloud_t = cloud_t.rotate(R, center=(0,0,0))
    np.savetxt(data_path + '/active/' + trial_name + '/intermediate/cloud_AA.txt', np.asarray(cloud_t.points))

    AABB = o3d.geometry.AxisAlignedBoundingBox.create_from_points(bbox_t.get_box_points())
    inliers = AABB.get_point_indices_within_bounding_box(cloud_t.points)

    inlier_cloud_AA = np.asarray(cloud_t.points)[inliers]
    np.savetxt(data_path + '/active/' + trial_name + '/intermediate/cut_AA.txt', inlier_cloud_AA)
    # inlier_cloud = np.asarray(o3_cloud.points)[inliers]
    inlier_cloud = cloud[inliers]
    np.savetxt(data_path + '/active/' + trial_name + '/intermediate/cut_0.txt', inlier_cloud)


    # inlier_cloud_labeled = np.ones([inlier_cloud.shape[0], inlier_cloud.shape[1]+1])
    # inlier_cloud_labeled[:, :3] = inlier_cloud
    inlier_labels_box = np.ones([inlier_cloud.shape[0]])
    inlier_labels_src = cloud[inliers, -1]

    fpr_0 = accuracy_score(inlier_labels_src, inlier_labels_box)

    return fpr_0

def points_from_box(cloud_xyzl, bbox):
    """
    workaround for get_point_indices_within_bounding_box (o3d OBB)

    in: bbox and cloud in n x 3 numpy arrays
    out: points of cloud that are within bbox
    """

    cloud = np.array(cloud_xyzl[:, :3], copy=True)
    # np.savetxt(data_path + '/active/' + trial_name + '/intermediate/cloud_IN.txt', cloud)
    bbox = np.array(bbox[:, :3], copy=True)
    # np.savetxt(data_path + '/active/' + trial_name + '/intermediate/bbox_IN.txt', bbox)

    obbox_o3_vec = o3d.utility.Vector3dVector(bbox[:, 0:3])
    obbox_o3 = o3d.geometry.OrientedBoundingBox.create_from_points(obbox_o3_vec)

    key_vert = np.asarray(obbox_o3.get_box_points())[:4, :]
    T = key_vert[0]
    key_vert = key_vert - T

    # to_file_txt = './data/out/beauty_testing/key_vert_original.txt'
    # np.savetxt(to_file_txt, key_vert[:4, :])

    x1 = np.array([1, 0, 0])
    y1 = np.array([0, 1, 0])
    z1 = np.array([0, 0, 1])
    axis_dirs = [x1, y1, z1]

    key_v = [key_vert[i, :] / np.sqrt(key_vert[i, :].dot(key_vert[i, :])) for i in range(1, 4)]

    R = np.array([
        [np.dot(key_v[0], axis_dirs[0]),
         np.dot(key_v[0], axis_dirs[1]),
         np.dot(key_v[0], axis_dirs[2])],
        [np.dot(key_v[1], axis_dirs[0]),
         np.dot(key_v[1], axis_dirs[1]),
         np.dot(key_v[1], axis_dirs[2])],
        [np.dot(key_v[2], axis_dirs[0]),
         np.dot(key_v[2], axis_dirs[1]),
         np.dot(key_v[2], axis_dirs[2])]
    ])

    # new_bbox = [np.dot(R, key_v[i]) for i in range(0, 3)]
    # np.savetxt('./data/out/beauty_testing/bbox_AA.txt', new_bbox)

    # cloud translation
    o3_cloud_3dvec = o3d.utility.Vector3dVector(cloud[:, :3])
    o3_cloud = o3d.geometry.PointCloud()
    o3_cloud.points = o3_cloud_3dvec

    # np.savetxt(data_path + '/active/' + trial_name + '/intermediate/bbox_0.txt', np.asarray(obbox_o3.get_box_points()))
    bbox_t = obbox_o3.translate(-T)
    ### numpy o3d rotation workaround #1
    # bbox_t = o3d_rotation_workaround_box(bbox_t, R)
    bbox_t = bbox_t.rotate(R, center=(0, 0, 0))
    # np.savetxt(data_path + '/active/' + trial_name + '/intermediate/bbox_AA.txt', np.asarray(bbox_t.get_box_points()))

    # np.savetxt(data_path + '/active/' + trial_name + '/intermediate/cloud_0.txt', np.asarray(o3_cloud.points))
    cloud_t = o3d.geometry.PointCloud()
    cloud_t.points = o3_cloud.points
    cloud_t.translate(-T)
    ### numpy o3d rotation workaround #2
    # cloud_t = o3d_rotation_workaround_cloud(cloud_t, R)
    cloud_t = cloud_t.rotate(R, center=(0, 0, 0))
    # np.savetxt(data_path + '/active/' + trial_name + '/intermediate/cloud_AA.txt', np.asarray(cloud_t.points))

    AABB = o3d.geometry.AxisAlignedBoundingBox.create_from_points(bbox_t.get_box_points())
    inliers = AABB.get_point_indices_within_bounding_box(cloud_t.points)

    inlier_cloud_AA = np.asarray(cloud_t.points)[inliers]
    # np.savetxt(data_path + '/active/' + trial_name + '/intermediate/cut_AA.txt', inlier_cloud_AA)
    # inlier_cloud = np.asarray(o3_cloud.points)[inliers]
    inlier_cloud = cloud_xyzl[inliers]
    # np.savetxt(data_path + '/active/' + trial_name + '/intermediate/cut_0.txt', inlier_cloud)


    # inlier_cloud_labeled = np.ones([inlier_cloud.shape[0], inlier_cloud.shape[1]+1])
    # inlier_cloud_labeled[:, :3] = inlier_cloud
    inlier_labels_box = np.ones([inlier_cloud.shape[0]])
    inlier_labels_src = cloud_xyzl[inliers, -1]

    fpr_0 = 1 - accuracy_score(inlier_labels_src, inlier_labels_box)

    return inlier_cloud, fpr_0

# trial_name = 'testing_2'
#
# data_path = '/home/boxy/BoxOffice/data'
#
# cloud_np = np.loadtxt(data_path + '/in/one_pipe_01_cut.txt')
# bbox_np = np.loadtxt(data_path + '/in/down__bbox_twoturn.txt')
#
# points_src = '/home/boxy/BoxOffice/data/in/one_pipe_01_cut.txt'#data_path + '/active/testing_2/down__testing_2_random_sampling_0.2.txt'
# classes_src = data_path + '/active/testing_2/down__testing_2.ol'
#
# boff_scene = boff.create_scene(points_src, classes_src)
#
# decompose_depth = 3
# print('depth of box decomposition: ', decompose_depth)
#
# boxes = boff_scene.get_object(1).decompose(decompose_depth, 0.99)
# bbox_np = np.zeros([len(boxes), 8, 3])
#
# fpr_1_sum = 0
#
#
# for box_nr in range(len(boxes)):
#     bbox_np[box_nr] = boxes[box_nr].bounding_box.get_vertices()
#     fpr_1 = 1 - points_from_box(cloud_np, bbox_np[box_nr])
#     print('fpr_1 =', round(fpr_1, 4))
#     fpr_1_sum += fpr_1
#
# fpr_1_mean = fpr_1_sum / len(boxes)
# print('mean fpr_1 = ', round(fpr_1_mean, 4))
