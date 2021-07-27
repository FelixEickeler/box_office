import numpy as np
from sklearn.metrics import accuracy_score
import itertools

import BoxOffice as bof
from pointcloud import Pointcloud
from bbox import points_from_box
from pathlib import Path
from evaluation import calculate_PSE
# from exceptions import NotImplementedError
import csv

print('Welcome to the Box Office!')
# '../data/in/bunny_classy_head.txt' # // '../data/in/bunny_classy_head.ol' #
point_src = '../data/in/bunny_classy.txt'  # "../data/in/down__testing_2_random_sampling_0.2.txt"  #'../data/in/bunny_classy.off'
class_src = '../data/in/bunny_classy.ol'  # "../data/in/down__testing_2.ol"  #'../data/in/bunny_classy.ol' #


# TODO Make static

# cloudy.save_to_txt('/home/boxy/BoxOffice/data/active/testing_2/intermediate/yo.txt')

def load_or_save_boxes(node_list, base_path, name):
    cnt = 0
    for node in node_list:
        cnt += 1
        bbox_path = base_path / "{}_{}.txt".format(name, cnt)
        bbox_path.parent.mkdir(parents=True, exist_ok=True)
        np.savetxt(bbox_path, node.bounding_box.get_vertices(), fmt="%1.8f")


# Felix added this:

gain = 0.99
max_kappa = 4
no_cache = False
out_path = Path("../data/out")
calculate_pure_sampling_error = True
calculate_pure_labelling_error = True
calculate_introduced_labelling_error = True
calculate_introduced_sampling_error = True

if __name__ == "__main__":
    down_scenarios = [["rand_samp", ratio] for ratio in [0.1, 0.2]]
    down_scenarios += [["min_dist_samp", ratio] for ratio in [0.0137, 0.0194]]
    up_scenarios = [["near_one", ratio] for ratio in [1]]
    up_scenarios += [["near_k", ratio] for ratio in [5]]

    scene = bof.create_scene(point_src, class_src)
    down_store = {}
    up_store = {}
    scene_store = {}
    PSE = []
    PLE = []
    ILE = []
    ISE = []

    # down-sample because passive parameter
    # original_full = Pointcloud(scene.get_points())
    original_pcs = Pointcloud(scene.get_points())
    for dmethod, dparam in down_scenarios:
        dkey = "{}-{}".format(dmethod, dparam)
        tmp_path = out_path / (dkey + ".txt")
        if no_cache or not tmp_path.exists():
            down_store[dkey] = original_pcs.sample_down(dmethod, dparam)
            print('downsampling scenario: ', dmethod, ':', dparam)
            down_store[dkey].save_to_txt(tmp_path)
        else:
            tmp = Pointcloud()
            tmp.load_from_txt(tmp_path)
            down_store[dkey] = tmp
        down_store[dkey].parent_cloud = original_pcs
        down_store[dkey].ID = "down"
        scene_store[dkey] = bof.create_scene(tmp_path.__str__(), class_src.__str__())

        for umethod, uparam in up_scenarios:
            ukey = dkey + "={}-{}".format(umethod, uparam)
            up_store[ukey] = down_store[dkey].sample_up(umethod, uparam)
            up_path = out_path / (ukey + ".txt")
            up_store[ukey].save_to_txt(up_path)

            # CALCULATE PSE
            pse_error = 1 - accuracy_score(original_pcs.xyzl[:, -1], up_store[ukey].xyzl[:, -1])
            PSE.append({"pse": pse_error, "down-sampling": dmethod, "down-param": dparam, "up-sampling": umethod, "up-param": uparam, "path": up_path, "ukey": ukey})

    # TODO write PSE

    scene_list = scene.list_objects()
    for o_id, o in scene_list.items():
        obj_base_path = Path(out_path / o.get_name())
        obj_truth = Pointcloud(o.get_points())

        for kappa in range(max_kappa):
            dec_base_path = obj_base_path / "bboxes" / "{}_{}".format(kappa, gain)
            dec_base_path.parent.mkdir(parents=True, exist_ok=True)
            orig_boxes = o.decompose(kappa)
            load_or_save_boxes(orig_boxes, dec_base_path, "original")

            # CALCULATE PLE
            wrong_cls_in_obj = 0
            print("############### \t" + o.get_name() + "\t" + str(kappa))
            for node in orig_boxes:
                [inlier_points, _] = points_from_box(original_pcs.xyzl, node.bounding_box.get_vertices())
                ann_points = np.copy(inlier_points)
                ann_points[:, -1] = o.get_id()
                wrong_cls_in_obj += ann_points.shape[0] - accuracy_score(inlier_points[:, -1], ann_points[:, -1], normalize=False)
            PLE.append({"kappa": kappa, "gain": gain, "obj_name": o.get_name(), "id": o.get_id(), "ple": wrong_cls_in_obj / len(o.get_points())})

            # CALCULATE ILE
            V_orignal = sum([node.bounding_box.volume() for node in orig_boxes])
            for dkey, prediction_scene in scene_store.items():
                obj_pred = prediction_scene.get_object(o.get_id())
                pred_boxes = obj_pred.decompose(kappa, gain)
                load_or_save_boxes(pred_boxes, dec_base_path, dkey)
                V_boxes = sum([node.bounding_box.volume() for node in pred_boxes])
                ile_error = abs(V_boxes - V_orignal) / V_orignal
                [dmethod, dparam] = dkey.split("-")
                ILE.append({"kappa": kappa, "gain": gain, "obj_name": o.get_name(), "id": o.get_id(), "ile": ile_error, "down-sampling": dmethod, "param": dparam})

                # CALCULATE ISE
                for ref_nodes in orig_boxes:
                    [_, indices] = points_from_box(down_store[dkey].xyzl, ref_nodes.bounding_box.get_vertices())
                    eval_point_cloud_xyzl = np.copy(down_store[dkey].xyzl)
                    eval_point_cloud_xyzl[indices][:, -1] = o.get_id()
                    eval_point_cloud = Pointcloud(eval_point_cloud_xyzl)
                    eval_point_cloud.ID = "down"
                    eval_point_cloud.parent_cloud = original_pcs

                    for umethod, uparam in up_scenarios:
                        ukey = dkey + "={}-{}".format(umethod, uparam)
                        now = eval_point_cloud.sample_up(umethod, uparam)
                        before = up_store[ukey]
                        diff = now.xyzl[now.xyzl[:, -1] != before.xyzl[:, -1]]
                        if diff.size > 0:
                            [_, diff_inside_indices] = points_from_box(diff, ref_nodes.bounding_box.get_vertices())
                            additional = (diff[~diff_inside_indices]).shape[0]
                        else:
                            additional = 0

                        [ref_inside, _] = points_from_box(original_pcs.xyzl, ref_nodes.bounding_box.get_vertices())
                        [now_inside, _] = points_from_box(now.xyzl, ref_nodes.bounding_box.get_vertices())
                        base = np.where(ref_inside[:, -1] != now_inside[:, -1])[0]
                        ise_count = base.shape[0] + additional

                        ISE.append({"kappa": kappa, "gain": gain, "obj_name": o.get_name(), "id": o.get_id(), "ise": ise_count / ref_inside.size, "down-sampling": dmethod,
                                    "down-param": dparam, "up-sampling": umethod, "up-param": uparam})

    with open(out_path / 'bunny_pse.csv', mode='w') as csv_file:
        fieldnames = ["down-sampling", "down-param", "up-sampling", "up-param", "pse", "path", "ukey"]
        writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
        writer.writeheader()
        for row in PSE:
            writer.writerow(row)

    with open(out_path / 'bunny_ple.csv', mode='w') as csv_file:
        fieldnames = ["kappa", "gain", "obj_name", "id", "ple"]
        writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
        writer.writeheader()
        for row in PLE:
            writer.writerow(row)

    with open(out_path / 'bunny_ile.csv', mode='w') as csv_file:
        fieldnames = ["kappa", "gain", "obj_name", "id", "down-sampling", "param", "ile"]
        writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
        writer.writeheader()
        for row in ILE:
            writer.writerow(row)

    with open(out_path / 'bunny_ise.csv', mode='w') as csv_file:
        fieldnames = ["kappa", "gain", "obj_name", "id", "ise", "down-sampling", "down-param", "up-sampling", "up-param"]
        writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
        writer.writeheader()
        for row in ISE:
            writer.writerow(row)
    print('you are doing great')
