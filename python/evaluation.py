import numpy as np
import BoxOffice as bo


# labels of relabeled vs raw cloud! AKA PURE SAMPLING ERROR "FPR_2_a"

# def calc_pure_sampling_error(cloud_in, cloud_relabeled):
#
#     if cloud_in.shape[0] != cloud_relabeled.shape[0]:
#         print('cloud size error, trying to calc pure sampling error')
#         raise
#
#     else:
#         sherlock = cloud_in[:, -1] != cloud_relabeled[:, -1]
#         watson = np.count_nonzero(sherlock)
#         fpr_2 = float(watson)/float(cloud_in.shape[0])
#
#     return fpr_2


def calculate_PSE(scenes, down_scenarios, up_sampling_scenarios, output_dir):
    pse = {}
    for dmethod, dparam in down_scenarios:
        dkey = "{}-{}".format(dmethod, dparam)
        down = scenes[dkey]
        for umethod, uparam in up_sampling_scenarios:
            ukey = dkey + "_{}_{}".format(umethod, uparam)
            prediction = down.up_sample(umethod, uparam)
            prediction.parent_cloud = down
            cstore[ukey] = prediction
            tmp_path = (output_dir / ukey).with_suffix(".txt")
            prediction.save_to_txt(tmp_path)
            PSE[ukey] = (ukey, 1 - accuracy_score(original[:, -1], prediction[:, -1]))
    return pse


def calculate_PLE(cstore, boxes):
    pass


def calculate_ILE(cstore, boxe):
    pass


def calculate_ISE(cstore, boxes):
    pass
