import numpy as np

# labels of relabeled vs raw cloud! AKA PURE SAMPLING ERROR "FPR_2_a"

def calc_pure_sampling_error(cloud_in, cloud_relabeled):

    if cloud_in.shape[0] != cloud_relabeled.shape[0]:
        print('cloud size error, trying to calc pure sampling error')
        raise

    else:
        sherlock = cloud_in[:, -1] != cloud_relabeled[:, -1]
        watson = np.count_nonzero(sherlock)
        fpr_2 = float(watson)/float(cloud_in.shape[0])

    return fpr_2