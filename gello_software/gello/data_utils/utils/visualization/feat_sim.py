import numpy as np
from sklearn.neighbors import KDTree


def iou(x, y):
    assert x.ndim == y.ndim == 2 and x.shape[0] == y.shape[0], ""
    tmp = np.concatenate([x, y], axis=1)

    def iou_item(t):
        i1, i2 = t[: x.shape[-1]], t[x.shape[-1] :]
        i = np.intersect1d(i1, i2)
        iou = len(i) * 1.0 / (x.shape[-1] + y.shape[-1] - len(i))
        return iou

    return np.apply_along_axis(iou_item, -1, tmp)


def feature_similarity(feat1, feat2, batchsize=400, k=128):
    """
    feat1, feat2: [N, D]
    return ious: [N]
    """

    assert feat1.ndim == 2 and feat1.shape[0] == feat2.shape[0], f"{feat1.shape} {feat2.shape}"
    num = feat1.shape[0]

    kd1 = KDTree(feat1)
    kd2 = KDTree(feat2)

    from tqdm import tqdm

    tqdm_obj = tqdm(mininterval=10, total=num)

    ious = []
    for i in range(0, num, batchsize):
        num_i = min(num - i, batchsize)
        slice_i = slice(i, i + num_i)
        knn1_i = kd1.query(feat1[slice_i], k=k)[1]
        knn2_i = kd2.query(feat2[slice_i], k=k)[1]
        iou_i = iou(knn1_i, knn2_i)  # [B]
        ious.append(iou_i)
        tqdm_obj.update(num_i)
    ious = np.concatenate(ious, axis=0)
    return ious
