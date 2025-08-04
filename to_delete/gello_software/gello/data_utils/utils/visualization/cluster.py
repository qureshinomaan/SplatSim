import numpy as np
from sklearn.cluster import KMeans


def kmeans(x, n_clusters=None, center=None, seed=0):
    if center is None:
        kmeans = KMeans(n_clusters=n_clusters, random_state=seed).fit(x)
        pred = kmeans.labels_
        center = kmeans.cluster_centers_
    else:
        pred = np.argmin(np.linalg.norm(x[..., None, :] - center, axis=-1), axis=-1)
    error = np.linalg.norm(x - center[pred], axis=-1)
    return center, pred, error
