from scipy.spatial import cKDTree
from collections import deque
import numpy as np


def euclidian_clusters(xyz, r):
    tree = cKDTree(xyz, compact_nodes=False, balanced_tree=False)
    to_check = deque()
    cluster_ids = np.zeros(xyz.shape[0], dtype=int)
    processed = np.zeros(xyz.shape[0], dtype=bool)

    cluster_id = 0
    for i in range(xyz.shape[0]):
        if processed[i]:
            continue

        processed[i] = True
        to_check.append(i)
        cluster_ids[i] = cluster_id

        while to_check:
            j = to_check.pop()
            if processed[j]:
                continue
            processed[j] = True
            for n in tree.query_ball_point(xyz[j], r=r):
                if not processed[n]:
                    to_check.append(n)
                    cluster_ids[n] = cluster_id
                    processed[n] = True

        to_check.clear()
        cluster_id = cluster_id + 1

    return cluster_ids


def uniform_downsample(xyz, r, use_centers=False):
    v, i = np.unique(xyz // r, axis=0, return_index=True)
    return v+r/2 if use_centers else xyz[i]


if __name__ == '__main__':
    print(euclidian_clusters(np.array([[0, 0, 0], [1, 1, 1]]), 2))