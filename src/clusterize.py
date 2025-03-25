import math
from typing import List, Tuple


def find(parent: List[int], i: int) -> int:
    if parent[i] != i:
        parent[i] = find(parent, parent[i])
    return parent[i]


def union(parent: List[int], rank: List[int], i: int, j: int) -> None:
    ri = find(parent, i)
    rj = find(parent, j)
    if ri != rj:
        if rank[ri] < rank[rj]:
            parent[ri] = rj
        elif rank[ri] > rank[rj]:
            parent[rj] = ri
        else:
            parent[rj] = ri
            rank[ri] += 1


def clusterize_coords(
    coords: List[Tuple[float, float, float]], threshold: float = 0.6
) -> List[Tuple[float, float, float]]:
    n = len(coords)
    if n == 0:
        return []
    parent = list(range(n))
    rank = [0] * n

    # Union points if their distance is within the threshold
    for i in range(n):
        for j in range(i + 1, n):
            dx = coords[i][0] - coords[j][0]
            dy = coords[i][1] - coords[j][1]
            if math.sqrt(dx * dx + dy * dy) <= threshold:
                union(parent, rank, i, j)

    # Group points by their cluster root
    clusters = {}
    for i in range(n):
        root = find(parent, i)
        clusters.setdefault(root, []).append(coords[i])

    # For each cluster, compute average x, y and average area
    cluster_points = []
    for group in clusters.values():
        avg_x = sum(pt[0] for pt in group) / len(group)
        avg_y = sum(pt[1] for pt in group) / len(group)
        avg_area = sum(pt[2] for pt in group) / len(group)
        cluster_points.append((avg_x, avg_y, avg_area))
    return cluster_points
