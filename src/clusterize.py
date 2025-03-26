import math
from typing import List, Tuple


def find(parent: List[int], i: int) -> int:
    """Finds the root of element i in the disjoint set with path compression.

    Args:
        parent: List representing the parent relationships in disjoint set.
        i: Index of the element to find root for.

    Returns:
        Root index of the set containing element i.
    """
    if parent[i] != i:
        parent[i] = find(parent, parent[i])
    return parent[i]


def union(parent: List[int], rank: List[int], i: int, j: int) -> None:
    """Merges two sets in the disjoint set data structure using union by rank.

    Args:
        parent: List representing the parent relationships.
        rank: List tracking the rank (depth) of each set.
        i: Index of first element.
        j: Index of second element.
    """
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
    """Groups nearby coordinates into clusters and computes their average properties.

    Args:
        coords: List of tuples containing (x, y, area) coordinates.
        threshold: Maximum distance between points to be considered in same cluster.

    Returns:
        List of tuples containing averaged (x, y, area) for each cluster.
    """
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
