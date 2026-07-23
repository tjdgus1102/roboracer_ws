#!/usr/bin/env python3
"""Offline tool: extract a closed-loop centerline path from a ROS map
(yaml + image) via skeletonization, and save it as the x,y,yaw CSV
format that path_follower.py replays through the MPPI controller.

Usage:
    python3 extract_centerline.py <map.yaml> <output.csv> [--smooth N] [--step M]
"""
import argparse
import math
import os
import sys

import numpy as np
import yaml
from PIL import Image
import networkx as nx
from skimage.morphology import skeletonize
from skimage.measure import label, regionprops


def load_map(yaml_path):
    with open(yaml_path, 'r') as f:
        meta = yaml.safe_load(f)
    img_path = os.path.join(os.path.dirname(yaml_path), meta['image'])
    img = np.array(Image.open(img_path).convert('L'))
    resolution = meta['resolution']
    origin = meta['origin']
    negate = meta.get('negate', 0)
    free_thresh = meta.get('free_thresh', 0.196)
    return img, resolution, origin, negate, free_thresh


def free_space_mask(img, negate, free_thresh):
    # ROS map convention: pixel -> occupancy probability via
    # occ = (255 - pixel) / 255 (flipped if negate=1). A pixel counts as
    # free only if comfortably below free_thresh - this also excludes the
    # mid-gray "unknown" pixels (occ ~ 0.5) that a naive brightness
    # threshold would otherwise misclassify as free.
    if negate:
        img = 255 - img
    occ = (255.0 - img.astype(np.float32)) / 255.0
    return occ < (free_thresh * 0.5)


def isolate_track_ring(free_mask):
    """Some maps are pure line art (thin wall outlines on a white canvas)
    rather than filled free/occupied regions. With 8-connectivity the free
    background, the track corridor, and the infield all stay merged into
    one blob through the thin walls, so skeletonizing the whole mask just
    traces the outer perimeter instead of the track centerline.

    Using 4-connectivity instead makes the thin walls act as real
    barriers, splitting free space into separate components. The track
    corridor is then identifiable as the one bounded, non-border-touching
    component that has a hole in it (the infield) - i.e. Euler number <= 0.
    """
    labeled = label(free_mask, connectivity=1)
    h, w = free_mask.shape
    border_labels = set(labeled[0, :]) | set(labeled[-1, :]) | set(labeled[:, 0]) | set(labeled[:, -1])
    border_labels.discard(0)

    candidates = []
    for region in regionprops(labeled):
        if region.label in border_labels:
            continue  # unbounded background
        if region.euler_number <= 0:
            candidates.append(region)

    if not candidates:
        raise RuntimeError(
            'Could not find a closed track ring (bounded free-space region with a hole). '
            'This map may not be a closed-loop track.'
        )
    best = max(candidates, key=lambda r: r.area)
    return labeled == best.label


def pixel_to_world(row, col, height, resolution, origin):
    x = origin[0] + (col + 0.5) * resolution
    y = origin[1] + (height - row - 0.5) * resolution
    return x, y


def skeleton_to_graph(skel):
    ys, xs = np.nonzero(skel)
    coords = set(zip(ys.tolist(), xs.tolist()))
    g = nx.Graph()
    g.add_nodes_from(coords)
    neighbors = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]
    for (r, c) in coords:
        for dr, dc in neighbors:
            n = (r + dr, c + dc)
            if n in coords:
                g.add_edge((r, c), n)
    return g


def prune_spurs(g, min_branch_len=15):
    """Iteratively remove short dead-end branches (degree-1 chains) that
    aren't part of the main loop, e.g. skeleton artifacts near doorways."""
    changed = True
    while changed:
        changed = False
        leaves = [n for n in g.nodes if g.degree(n) == 1]
        for leaf in leaves:
            # walk the branch until a junction (degree != 2) is found
            branch = [leaf]
            prev, cur = None, leaf
            while True:
                nbrs = [n for n in g.neighbors(cur) if n != prev]
                if len(nbrs) != 1:
                    break
                prev, cur = cur, nbrs[0]
                branch.append(cur)
                if g.degree(cur) != 2:
                    break
            if len(branch) < min_branch_len and g.degree(branch[-1]) > 2:
                g.remove_nodes_from(branch[:-1])
                changed = True
    return g


def find_main_cycle(g):
    cycles = nx.cycle_basis(g)
    if not cycles:
        raise RuntimeError('No closed loop found in the map skeleton - is the track a closed circuit?')
    return max(cycles, key=len)


def order_cycle(g, cycle_nodes):
    sub = g.subgraph(cycle_nodes).copy()
    # drop any chord edges so every node has degree 2 (a pure ring)
    for n in list(sub.nodes):
        while sub.degree(n) > 2:
            nbrs = list(sub.neighbors(n))
            sub.remove_edge(n, nbrs[-1])
    start = cycle_nodes[0]
    ordered = [start]
    prev, cur = None, start
    while True:
        nbrs = [n for n in sub.neighbors(cur) if n != prev]
        if not nbrs:
            break
        nxt = nbrs[0]
        if nxt == start:
            break
        ordered.append(nxt)
        prev, cur = cur, nxt
    return ordered


def smooth(points, window):
    if window <= 1:
        return points
    pts = np.array(points)
    n = len(pts)
    kernel = np.ones(window) / window
    padded = np.vstack([pts[-window:], pts, pts[:window]])
    sm = np.zeros_like(pts)
    for d in range(2):
        sm[:, d] = np.convolve(padded[:, d], kernel, mode='same')[window:window + n]
    return sm.tolist()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('map_yaml')
    parser.add_argument('output_csv')
    parser.add_argument('--smooth', type=int, default=9, help='moving-average smoothing window (points)')
    parser.add_argument('--step', type=int, default=4, help='keep every Nth ordered skeleton point')
    args = parser.parse_args()

    img, resolution, origin, negate, free_thresh = load_map(args.map_yaml)
    free = free_space_mask(img, negate, free_thresh)
    track_ring = isolate_track_ring(free)
    skel = skeletonize(track_ring)

    g = skeleton_to_graph(skel)
    g = prune_spurs(g)
    cycle = find_main_cycle(g)
    ordered_px = order_cycle(g, cycle)

    ordered_px = ordered_px[::args.step]

    height = img.shape[0]
    world_pts = [pixel_to_world(r, c, height, resolution, origin) for (r, c) in ordered_px]
    world_pts = smooth(world_pts, args.smooth)

    n = len(world_pts)
    with open(args.output_csv, 'w') as f:
        f.write('# x,y,yaw\n')
        for i, (x, y) in enumerate(world_pts):
            nx_, ny_ = world_pts[(i + 1) % n]
            yaw = math.atan2(ny_ - y, nx_ - x)
            f.write(f'{x:.4f},{y:.4f},{yaw:.4f}\n')

    print(f'Wrote {n} waypoints to {args.output_csv}')


if __name__ == '__main__':
    main()
