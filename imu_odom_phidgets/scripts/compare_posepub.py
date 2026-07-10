#!/usr/bin/env python3
# Compare /posePub_merged messages between two rosbag files.
# Usage: python3 compare_posepub.py recorded.bag original.bag

import sys
import math

try:
    import rosbag
    from geometry_msgs.msg import PoseStamped
    from state_estimator_msgs.msg import Estimator
except Exception as e:
    print('Missing ROS python packages. Run this from a ROS-enabled Python environment.')
    raise


def extract_poses(bag_path):
    poses = []  # tuples (stamp, position[3], orientation[4])
    with rosbag.Bag(bag_path, 'r') as bag:
        for topic, msg, t in bag.read_messages(topics=['/posePub_merged']):
            # accept Estimator message shape
            try:
                est = msg
                p = est.pose.pose.position
                o = est.pose.pose.orientation
                stamp = est.header.stamp.to_sec() if hasattr(est, 'header') else t.to_sec()
                poses.append((stamp, (p.x, p.y, p.z), (o.w, o.x, o.y, o.z)))
            except Exception:
                continue
    return poses


def mean_abs_err(a, b):
    # Pair each sample in `a` with the nearest-in-time sample in `b`.
    if len(a) == 0 or len(b) == 0:
        return None
    pos_err = 0.0
    ori_err = 0.0
    paired = 0
    # naive nearest-neighbour pairing (sufficient for bag sizes here)
    for sa, pa, oa in a:
        best_dt = None
        best_pb = None
        best_ob = None
        for sb, pb, ob in b:
            dt = abs(sa - sb)
            if best_dt is None or dt < best_dt:
                best_dt = dt
                best_pb = pb
                best_ob = ob
        # only accept pairs within 20 ms
        if best_dt is not None and best_dt <= 0.02:
            for k in range(3):
                pos_err += abs(pa[k] - best_pb[k])
            for k in range(4):
                ori_err += abs(oa[k] - best_ob[k])
            paired += 1
    if paired == 0:
        return None
    return pos_err / (paired*3), ori_err / (paired*4)


if __name__ == '__main__':
    if len(sys.argv) != 3:
        print('Usage: compare_posepub.py recorded.bag original.bag')
        sys.exit(1)
    rec_bag = sys.argv[1]
    orig_bag = sys.argv[2]
    print('Extracting poses from', rec_bag)
    a = extract_poses(rec_bag)
    print('Extracting poses from', orig_bag)
    b = extract_poses(orig_bag)
    print('Counts: recorded=%d, original=%d' % (len(a), len(b)))
    mae_pos, mae_ori = mean_abs_err(a, b)
    if mae_pos is None:
        print('No overlapping poses to compare')
        sys.exit(2)
    print('Mean absolute position error (m):', mae_pos)
    print('Mean absolute orientation component error:', mae_ori)
    # Optionally print first few paired samples
    n = min(5, len(a), len(b))
    print('\nFirst %d paired samples (recorded -> original):' % n)
    for i in range(n):
        print(i, 'rec_time=%.6f orig_time=%.6f pos_rec=%s pos_orig=%s' % (a[i][0], b[i][0], a[i][1], b[i][1]))
