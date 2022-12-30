import numpy as np
import matplotlib.pyplot as plt

def key_points(o_dict):
    offsets1 = [(1, 0), (0, 1), (-1, 0), (1, 0)]
    offsets2 = [(1, 1), (-1, 1), (-1, -1), (1, -1)]
    offsets3 = [(0, 1), (-1, 0), (0, -1), (0, -1)]
    c_list = []
    for grid_point, obs_status in o_dict.items():
        if obs_status:
            continue
        empty_space = True
        x, y = grid_point
        for i in [-1, 0, 1]:
            for j in [-1, 0, 1]:
                if (x + i, y + j) not in o_dict.keys():
                    continue
                if o_dict[(x + i, y + j)]:
                    empty_space = False
                    break
        if empty_space:
            continue
        for offset1, offset2, offset3 in zip(offsets1, offsets2, offsets3):
            i1, j1 = offset1
            i2, j2 = offset2
            i3, j3 = offset3
            if ((x + i1, y + j1) not in o_dict.keys()) or \
                    ((x + i2, y + j2) not in o_dict.keys()) or \
                    ((x + i3, y + j3) not in o_dict.keys()):
                continue
            obs_count = 0
            if o_dict[(x + i1, y + j1)]:
                obs_count += 1
            if o_dict[(x + i2, y + j2)]:
                obs_count += 1
            if o_dict[(x + i3, y + j3)]:
                obs_count += 1
            if obs_count == 3 or obs_count == 1:
                c_list.append((x, y))
                if show_animation:
                    plt.plot(x, y, ".y")
                break
    if only_corners:
        return c_list