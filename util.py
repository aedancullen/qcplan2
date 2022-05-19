import numpy as np
from numba import njit

@njit(cache=True)
def nearest_point_on_trajectory(point, trajectory):
    '''
    Return the nearest point along the given piecewise linear trajectory.
    Same as nearest_point_on_line_segment, but vectorized. This method is quite fast, time constraints should
    not be an issue so long as trajectories are not insanely long.
        Order of magnitude: trajectory length: 1000 --> 0.0002 second computation (5000fps)
    point: size 2 numpy array
    trajectory: Nx2 matrix of (x,y) trajectory waypoints
        - these must be unique. If they are not unique, a divide by 0 error will destroy the world
    '''
    diffs = trajectory[1:,:] - trajectory[:-1,:]
    l2s   = diffs[:,0]**2 + diffs[:,1]**2
    # this is equivalent to the elementwise dot product
    # dots = np.sum((point - trajectory[:-1,:]) * diffs[:,:], axis=1)
    dots = np.empty((trajectory.shape[0]-1, ))
    for i in range(dots.shape[0]):
        dots[i] = np.dot((point - trajectory[i, :]), diffs[i, :])
    t = dots / l2s
    t[t<0.0] = 0.0
    t[t>1.0] = 1.0
    # t = np.clip(dots / l2s, 0.0, 1.0)
    projections = trajectory[:-1,:] + (t*diffs.T).T
    # dists = np.linalg.norm(point - projections, axis=1)
    dists = np.empty((projections.shape[0],))
    for i in range(dists.shape[0]):
        temp = point - projections[i]
        dists[i] = np.sqrt(np.sum(temp*temp))
    min_dist_segment = np.argmin(dists)
    return projections[min_dist_segment], dists[min_dist_segment], t[min_dist_segment], min_dist_segment

@njit(cache=True)
def walk_along_trajectory(trajectory, t, i, distance):
    iplus = i + 1
    if iplus == trajectory.shape[0]:
        iplus = 0
    acc = -t * np.linalg.norm(trajectory[iplus, :] - trajectory[i, :])
    while acc <= distance:
        i += 1
        if i == trajectory.shape[0]:
            i = 0
        acc += np.linalg.norm(trajectory[i, :] - trajectory[i - 1, :])
    lenlast = np.linalg.norm(trajectory[i, :] - trajectory[i - 1, :])
    t_out = 1 - ((acc - distance) / lenlast)
    diff = trajectory[i, :] - trajectory[i - 1, :]
    point_out = trajectory[i - 1, :] + t_out * diff
    i_out = i - 1
    if i_out == -1:
        i_out = trajectory.shape[0]

    angle_out = np.arctan2(diff[1], diff[0])
    return point_out, angle_out, t_out, i_out

@njit(cache=True)
def fast_state_validity_check(np_state, np_laserstate, laserscan, occupancygrid, car_x_dim, car_y_dim):
    ranges, angle_min, angle_max, angle_increment = laserscan
    data, resolution, width, height, x, y = occupancygrid

    cells_per = 1 / resolution

    for i in range(len(ranges)):
        dist = ranges[i]
        angle = np_laserstate[2] + angle_min + i * angle_increment
        location_x = np_laserstate[0] + dist * np.cos(angle)
        location_y = np_laserstate[1] + dist * np.sin(angle)
        bi_x = round((location_x - x) * cells_per)
        bi_y = round((location_y - y) * cells_per)
        if bi_x >= 0 and bi_x < width and bi_y >= 0 and bi_y < height:
            data[bi_y, bi_x] = 100

    direction = np_state[2]
    step = resolution / 2
    cos_step = np.cos(direction) * step
    sin_step = np.sin(direction) * step
    cos_step_perp = np.cos(direction + np.pi / 2) * step
    sin_step_perp = np.sin(direction + np.pi / 2) * step
    trans_x = np_state[0] - np.cos(direction) * car_x_dim / 2
    trans_y = np_state[1] - np.sin(direction) * car_x_dim / 2
    dist = 0
    while dist <= car_x_dim:
        trans_x_perp = 0
        trans_y_perp = 0
        dist_perp = 0
        while dist_perp <= car_y_dim / 2:
            x1 = round((trans_x + trans_x_perp - x) * cells_per)
            y1 = round((trans_y + trans_y_perp - y) * cells_per)
            x2 = round((trans_x - trans_x_perp - x) * cells_per)
            y2 = round((trans_y - trans_y_perp - y) * cells_per)
            if data[y1, x1] >= 10 or data[y2, x2] >= 10:
                return False
            trans_x_perp += cos_step_perp
            trans_y_perp += sin_step_perp
            dist_perp += step
        trans_x += cos_step
        trans_y += sin_step
        dist += step
    return True
