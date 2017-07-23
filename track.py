import numpy as np
import matplotlib.pyplot as plt


def parametric_track(d, samples):
    # function for the definition of a parametric track

    # define the extremal points of the segments
    length = (4 + np.pi) * d
    t_a = 2 * d / length
    t_b = t_a + (d * np.pi / 2) / length
    t_c = t_b + 2 * d / length
    t_d = 1

    t_interval = 1. / samples
    points = np.zeros((samples, 3)) # x, y, orientation respect to center
    sector = ['A']*samples

    # evaluate the points
    t = 0
    x = 0
    y = 0
    theta = 0
    for i in range(0, samples):
        if t <= t_a:
            x = t * 2 * d / t_a
            y = 0
            theta = 0
            sector[i] = 'A'
        elif t_a < t <= t_b:
            x = 2 * d + (d / 2.) * np.cos(-np.pi * (t - t_a) / (t_b - t_a) + np.pi / 2)
            y = -d / 2. + (d / 2.) * np.sin(-np.pi * (t - t_a) / (t_b - t_a) + np.pi / 2)
            theta = -np.pi * (t - t_a) / (t_b - t_a) #+ np.pi / 2
            sector[i] = 'B'
        elif t_b < t <= t_c:
            x = 2 * d - (t - t_b) * 2 * d / (t_c - t_b)
            y = -d
            theta = 0
            sector[i] = 'C'
        elif t_c < t <= t_d:
            x = (d / 2.) * np.cos(+np.pi * (1 - ((t - t_c) / (t_d - t_c))) + np.pi / 2)
            y = -d / 2. + (d / 2.) * np.sin(+np.pi * (1 - ((t - t_c) / (t_d - t_c))) + np.pi / 2)
            theta = np.pi*(1 - ((t - t_c) / (t_d - t_c))) #+ np.pi / 2
            sector[i] = 'D'

        points[i, 0] = x
        points[i, 1] = y
        points[i, 2] = theta
        t += t_interval
        # print("t = {}, point = {}".format(t, points[i, :]))
    return points, sector

#
# test code for the function
#
# d = 10
# samples = 100
# array = parametric_track(d, samples)
# # plot
# plt.plot(array[:, 0], array[:, 1], 'ro')
# plt.show()
