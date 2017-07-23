import math


# accessory functions

def find_horizon(super_event):
    # find horizon and range of y values
    y_min = 128
    y_max = 0
    y_freq = [0] * 128
    for j in range(len(super_event)):
        if (j % 4) == 2:
            y_freq[super_event[j]] += 1
            if super_event[j] > y_max:
                y_max = super_event[j]
            if super_event[j] < y_min:
                y_min = super_event[j]

    hor = y_freq.index(max(y_freq))
    # print("horizon: {} min y: {} max y: {}".format(hor, y_min, y_max))
    return hor, y_min, y_max


# main control functions

# naive control algorithm
def control_naive(super_event, previous_angle):
    steeringAngleDx = 2 * math.pi / 180

    hor, y_min, y_max = find_horizon(super_event)

    # find pixels on right and left sides
    lhs = 0
    rhs = 0

    dist = float(y_max - y_min)

    for j in range(len(super_event)):
        if (j % 4) == 1 and super_event[j] > 64:
            rhs += 1
        elif (j % 4) == 1 and super_event[j] < 64:
            lhs += 1
    #print(len(super_event))

    # control algorithm
    diff = rhs - lhs
    z = float(len(super_event) / 4.0)
    PARAM = diff / z
    print("lhs = {}, rhs = {}, diff = {}".format(lhs, rhs, diff))

    desired_angle = 0.275*PARAM
    #k = 0.001
    #desired_angle = k * diff
    if rhs > 1000:
        desired_angle = previous_angle

    # control that physical limits are respected
    if desired_angle < -45 * math.pi / 180:
        desired_angle = -45 * math.pi / 180
    if desired_angle > 45 * math.pi / 180:
        desired_angle = 45 * math.pi / 180

    print("desired angle: {}".format(desired_angle * 180 / math.pi))
    print("regularized control param diff/n_events: {}".format(PARAM))

    return desired_angle


# moments control algorithm (Chaumette)
def control_moments(super_event, previous_angle):
    # find pixels on right and left sides
    g = [0, 0]

    for j in range(len(super_event)):
        if (j % 4) == 1:
            g[0] += super_event[j]
        elif (j % 4) == 2:
            g[1] += super_event[j]
    print(len(super_event))

    z = float(len(super_event)/4)
    G = [x / z for x in g]
    # print ("g: {}, G: {}".format(g, G))

    # control algorithm

    k = 0.008  # probably needs to be higher
    desired_angle = k * (G[0]-64)
    if z>5000:
        desired_angle = previous_angle

    # control that physical limits are respected
    if desired_angle < -45 * math.pi / 180:
        desired_angle = -45 * math.pi / 180
    if desired_angle > 45 * math.pi / 180:
        desired_angle = 45 * math.pi / 180

    # print("desired angle: {}".format(desired_angle * 180 / math.pi))

    return desired_angle
