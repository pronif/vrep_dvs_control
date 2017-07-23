import numpy as np


def is_passed(position, orientation, target_point, sector, d):
    if sector == 'A':
        if position[0] > target_point[0]:
            return True
    elif sector == 'B':
        if position[1] < np.tan(target_point[2]+np.pi/2)*(position[0]-2*d)-d/2:
            return True
    elif sector == 'C':
        if float(position[0]) < float(target_point[0]):
            return True
    elif sector == 'D':
        if position[1] > np.tan(target_point[2] + np.pi / 2) * (position[0]) - d / 2:
            return True

    return False


# np.pi / 2 - orientation[2] < target_point[2]:
