import numpy as np
import math
import vrep
import sys
import track as trk
import control
import evaluation

#
#
#
# TRACK
#
#
#

# service variables
DEBUG = False
SAVE_TEXT = True

# establish a connection
vrep.simxFinish(-1)  # just in case, close all opened connections
clientID = vrep.simxStart('127.0.0.1', 19999, True, True, 5000, 5)  # Connect to V-REP

if clientID != -1:
    print('Connected to remote API server')
else:
    print('Connection not successful')
    sys.exit('Could not connect')

# synchronous start
returnCode = vrep.simxSynchronous(clientID, True)
vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot)

# obtain the parametric track
d_tr = 5.0  # distance: the only parameter of the track
samples = 50  # number of samples required
points, sector = trk.parametric_track(d_tr, samples)
if DEBUG:
    print(points)
    print(sector)
if SAVE_TEXT:
    np.savetxt('points.csv', points, fmt='%.5f,%.5f,%.5f', delimiter=',', header='Points: x,y,theta', comments='')

# get the handle of the basis rectangle
basisRec = vrep.simxGetObjectHandle(clientID, 'basis_rec', vrep.simx_opmode_blocking)[1]

# get the handles of the curve centers
centerB = vrep.simxGetObjectHandle(clientID, 'center_b', vrep.simx_opmode_blocking)[1]
centerD = vrep.simxGetObjectHandle(clientID, 'center_d', vrep.simx_opmode_blocking)[1]
# set the position of the curve centers
B_pos = [2 * d_tr, -d_tr / 2, 0.5]
D_pos = [0, -d_tr / 2, 0.5]
vrep.simxSetObjectPosition(clientID, centerB, -1, B_pos, vrep.simx_opmode_blocking)
vrep.simxSetObjectPosition(clientID, centerD, -1, D_pos, vrep.simx_opmode_blocking)

# loop over the elements of points
for i in range(len(points)):
    # copy the basis rectangle
    copyRec = vrep.simxCopyPasteObjects(clientID, [basisRec], vrep.simx_opmode_blocking)[1]
    # set the position of the copied rectangle
    nextPoint = [points[i, 0], points[i, 1], 0.0005]
    vrep.simxSetObjectPosition(clientID, copyRec[0], -1, nextPoint, vrep.simx_opmode_blocking)
    # if we are in a curve, set the orientation
    if sector[i] == 'B':
        res = vrep.simxSetObjectOrientation(clientID, copyRec[0], centerB, [0, 0, points[i, 2]],
                                            vrep.simx_opmode_blocking)
    elif sector[i] == 'D':
        res = vrep.simxSetObjectOrientation(clientID, copyRec[0], centerD, [0, 0, points[i, 2]],
                                            vrep.simx_opmode_blocking)

#
#
#
# CONTROL
#
#
#

# Get the handles of the car motors and steering
kart_body = vrep.simxGetObjectHandle(clientID, 'kart_basic', vrep.simx_opmode_blocking)[1]
steeringLeft = vrep.simxGetObjectHandle(clientID, 'nakedCar_steeringLeft', vrep.simx_opmode_blocking)[1]
steeringRight = vrep.simxGetObjectHandle(clientID, 'nakedCar_steeringRight', vrep.simx_opmode_blocking)[1]
motorLeft = vrep.simxGetObjectHandle(clientID, 'nakedCar_motorLeft', vrep.simx_opmode_blocking)[1]
motorRight = vrep.simxGetObjectHandle(clientID, 'nakedCar_motorRight', vrep.simx_opmode_blocking)[1]

# Car parameters
desiredSteeringAngle = 0
d = 0.0755  # 2*d=distance between left and right wheels
l = 0.25772  # l=distance between front and rear wheels

# Set the velocity to a constant value
desiredWheelRotSpeed = 180 * math.pi / 180
desiredWheelRotSpeed *= 6
vrep.simxSetJointTargetVelocity(clientID, motorLeft, desiredWheelRotSpeed, vrep.simx_opmode_blocking)
vrep.simxSetJointTargetVelocity(clientID, motorRight, desiredWheelRotSpeed, vrep.simx_opmode_blocking)

# Initialize the DVS and the wheel angle
vrep.simxReadStringStream(clientID, "dvsData", vrep.simx_opmode_streaming)
vrep.simxSetJointTargetPosition(clientID, steeringLeft, 0, vrep.simx_opmode_streaming)
vrep.simxSetJointTargetPosition(clientID, steeringRight, 0, vrep.simx_opmode_streaming)

# initialize the variable for the evaluation metrics
pass_point = np.zeros((samples, 2))
pp_index = 1
target_point = [points[1, 0], points[1, 1], 0]
current_sector = sector[pp_index]

total_events = 0

for i in range(100000):

    # triggers next step
    vrep.simxSynchronousTrigger(clientID)
    # ping is a blocking function. wait until end of step calculations
    vrep.simxGetPingTime(clientID)

    # read the string of DVS events
    err, signal = vrep.simxReadStringStream(clientID, "dvsData", vrep.simx_opmode_buffer)
    # convert the string to readable integers
    # event is a list of n*4 elements.
    # Each one of the n events is described by: simulationtime*1000, posx, posy and +/- 1
    event = vrep.simxUnpackInts(signal)
    print("string {}:".format(i))
    total_events += len(event)
    avg = total_events/(i+1)
    print("total events: {}, avg: {}".format(total_events, avg))

    # if there are no events, do nothing
    if not event:
        print("string {} is empty".format(i))
        continue

    # obtain control signal
    beforeAngle = desiredSteeringAngle
    desiredSteeringAngle = control.control_moments(event, beforeAngle)

    # Apply control input coherent to Ackermann steering
    if desiredSteeringAngle != 0:
        steeringAngleLeft = math.atan(l / (-d + l / math.tan(desiredSteeringAngle)))
        steeringAngleRight = math.atan(l / (d + l / math.tan(desiredSteeringAngle)))
        vrep.simxSetJointTargetPosition(clientID, steeringLeft, steeringAngleLeft, vrep.simx_opmode_blocking)
        vrep.simxSetJointTargetPosition(clientID, steeringRight, steeringAngleRight, vrep.simx_opmode_blocking)

    # evaluation
    pos = vrep.simxGetObjectPosition(clientID, kart_body, -1, vrep.simx_opmode_blocking)[1]
    if current_sector == 'B':
        orient = vrep.simxGetObjectOrientation(clientID, kart_body, centerB, vrep.simx_opmode_blocking)[1]
    else:
        orient = vrep.simxGetObjectOrientation(clientID, kart_body, centerB, vrep.simx_opmode_blocking)[1]
    if DEBUG:
        print("position: {} orientation:{} sector: {}".format(pos, orient, current_sector))
    if evaluation.is_passed(pos, orient, target_point, current_sector, d_tr):
        pass_point[pp_index] = [pos[0], pos[1]]
        pp_index += 1
        if pp_index >= samples:
            if SAVE_TEXT:
                np.savetxt('pass_point.csv', pass_point, fmt='%.5f,%.5f', delimiter=',', header='Points: x,y',
                           comments='')
            sys.exit('One lap terminated')
        target_point = points[pp_index]
        current_sector = sector[pp_index]
        print("position: {} orientation:{}".format(pos, orient))
        print("next_sector: {} next_target {}".format(current_sector, target_point))
        print(pass_point)
        if SAVE_TEXT:
            np.savetxt('pass_point.csv', pass_point, fmt='%.5f,%.5f', delimiter=',', header='Points: x,y', comments='')

    print

# stop the simulation until enter is pressed
vrep.simxSynchronousTrigger(clientID)
vrep.simxGetPingTime(clientID)
# raw_input()
