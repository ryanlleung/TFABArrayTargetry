
# Import libraries
import numpy as np
from scipy.spatial.transform import Rotation
from scipy.optimize import fsolve

## All angles input in degrees ##

# Relative transformations in X-Y-Z-Rx-Ry-Rz
P = np.array([[0,54.43,0,0,0,0],    # ground to X-top
            [0,62.43,0,0,0,0],      # X-top to Z-top (including 8mm offset from adapter plate)
            [-217.43,281.43,0,0,0,0], # Z-top to Y-base
            [54.43,0,0,0,0,0],      # Y-base to Y-top
            [306,90,0,0,0,0],       # Y-top to ball
            [-163,198,0,0,0,0]])    # ball to target centroid

# Function to return the geometry
def get_geometry():
    return P

# Define transformation matrix (6 DoF)
def get_T(pose):
    if len(pose) != 6: raise ValueError('Pose must be 6 elements long')
    X, Y, Z, Rx, Ry, Rz = pose
    Rx, Ry, Rz = np.deg2rad([Rx, Ry, Rz])
    R = Rotation.from_euler('xyz', [Rx, Ry, Rz])
    T = np.eye(4)
    T[:3,:3] = R.as_matrix()
    T[:3,3] = [X, Y, Z]
    return T

# Get transformation matrix from ground to points 1-6 from stage positions (5DoF)
def get_transformation(stagePos):

    X, Y, Z, Rx, Ry = stagePos

    T01 = get_T([P[0][0]+X, P[0][1], P[0][2], 0, 0, 0])
    T12 = get_T([P[1][0], P[1][1], P[1][2]+Z, 0, 0, 0])
    T23 = get_T([P[2][0], P[2][1], P[2][2], 0, 0, 0])
    T34 = get_T([P[3][0], P[3][1]+Y, P[3][2], 0, 0, 0])
    T45 = get_T([P[4][0], P[4][1], P[4][2], Rx, Ry, 0])
    T56 = get_T([P[5][0], P[5][1], P[5][2], 0, 0, 0])

    T02 = np.dot(T01, T12)
    T03 = np.dot(T02, T23)
    T04 = np.dot(T03, T34)
    T05 = np.dot(T04, T45)
    T06 = np.dot(T05, T56)

    return T01, T02, T03, T04, T05, T06

# Function to calculate transformation from ground to target point
# poseTP (5DoF) is target point pose, x-y coordinates and z, Rx, Ry offsets
def get_transformation_TP(stagePos, poseTP):

    T06 = get_transformation(stagePos)[-1]
    T6TP = get_T(np.append(poseTP, 0)) # append 0 to poseTP to make it 6 elements long
    T0TP = np.dot(T06, T6TP)

    return T0TP

# Function to generate target plane surface
def get_target_plane(stagePos, width=190, height=290, density=2):

    Tx = np.linspace(-0.5*width, 0.5*width, density)
    Ty = np.linspace(-0.5*height, 0.5*height, density)
    TS = np.zeros((density, density, 3))

    for i in range(0, density):
        for j in range(0, density):
            # Abuse get_transformation_TP to get the plane mesh coordinates
            T0TP = get_transformation_TP(stagePos, [Tx[i], Ty[j], 0, 0, 0])
            for k in range(0, 3):
                # Extract the coordinates
                TS[i][j][k] = T0TP[k][3]

    return TS

# Function to use inverse kinematics to calculate the stage positions
# poseSetpoint (5DoF) = target point coordinates in the space frame
# poseTP (5DoF) = target point pose in the target plane frame (x-y coordinates and z, Rx, Ry offsets)
def get_stage_positions(poseSetpoint, poseTP):

    if len(poseSetpoint) != 5: raise ValueError('Setpoint must be 5 elements long')
    if len(poseTP) != 5: raise ValueError('Target point pose must be 5 elements long')

    # Function to calculate the error between the current position and the setpoint
    # q = stage positions guess
    def error_function(q, TSx=0, TSy=0, TSz=0, TSrx=0, TSry=0, poseTP=[0,0,0,0,0,0]):
        T_guess = get_transformation_TP(q, poseTP)
        
        # Get angles from T_guess
        R_guess = Rotation.from_matrix(T_guess[0:3,0:3])
        angles_guess = R_guess.as_euler('xyz', degrees=True)
        
        # Calculate the error
        error = np.array([T_guess[0,3]-TSx,
                          T_guess[1,3]-TSy,
                          T_guess[2,3]-TSz,
                          angles_guess[0]-TSrx,
                          angles_guess[1]-TSry])
        return error

    TSx, TSy, TSz, TSrx, TSry = poseSetpoint

    q_solution = fsolve(error_function, [0,0,0,0,0],
                        args=(TSx, TSy, TSz, TSrx, TSry, poseTP))

    return q_solution

# Example
if __name__ == '__main__':
    print(get_stage_positions([0,450,0,0,0], [0,0,0,0,0]))
