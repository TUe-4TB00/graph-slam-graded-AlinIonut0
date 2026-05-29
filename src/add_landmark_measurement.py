import math
import numpy as np
import gtsam
from gtsam.symbol_shorthand import L, X

PRIOR_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.1, 0.1, 0.05]))  # (x, y, theta)
ODOMETRY_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.2, 0.2, 0.1]))  # (dx, dy, dtheta)
MEASUREMENT_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.05, 0.1]))  # (bearing, range)

def add_landmark_measurement(graph, initial_estimate, result):
    # Determine the correct rotation (bearing) and distance from X(4) to L(2) 

    pose_x4 = result.atPose2(X(4))
    landmark_l2 = result.atPoint2(L(2))


    rotation = np.degrees(np.arctan2(landmark_l2[1] - pose_x4.y(), landmark_l2[0] - pose_x4.x()) - pose_x4.theta())
    distance = np.sqrt((landmark_l2[0] - pose_x4.x())**2 + (landmark_l2[1] - pose_x4.y())**2)

    print(rotation)
    print(distance)

    graph.add(gtsam.BearingRangeFactor2D(X(4), L(2), gtsam.Rot2.fromDegrees(rotation), distance, MEASUREMENT_NOISE))
    return graph