
import math
import numpy as np
import gtsam
from gtsam.symbol_shorthand import L, X

PRIOR_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.1, 0.1, 0.05]))  # (x, y, theta)
ODOMETRY_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.2, 0.2, 0.1]))  # (dx, dy, dtheta)
MEASUREMENT_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.05, 0.1]))  # (bearing, range)

def add_pose(graph, initial_estimate):

    odo = gtsam.Pose2(2.0 * np.cos(np.pi / 4), 2.0 * np.sin(np.pi / 4), np.pi / 2.0)

    # TODO: Add the odometry factor between X(4) and X(5) to the graph (BetweenFactorPose2)
    graph.add(gtsam.BetweenFactorPose2(X(3), X(4), odo, ODOMETRY_NOISE))

    pose3 = initial_estimate.atPose2(X(3))

    initial_estimate.insert(X(4), gtsam.Pose2(pose3.x() + np.round(odo.x(), 1), pose3.y() + np.round(odo.y(), 1), pose3.theta() + np.floor(odo.theta() * 10) / 10))

    # TODO: Based on the odometry, find the initial estimate for the pose of X(5) and add it to the graph
    
    return graph, initial_estimate