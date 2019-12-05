import numpy as np

def closestCircPoint(center, radius, point):
    # x,y = (point-center)
    # theta = np.arctan2(y,x)
    # if(np.linalg.norm(np.array([radius*np.cos(theta), radius * np.sin(theta)])) > np.linalg.norm(np.array([radius * np.cos(theta + 3.1415), radius * np.sin(theta + 3.1415)]))):
    #     theta = theta+3.1415
    # closest = np.array([radius * np.cos(theta), radius * np.sin(theta)])
    dis = radius - np.linalg.norm(point-center)
    return dis