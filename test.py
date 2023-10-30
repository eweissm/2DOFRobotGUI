import numpy as np
def NormalizeAngle(angle):
    if angle > 2*np.pi:
        solution = angle - abs(np.floor(angle / (2*np.pi)) * 2 * np.pi)
        print("angle > 2pi")
    elif angle < 0:
        solution = angle + abs(np.floor(angle / (2*np.pi)) * 2 * np.pi)
        print("angle < 2pi")
    else:
        solution = angle
        print("angle did not need to be normalized")

    return solution

angle_deg = 1500
angle = angle_deg * np.pi / 180
print(NormalizeAngle(angle) * 180/ np.pi)
