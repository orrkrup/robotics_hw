import numpy as np
import matplotlib.pyplot as plt

alpha = np.radians(30)
point = np.array((np.cos(alpha), np.sin(alpha), 1))

angle = 98.42
theta = np.radians(angle)

r = [-1.0 - np.sqrt(2.0) / 2.0, np.sqrt(2.0) / 2.0, -np.sqrt(2.0) / 2.0] / (2 * np.sin(theta))

def rotate(x, t):
    c = np.cos(t)
    s = np.sin(t)

    rotation = np.array([
        [r[0] * r[0] * (1 - c) + c, r[0] * r[1] *(1 - c) + r[2] * s, r[0] * r[2] * (1-c) + r[1] * s],
        [r[0] * r[1] * (1 - c) + r[2] * s, r[1] * r[1] * (1 - c) + c, r[1] * r[2] * (1 - c) + r[0] * s],
        [r[0] * r[2] * (1 - c) + r[1] * s, r[1] * r[2] * (1 - c) + r[0] * s, r[2] * r[2] * (1 - c) + s]
    ])
    
    return np.dot(rotation, x)

xyz = []
for t in np.linspace(0.0, theta, num=100):
    xyz.append(rotate(point, t))

points = np.stack(xyz)

fig = plt.figure()
ax = fig.add_subplot(projection='3d')
plt.scatter(points[:, 0], points[:, 1], points[:, 2])