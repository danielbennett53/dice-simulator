import numpy as np
import signal
import sys
import Geometry as geo
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import random


def signal_handler(sig, frame):
    sys.exit(0)


signal.signal(signal.SIGINT, signal_handler)

points = []
for i in range(10):
    points.append(np.array([random.uniform(-1.0, 1.0), random.uniform(-1.0,1.0)], dtype=np.float64))

# points = [
#     np.array([0, 0], dtype=np.float64),
#     np.array([0, 1], dtype=np.float64),
#     np.array([.1, 0.5], dtype=np.float64),
#     np.array([1, 1], dtype=np.float64),
# ]

out = geo.convexHull2d(points)
xOut = np.vstack([a for a in out])[:,0]
yOut = np.vstack([a for a in out])[:,1]

agg = np.vstack([a for a in points])
x = agg[:,0]
y = agg[:,1]

fig1 = plt.figure()
ax = fig1.add_subplot(111)
ax.fill(xOut, yOut, 'g')
ax.plot(x, y, 'ro')

print("Area = {}".format(geo.polygonArea(points)))

normal = np.array([random.uniform(-1.0, 1.0), random.uniform(-1.0,1.0), random.uniform(-1.0,1.0)])
normal = normal / np.linalg.norm(normal)

# fig2 = plt.figure()
# ax2 = fig2.add_subplot(111, projection='3d')
# ax2.plot([0, 4*normal[0]], [0, 4*normal[1]], zs=[0, 4*normal[2]])
points = []
for i in range(10):
    x = random.uniform(-5.0, 5.0)
    y = random.uniform(-5.0, 5.0)
    z = (-normal[0]*x - normal[1]*y) / normal[2]
    points.append(np.array([x, y, z]))
    # ax2.scatter(x, y, z, c='r')


out = geo.planeProjection(points, normal)

# for o in out:
#     ax2.scatter(o[0], o[1], o[2], c='b')
# ax2.set_xlabel('X')
# ax2.set_ylabel('Y')
# ax2.set_zlabel('Z')

## Triangle intersection
# while True:
#     tri1 = []
#     tri2 = []
#     for i in range(3):
#         tri1.append(np.array([random.uniform(-1.0, 1.0), random.uniform(-1.0, 1.0), random.uniform(-1.0, 1.0)]))
#         tri2.append(np.array([random.uniform(-1.0, 1.0), random.uniform(-1.0, 1.0), random.uniform(-1.0, 1.0)]))

#     # tri1 = [
#     #     np.array([-1, 0, 0]),
#     #     np.array([1, 0, 0]),
#     #     np.array([0, 1, 0])
#     # ]

#     # tri2 = [
#     #     np.array([0, 0.5, -1]),
#     #     np.array([0, 0.5, 1]),
#     #     np.array([0.2, 1.2, 0])
#     # ]

#     isect = geo.triTriIntersection3d(tri1, tri2)
#     tri1.append(tri1[0])
#     tri2.append(tri2[0])

#     tri1X = np.vstack([a for a in tri1])[:,0]
#     tri1Y = np.vstack([a for a in tri1])[:,1]
#     tri1Z = np.vstack([a for a in tri1])[:,2]

#     tri2X = np.vstack([a for a in tri2])[:,0]
#     tri2Y = np.vstack([a for a in tri2])[:,1]
#     tri2Z = np.vstack([a for a in tri2])[:,2]


#     fig3 = plt.figure()
#     ax3 = fig3.add_subplot(111, projection='3d')
#     ax3.plot(tri1X, tri1Y, zs=tri1Z, c='b')
#     ax3.plot(tri2X, tri2Y, zs=tri2Z, c='g')

#     if len(isect) != 0:
#         isectX = np.vstack([a for a in isect])[:,0]
#         isectY = np.vstack([a for a in isect])[:,1]
#         isectZ = np.vstack([a for a in isect])[:,2]
#         ax3.plot(isectX, isectY, zs=isectZ, c='r')

#     ax3.set_xlabel('X')
#     ax3.set_ylabel('Y')
#     ax3.set_zlabel('Z')
plt.show()