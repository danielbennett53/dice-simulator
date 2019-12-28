import numpy as np
import Geometry as geo
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import random

points = []
for i in range(10):
    points.append(np.array([random.uniform(-1.0, 1.0), random.uniform(-1.0,1.0)], dtype=np.float64))

out = geo.convexHull2D(points)
xOut = np.vstack([a for a in out])[:,0]
yOut = np.vstack([a for a in out])[:,1]

agg = np.vstack([a for a in points])
x = agg[:,0]
y = agg[:,1]

fig = plt.figure()
ax = fig.add_subplot(311)
ax.fill(xOut, yOut, 'g')
ax.plot(x, y, 'ro')

normal = np.array([random.uniform(-1.0, 1.0), random.uniform(-1.0,1.0), random.uniform(-1.0,1.0)])
normal = normal / np.linalg.norm(normal)

ax2 = fig.add_subplot(312, projection='3d')
ax2.plot([0, 4*normal[0]], [0, 4*normal[1]], zs=[0, 4*normal[2]])
points = []
for i in range(10):
    x = random.uniform(-5.0, 5.0)
    y = random.uniform(-5.0, 5.0)
    z = (-normal[0]*x - normal[1]*y) / normal[2]
    points.append(np.array([x, y, z]))
    ax2.scatter(x, y, z, c='r')


out = geo.planeProjection(points, normal)

for o in out:
    ax2.scatter(o[0], o[1], o[2], c='b')
ax2.set_xlabel('X')
ax2.set_ylabel('Y')
ax2.set_zlabel('Z')

## Triangle intersection
tri1 = [
    np.array([-1, 0, 0]),
    np.array([1, 0, 0]),
    np.array([0, 1, 0])
]

tri2 = [
    np.array([0, 0.5, -1]),
    np.array([0, 0.5, 1]),
    np.array([0.2, 1.2, 0])
]

out = np.array([], dtype=np.float64)
cop = 0

print(geo.triTriIntersection3d(tri1, tri2, out, cop))
print(out)

ax3 = fig.add_subplot(313, projection='3d')
ax3.add_collection3d(Poly3DCollection([tri1], facecolors="r"))
ax3.add_collection3d(Poly3DCollection([tri2], facecolors="b"))
ax3.set_xlabel('X')
ax3.set_ylabel('Y')
ax3.set_zlabel('Z')
plt.show()