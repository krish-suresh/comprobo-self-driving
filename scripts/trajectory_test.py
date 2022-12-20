from curves import CubicSpline2D, RaceTrack
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import FancyArrow, Rectangle

track = RaceTrack("Budapest_raceline.csv", 13, 9)
x = []
y = []
sp = track.create_spline()
s = np.linspace(0,sp.s[-1], 500)
L = 0.29
phi = []
print(sp.calc_position(0))
print(sp.calc_yaw(0))
for si in s[:-1]:

    curr_x, curr_y = sp.calc_position(si)
    x.append(curr_x)
    y.append(curr_y)
    phi.append(np.arctan(L*sp.calc_curvature(si)))
phi.sort()
print(np.degrees(phi[-20:]))
fig = plt.figure()
ax = fig.add_subplot(111)
ax.plot(x, y)
ax.add_patch(Rectangle((-2, -0.5), 8,6.5, fc='none', ec='k', lw=1, rotation_point="center", angle = 45))
ax.axis('equal')
plt.show()