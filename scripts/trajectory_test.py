from curves import CubicSpline2D
import matplotlib.pyplot as plt
import numpy as np

x = np.array(
    [
        0,
        1,
        1 + 2**0.5 / 2,
        1 + 2**0.5 / 2,
        1,
        0,
        -1,
        -1 - 2**0.5 / 2,
        -1 - 2**0.5 / 2,
        -1,
        0,
    ]
)
y = np.array(
    [
        0,
        0,
        1 - 2**0.5 / 2,
        1 + 2**0.5 / 2,
        2,
        2,
        2,
        1 + 2**0.5 / 2,
        1 - 2**0.5 / 2,
        0,
        0,
    ]
)

sp = CubicSpline2D(x, y)

x = []
y = []
s = np.linspace(0, sp.s[-1], 500)
for si in s[:-1]:
    print(si)
    curr_x, curr_y = sp.calc_position(si)
    x.append(curr_x)
    y.append(curr_y)

plt.plot(x, y)
plt.axis("equal")
plt.show()
