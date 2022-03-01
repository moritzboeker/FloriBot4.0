from matplotlib.patches import Patch
import matplotlib.pyplot as plt
import numpy as np

fig, ax = plt.subplots()

X = np.arange(-10, 10, 1)
Y = np.arange(-10, 10, 1)
U, V = np.meshgrid(X, Y)

q = ax.quiver(X, Y, U, V)

qk = ax.quiverkey(q, 0.9,0.8, U=10, label='QK length = 10', labelpos='E')

legend_elements = [
    Patch(facecolor="red", edgecolor="black", label="First Element"),
    qk
]

ax.legend(handles=legend_elements)

plt.show()