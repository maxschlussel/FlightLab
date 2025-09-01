import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv("output/data_log.csv")

fig, axs = plt.subplots(5, 3, figsize=(12, 8))
axs = axs.flatten()

state_names = ["u", "v", "w", "p", "q", "r", "phi", "theta", "psi", "x", "y", "z"]

for i, state in enumerate(state_names):
    axs[i].plot(df["time"], df[state])
    axs[i].set_title(state)
    axs[i].set_xlabel("Time [s]")
    axs[i].grid(True)

axs[12].plot(df["x"], df["y"])
axs[12].set_title("x vs y")
axs[12].set_xlabel("x")
axs[12].set_ylabel("y")
axs[12].grid(True)

plt.tight_layout()
plt.show()




import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from scipy.spatial.transform import Rotation as R


# Convert Euler angles to rotation matrices (ZYX convention: psi, theta, phi)
rotations = R.from_euler('xyz', np.column_stack([df.phi, df.theta, df.psi])).as_matrix()

# Axes to plot: unit vectors in body frame
body_axes = np.eye(3)  # x, y, z

# Setup plot
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.set_xlim([-1,1])
ax.set_ylim([1,-1])
ax.set_zlim([1,-1])
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

# Create quivers for axes: red=x, green=y, blue=z
quivers = [ax.quiver(0,0,0,1,0,0,color='r'),
           ax.quiver(0,0,0,0,1,0,color='g'),
           ax.quiver(0,0,0,0,0,1,color='b')]

def update(i):
    R_i = rotations[i]
    for j, q in enumerate(quivers):
        vec = R_i @ body_axes[:,j]
        # Remove old quiver and redraw new one
        q.remove()
        quivers[j] = ax.quiver(0,0,0,vec[0],vec[1],vec[2],color=['r','g','b'][j])
    return quivers

ani = FuncAnimation(fig, update, frames=len(df.time), interval=10)
plt.show()
