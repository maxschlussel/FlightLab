
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


phi   = df["phi"]
theta = df["theta"]
psi   = df["psi"]
time = df["time"]


import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from scipy.spatial.transform import Rotation as R


n = len(phi)

# Body axes in body frame
body_axes = np.eye(3)

fig = plt.figure()
ax = fig.add_subplot(111, projection="3d")
ax.set_xlim([-1, 1])
ax.set_ylim([-1, 1])
ax.set_zlim([-1, 1])
ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z (down)")

lines = [ax.plot([0,0],[0,0],[0,0])[0] for _ in range(3)]
colors = ["r","g","b"]

def update(i):
    rot = R.from_euler("xyz", [phi[i], theta[i], psi[i]], degrees=True)
    rotated_axes = rot.apply(body_axes)

    for j in range(3):
        lines[j].set_data([0, rotated_axes[j,0]], [0, rotated_axes[j,1]])
        lines[j].set_3d_properties([0, -rotated_axes[j,2]])  # flip Z

        lines[j].set_color(colors[j])

    return lines

plot_every = 10  # plot every 5 timesteps
ani = FuncAnimation(fig, update, frames=range(0, len(time), plot_every), interval=plot_every*10)
plt.show()
