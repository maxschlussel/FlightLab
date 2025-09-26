import math
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

from matplotlib.animation import FuncAnimation


def show_rot(csv_file: str = 'output/data_log.csv'):
    """
    Creates a bisic wireframe 3D animated visualization of an aircraft's attitude.
    """

    df = pd.read_csv(csv_file, skipinitialspace=True)

    time  = df["time"]
    phi   = df["X_phi"]   * 180/math.pi
    theta = df["X_theta"] * 180/math.pi
    psi   = df["X_psi"]   * 180/math.pi

    # Body axes in body frame
    body_axes = np.eye(3)

    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")
    ax.set_xlim([-1, 1])
    ax.set_ylim([-1, 1])
    ax.set_zlim([1, -1])
    ax.set_xlabel("East")
    ax.set_ylabel("North")
    ax.set_zlabel("Down")

    lines = [ax.plot([0,0],[0,0],[0,0])[0] for _ in range(3)]
    colors = ["r","g","b"]

    def update(i):
        rot = R.from_euler("xyz", [phi[i], theta[i], psi[i]], degrees=True)
        rotated_axes = rot.apply(body_axes)

        for j in range(3):
            lines[j].set_data([0, rotated_axes[j,0]], [0, rotated_axes[j,1]])
            lines[j].set_3d_properties([0, rotated_axes[j,2]])
            lines[j].set_color(colors[j])

        return lines

    plot_every = 5  # plot every 5 timesteps
    ani = FuncAnimation(fig, update, frames=range(0, len(time), plot_every), interval=plot_every*10)

    plt.show()


if __name__ == "__main__":
    show_rot()