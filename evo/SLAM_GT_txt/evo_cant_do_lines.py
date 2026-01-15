import numpy as np
import matplotlib.pyplot as plt

def load_tum_xy(filename):
    """
    Load a TUM trajectory and return x, y arrays.
    TUM format:
    timestamp tx ty tz qx qy qz qw
    """
    data = np.loadtxt(filename)
    x = data[:, 1]
    y = data[:, 2]
    return x, y


# ---- load aligned trajectories ----
x_gt,    y_gt    = load_tum_xy("ground_truth.tum")
x_liloc, y_liloc = load_tum_xy("liloc_estimate.tum")
x_fast,  y_fast  = load_tum_xy("FAST_LIO.tum")
x_lio,   y_lio   = load_tum_xy("LIO-SAM.tum")
x_lvi,   y_lvi   = load_tum_xy("LVI-SAM.tum")


# ---- plot with different line styles ----
plt.figure(figsize=(8, 8))

plt.plot(x_gt,   y_gt,   label='Ground Truth')  # solid

plt.plot(x_liloc, y_liloc, '--', marker='.', label='LiLoc')    # dashed + markers
plt.plot(x_fast,  y_fast,  '--', label='FAST-LIO')    # dashed
plt.plot(x_lio,   y_lio,   ':',  label='LIO-SAM')     # dotted
plt.plot(x_lvi,   y_lvi,   '-.', label='LVI-SAM')     # dash-dot

plt.axis('equal')
plt.xlabel("x [m]")
plt.ylabel("y [m]")
plt.legend()
plt.grid(True)

plt.savefig("map.png", dpi=300, bbox_inches="tight")
plt.show()
