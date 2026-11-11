import pandas as pd, numpy as np, matplotlib.pyplot as plt

# d = pd.read_csv("traj.csv")
# dt = 0.001
#
# for c in ["q1", "q2"]:
#     d[c + "_vel"] = np.gradient(d[c], dt)
#     d[c + "_acc"] = np.gradient(d[c + "_vel"], dt)
#
# d.plot(x="t", y=["x_ref", "z_ref"])
# d.plot(x="t", y=["q1_acc", "q2_acc"])
# d.plot(x="x_ref", y="z_ref")
# d.plot(x="t", y="err")

for name in ("trot", "walk"):
    d = pd.read_csv(f"gait_{name}.csv")
    fig, ax = plt.subplots(2, 1, sharex=True)
    for i, leg in enumerate(("LF", "RF", "LB", "RB")):
        ax[0].plot(d.t, d[f"x{i}"], label=leg)
        ax[1].plot(d.t, d[f"z{i}"], label=leg)
    ax[0].set_ylabel("x [m]");
    ax[1].set_ylabel("z [m]");
    ax[0].set_xlabel("t [s]");
    ax[0].legend()
    fig.suptitle(name)
plt.show()
