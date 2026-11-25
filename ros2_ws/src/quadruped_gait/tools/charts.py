import pandas as pd, numpy as np, matplotlib.pyplot as plt

LEGS = ("LF", "RF", "LB", "RB")

def plot_traj(path="traj.csv"):
    d = pd.read_csv(path)
    for c in ("q1", "q2"):
        d[c + "_vel"] = np.gradient(d[c], dt)
        d[c + "_acc"] = np.gradient(d[c + "_vel"], dt)

    d.plot(x="t", y=["q1", "q2"], ylabel="angle [rad]")
    d.plot(x="t", y=["q1_vel", "q2_vel"], ylabel="velocity [rad/s]")
    d.plot(x="t", y=["q1_acc", "q2_acc"], ylabel="acceleration [rad/s^2]")
    d.plot(x="y_fk", y="z_fk", title="foot path x-y")
    d.plot(x="x_fk", y="z_fk", title="foot path x-z")

def plot_gait(name):
    d = pd.read_csv(f"gait_{name}.csv")
    cmd = d[["v_cmd", "w_cmd"]]
    edges = d.t[(cmd.diff().abs().sum(axis=1) > 0)]

    fig, ax = plt.subplots(3, 1, sharex=True, figsize=(10, 8))
    for i, leg in enumerate(LEGS):
        ax[0].plot(d.t, d[f"s_{i}"], label=leg)
        ax[1].plot(d.t, np.degress(d[f"q1_{i}"]), label=leg)
        ax[2].plot(d.t, np.degress(d[f"q2_{i}"]), label=leg)

    ax[0].set_ylabel("s [-]")
    ax[1].set_ylabel("q1 [deg]")
    ax[2].set_ylabel("q2 [deg]")
    ax[2].set_xlabel("t [s]")
    ax[0].legend(ncol=4)

    for a in ax:
        a.grid(alpha=0.3)
        for te in edges:
            a.axvline(te, color="k", lw=0.5, ls="--", alpha=0.5)

    fig.suptitle(name)
    fig.tight_layout()
    fig.savefig(f"gait_{name}.pdf", bbox_inches="tight")

if __name__ == "__main__":
    plot_traj()
    for name in ("walk", "trot"):
        plot_gait(name)
    plt.show()
