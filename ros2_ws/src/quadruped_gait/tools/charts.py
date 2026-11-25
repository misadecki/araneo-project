import pandas as pd, numpy as np, matplotlib.pyplot as plt

LEGS = ("LF", "RF", "LB", "RB")

def segment_edges(d):
    ch = d[["v_cmd", "w_cmd"]].diff().abs().sum(axis=1) > 0
    return d.t[ch].to_list()

def plot_overview(name, d):
    fig, ax = plt.subplots(2, 1, sharex=True, figsize=(10, 5))
    for i, leg in enumerate(LEGS):
        ax[0].plot(d.t, d[f"s_{i}"], label=leg)

    ax[1].plot(d.t, d.v_cmd, label="v_cmd")
    ax[1].plot(d.t, d.w_cmd, label="w_cmd")
    ax[0].set_ylabel("s [-]")
    ax[1].set_ylabel("cmd")
    ax[1].set_ylabel("t [s]")

    for a in ax:
        a.legend(ncol=4)
        a.grid(alpha=0.3)

    fig.suptitle(f"{name}: overview")
    fig.tight_layout()
    fig.savefig(f"gait_{name}_overview.pdf", bbox_inches="tight")

def plot_window(name, d, t0, t1, tag):
    w = d[(d.t >= t0) & (d.t <= t1)]
    fig, ax = plt.subplots(2, 1, sharex=True, figsize=(10, 5))
    for i, leg in enumerate(LEGS):
        ax[0].plot(w.t, np.degrees(w[f"q1_{i}"]), label=leg)
        ax[1].plot(w.t, np.degrees(w[f"q2_{i}"]), label=leg)

    ax[0].set_ylabel("q1 [deg]")
    ax[1].set_ylabel("q2 [deg]")
    ax[1].set_xlabel("t [s]")
    ax[0].legend(ncol=4)

    for a in ax:
        a.grid(alpha=0.3)
    fig.suptitle(f"{name}: {tag}")
    fig.tight_layout()
    fig.savefig(f"gait_{name}_{tag}.pdf", bbox_inches="tight")


if __name__ == "__main__":
    # plot_traj()
    for name in ("walk", "trot"):
        d = pd.read_csv(f"gait_{name}.csv")
        plot_overview(name, d)
        edges = segment_edges(d) + [d.t.iloc[-1]]
        tags = ["forward", "backward", "turn_left", "arc_left"]
        moving = [e for e in range(len(edges) - 1)
                  if (d.v_cmd[d.t >= edges[e]].iloc[0] != 0
                      or d.w_cmd[d.t >= edges[e]].iloc[0] != 0)]
        for tag, e in zip(tags, moving):
            plot_window(name, d, edges[e + 1] - 3.0, edges[e + 1], tag)
        plt.show()

