import pandas as pd, numpy as np, matplotlib.pyplot as plt

d = pd.read_csv("traj.csv")
dt = 0.001

for c in ["q1", "q2"]:
    d[c + "_vel"] = np.gradient(d[c], dt)
    d[c + "_acc"] = np.gradient(d[c + "_vel"], dt)

d.plot(x="t", y=["x_ref", "z_ref"])
d.plot(x="t", y=["q1_acc", "q2_acc"])
d.plot(x="x_ref", y="z_ref")
d.plot(x="t", y="err")
