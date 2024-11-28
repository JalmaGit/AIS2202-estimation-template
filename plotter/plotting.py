import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

baseline_estimate = pd.read_csv('results/baseline_estimate.csv')
baseline_unbiased = pd.read_csv('results/baseline_unbiased.csv')
baseline_wrench = pd.read_csv('results/baseline_cont_wrench.csv')

fig, ax = plt.subplots()
ax.plot(baseline_unbiased['time'], baseline_unbiased['ty'], label='T_2')
ax.plot(baseline_estimate['time'], baseline_estimate['ty'], label='x_8')
ax.plot(baseline_wrench['time'], baseline_wrench['ty'], label='z_{c,5}')
ax.legend()
plt.show()
fig, ax = plt.subplots()
ax.plot(baseline_unbiased['time'], baseline_unbiased['az'], label='A_2')
ax.plot(baseline_estimate['time'], baseline_estimate['az'], label='x_2')
ax.legend()
plt.show()
