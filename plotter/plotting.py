import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

def plot(unbiased, estimate, wrench, experiment):
    plt.rc('text', usetex=False)
    plt.rc('font', family='open sans')

    fileName = "plots/" + experiment + '_' + "az" + '.png'

    fig, ax = plt.subplots(figsize=(14, 8))
    ax.plot(unbiased['time'], unbiased['az'], label=r"$\mathcal{A}_3$",linewidth=4)
    ax.plot(estimate['time'], estimate['az'], label=r"$\hat{\mathcal{x}}_3$",linewidth=4)
    ax.tick_params(axis='both', which='major', labelsize=20)
    ax.set_title(experiment.capitalize() + " Accel", size=35)
    ax.set_xlabel("Time (s)", size=30)
    ax.set_ylabel("Accel (m/s2)", size=30)
    ax.grid(True)
    ax.legend(loc='lower right', prop={'size': 20})
    plt.savefig(fileName, bbox_inches='tight')
    plt.show()

    fileName = "plots/" + experiment + '_' + "fz" + '.png'

    fig, ax = plt.subplots(figsize=(14, 8))
    ax.plot(unbiased['time'], unbiased['fz'], label=r"$\mathcal{F}_3$",linewidth=4)
    ax.plot(estimate['time'], estimate['fz'], label=r"$\hat{\mathcal{x}}_6$",linewidth=4)
    ax.plot(wrench['time'], wrench['fz'], label=r"$\hat{\mathcal{z}}_{c,3}$", linewidth=4)
    ax.tick_params(axis='both', which='major', labelsize=20)
    ax.set_title(experiment.capitalize() + " Force", size=35)
    ax.set_xlabel("Time (s)", size=30)
    ax.set_ylabel("Force (N)", size=30)
    ax.grid(True)
    ax.legend(loc='lower right', prop={'size': 20})
    plt.savefig(fileName, bbox_inches='tight')
    plt.show()

    fileName = "plots/" + experiment + '_' + "tz" + '.png'

    fig, ax = plt.subplots(figsize=(14, 8))
    ax.plot(unbiased['time'], unbiased['tz'], label=r"$\mathcal{T}_3$",linewidth=4)
    ax.plot(estimate['time'], estimate['tz'], label=r"$\hat{\mathcal{x}}_9$",linewidth=4)
    ax.plot(wrench['time'], wrench['tz'], label=r"$\hat{\mathcal{z}}_{c,6}$",linewidth=4)
    ax.tick_params(axis='both', which='major', labelsize=20)
    ax.set_title(experiment.capitalize() + " Torque", size=35)
    ax.set_xlabel("Time (s)", size=30)
    ax.set_ylabel("Torque (Nm)", size=30)
    ax.grid(True)
    ax.legend(loc='lower right', prop={'size': 20})
    plt.savefig(fileName, bbox_inches='tight')
    plt.show()

    fileName = "plots/" + experiment + '_' + "fy" + '.png'

    fileName = "plots/" + experiment + '_' + "ay" + '.png'

    fig, ax = plt.subplots(figsize=(14, 8))
    ax.plot(unbiased['time'], unbiased['ay'], label=r"$\mathcal{A}_2$",linewidth=4)
    ax.plot(estimate['time'], estimate['ay'], label=r"$\hat{\mathcal{x}}_2$",linewidth=4)
    ax.tick_params(axis='both', which='major', labelsize=20)
    ax.set_title(experiment.capitalize() + " Accel", size=35)
    ax.set_xlabel("Time (s)", size=30)
    ax.set_ylabel("Accel (m/s2)", size=30)
    ax.grid(True)
    ax.legend(loc='lower right', prop={'size': 20})
    plt.savefig(fileName, bbox_inches='tight')
    plt.show()

    fig, ax = plt.subplots(figsize=(14, 8))
    ax.plot(unbiased['time'], unbiased['fy'], label=r"$\mathcal{F}_2$",linewidth=4)
    ax.plot(estimate['time'], estimate['fy'], label=r"$\hat{\mathcal{x}}_5$",linewidth=4)
    ax.plot(wrench['time'], wrench['fy'], label=r"$\hat{\mathcal{z}}_{c,2}$", linewidth=4)
    ax.tick_params(axis='both', which='major', labelsize=20)
    ax.set_title(experiment.capitalize() + " Force", size=35)
    ax.set_xlabel("Time (s)", size=30)
    ax.set_ylabel("Force (N)", size=30)
    ax.grid(True)
    ax.legend(loc='lower right', prop={'size': 20})
    plt.savefig(fileName, bbox_inches='tight')
    plt.show()

    fileName = "plots/" + experiment + '_' + "ty" + '.png'

    fig, ax = plt.subplots(figsize=(14, 8))
    ax.plot(unbiased['time'], unbiased['ty'], label=r"$\mathcal{T}_2$",linewidth=4)
    ax.plot(estimate['time'], estimate['ty'], label=r"$\hat{\mathcal{x}}_8$",linewidth=4)
    ax.plot(wrench['time'], wrench['ty'], label=r"$\hat{\mathcal{z}}_{c,5}$",linewidth=4)
    ax.tick_params(axis='both', which='major', labelsize=20)
    ax.set_title(experiment.capitalize() + " Torque", size=35)
    ax.set_xlabel("Time (s)", size=30)
    ax.set_ylabel("Torque (Nm)", size=30)
    ax.grid(True)
    ax.legend(loc='lower right', prop={'size': 20})
    plt.savefig(fileName, bbox_inches='tight')
    plt.show()


baseline_estimate = pd.read_csv('results/baseline_estimate.csv')
baseline_unbiased = pd.read_csv('results/baseline_unbiased.csv')
baseline_wrench = pd.read_csv('results/baseline_cont_wrench.csv')

plot(baseline_unbiased, baseline_estimate, baseline_wrench, "baseline")

vibrations_estimate = pd.read_csv('results/vibrations_estimate.csv')
vibrations_unbiased = pd.read_csv('results/vibrations_unbiased.csv')
vibrations_wrench = pd.read_csv('results/vibrations_cont_wrench.csv')

plot(vibrations_unbiased, vibrations_estimate, vibrations_wrench, "vibrations")

contact_estimate = pd.read_csv('results/contact_estimate.csv')
contact_unbiased = pd.read_csv('results/contact_unbiased.csv')
contact_wrench = pd.read_csv('results/contact_cont_wrench.csv')

plot(contact_unbiased, contact_estimate, contact_wrench, "contact")

