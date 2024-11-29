import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

def plot(unbiased, estimate, wrench, experiment):
    plt.rc('text', usetex=False)
    plt.rc('font', family='open sans')

    fileName = "plots/" + experiment + '_' + "az" + '.png'

    fig, ax = plt.subplots(figsize=(14, 8))
    ax.plot(unbiased['time'][50:], unbiased['az'][50:], label=r"$\mathcal{A}_3$",linewidth=4)
    ax.plot(estimate['time'][50:], estimate['az'][50:], label=r"$\hat{\mathcal{x}}_3$",linewidth=4)
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
    ax.plot(unbiased['time'][50:], unbiased['fz'][50:], label=r"$\mathcal{F}_3$",linewidth=4)
    ax.plot(estimate['time'][50:], estimate['fz'][50:], label=r"$\hat{\mathcal{x}}_6$",linewidth=4)
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
    ax.plot(unbiased['time'][50:], unbiased['tz'][50:], label=r"$\mathcal{T}_3$",linewidth=4)
    ax.plot(estimate['time'][50:], estimate['tz'][50:], label=r"$\hat{\mathcal{x}}_9$",linewidth=4)
    ax.plot(wrench['time'][50:], wrench['tz'][50:], label=r"$\hat{\mathcal{z}}_{c,6}$",linewidth=4)
    ax.tick_params(axis='both', which='major', labelsize=20)
    ax.set_title(experiment.capitalize() + " Torque", size=35)
    ax.set_xlabel("Time (s)", size=30)
    ax.set_ylabel("Torque (Nm)", size=30)
    ax.grid(True)
    ax.legend(loc='lower right', prop={'size': 20})
    plt.savefig(fileName, bbox_inches='tight')
    plt.show()

    fileName = "plots/" + experiment + '_' + "ay" + '.png'

    fig, ax = plt.subplots(figsize=(14, 8))
    ax.plot(unbiased['time'][50:], unbiased['ay'][50:], label=r"$\mathcal{A}_2$",linewidth=4)
    ax.plot(estimate['time'][50:], estimate['ay'][50:], label=r"$\hat{\mathcal{x}}_2$",linewidth=4)
    ax.tick_params(axis='both', which='major', labelsize=20)
    ax.set_title(experiment.capitalize() + " Accel", size=35)
    ax.set_xlabel("Time (s)", size=30)
    ax.set_ylabel("Accel (m/s2)", size=30)
    ax.grid(True)
    ax.legend(loc='lower right', prop={'size': 20})
    plt.savefig(fileName, bbox_inches='tight')
    plt.show()

    fileName = "plots/" + experiment + '_' + "fy" + '.png'

    fig, ax = plt.subplots(figsize=(14, 8))
    ax.plot(unbiased['time'][50:], unbiased['fy'][50:], label=r"$\mathcal{F}_2$",linewidth=4)
    ax.plot(estimate['time'][50:], estimate['fy'][50:], label=r"$\hat{\mathcal{x}}_5$",linewidth=4)
    ax.plot(wrench['time'][50:], wrench['fy'][50:], label=r"$\hat{\mathcal{z}}_{c,2}$", linewidth=4)
    ax.tick_params(axis='both', which='major', labelsize=20)
    ax.set_title(experiment.capitalize() + " Force ", size=35)
    ax.set_xlabel("Time (s)", size=30)
    ax.set_ylabel("Force (N)", size=30)
    ax.grid(True)
    ax.legend(loc='lower right', prop={'size': 20})
    plt.savefig(fileName, bbox_inches='tight')
    plt.show()

    fileName = "plots/" + experiment + '_' + "ty" + '.png'

    fig, ax = plt.subplots(figsize=(14, 8))
    ax.plot(unbiased['time'][50:], unbiased['ty'][50:], label=r"$\mathcal{T}_2$",linewidth=4)
    ax.plot(estimate['time'][50:], estimate['ty'][50:], label=r"$\hat{\mathcal{x}}_8$",linewidth=4)
    ax.plot(wrench['time'][50:], wrench['ty'][50:], label=r"$\hat{\mathcal{z}}_{c,5}$",linewidth=4)
    ax.tick_params(axis='both', which='major', labelsize=20)
    ax.set_title(experiment.capitalize() + " Torque", size=35)
    ax.set_xlabel("Time (s)", size=30)
    ax.set_ylabel("Torque (Nm)", size=30)
    ax.grid(True)
    ax.legend(loc='lower right', prop={'size': 20})
    plt.savefig(fileName, bbox_inches='tight')
    plt.show()

    fileName = "plots/" + experiment + '_' + "ax" + '.png'

    fig, ax = plt.subplots(figsize=(14, 8))
    ax.plot(unbiased['time'][50:], unbiased['ax'][50:], label=r"$\mathcal{A}_1$",linewidth=4)
    ax.plot(estimate['time'][50:], estimate['ax'][50:], label=r"$\hat{\mathcal{x}}_1$",linewidth=4)
    ax.tick_params(axis='both', which='major', labelsize=20)
    ax.set_title(experiment.capitalize() + " Accel", size=35)
    ax.set_xlabel("Time (s)", size=30)
    ax.set_ylabel("Accel (m/s2)", size=30)
    ax.grid(True)
    ax.legend(loc='lower right', prop={'size': 20})
    plt.savefig(fileName, bbox_inches='tight')
    plt.show()

    fileName = "plots/" + experiment + '_' + "fx" + '.png'

    fig, ax = plt.subplots(figsize=(14, 8))
    ax.plot(unbiased['time'][50:], unbiased['fx'][50:], label=r"$\mathcal{F}_1$",linewidth=4)
    ax.plot(estimate['time'][50:], estimate['fx'][50:], label=r"$\hat{\mathcal{x}}_4$",linewidth=4)
    ax.plot(wrench['time'][50:], wrench['fx'][50:], label=r"$\hat{\mathcal{z}}_{c,1}$", linewidth=4)
    ax.tick_params(axis='both', which='major', labelsize=20)
    ax.set_title(experiment.capitalize() + " Force", size=35)
    ax.set_xlabel("Time (s)", size=30)
    ax.set_ylabel("Force (N)", size=30)
    ax.grid(True)
    ax.legend(loc='lower right', prop={'size': 20})
    plt.savefig(fileName, bbox_inches='tight')
    plt.show()

    fileName = "plots/" + experiment + '_' + "tx" + '.png'

    fig, ax = plt.subplots(figsize=(14, 8))
    ax.plot(unbiased['time'][50:], unbiased['tx'][50:], label=r"$\mathcal{T}_1$",linewidth=4)
    ax.plot(estimate['time'][50:], estimate['tx'][50:], label=r"$\hat{\mathcal{x}}_7$",linewidth=4)
    ax.plot(wrench['time'][50:], wrench['tx'][50:], label=r"$\hat{\mathcal{z}}_{c,4}$",linewidth=4)
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

