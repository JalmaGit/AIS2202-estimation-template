import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

def plot(unbiased, estimate, wrench, experiment):
    plt.rc('text', usetex=False)
    plt.rc('font', family='serif')

    fileName = "plots/" + experiment + '_' + "fz" + '.png'

    fig, ax = plt.subplots(figsize=(12, 8))
    ax.plot(unbiased['time'], unbiased['fz'], label=r"$\mathcal{F}_3$")
    ax.plot(estimate['time'], estimate['fz'], label=r"$\hat{\mathcal{x}}_6$")
    ax.plot(wrench['time'], wrench['fz'], label=r"$\hat{\mathcal{z}}_{c,3}$")
    ax.legend(loc='lower right', prop={'size': 20})
    plt.savefig(fileName, bbox_inches='tight')
    plt.show()

    fileName = "plots/" + experiment + '_' + "ty" + '.png'

    fig, ax = plt.subplots(figsize=(12, 8))
    ax.plot(unbiased['time'], unbiased['ty'], label=r"$\mathcal{T}_2$")
    ax.plot(estimate['time'], estimate['ty'], label=r"$\hat{\mathcal{x}}_8$")
    ax.plot(wrench['time'], wrench['ty'], label=r"$\hat{\mathcal{z}}_{c,5}$")
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

