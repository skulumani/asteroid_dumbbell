"""Publication quality plots for exploration

Author
------
Shankar Kulumani		GWU		skulumani@gwu.edu
"""
import numpy as np

import matplotlib
import seaborn as sns
from matplotlib import pyplot as plt
from matplotlib import colors, cm

def figsize(scale):
    fig_width_pt = 483.0                            # Get this from LaTeX using \the\textwidth
    inches_per_pt = 1.0/72.27                       # Convert pt to inch
    golden_mean = (np.sqrt(5.0)-1.0)/2.0            # Aesthetic ratio (you could change this)
    fig_width = fig_width_pt*inches_per_pt*scale    # width in inches
    fig_height = fig_width*golden_mean              # height in inches
    fig_size = [fig_width,fig_height]
    return fig_size

def scale_figsize(wscale, hscale):
    fig_width_pt = 483.0
    fig_height_pt = 682.0
    inches_per_pt = 1 / 72.27
    fig_width = fig_width_pt * inches_per_pt * wscale
    fig_height = fig_height_pt * inches_per_pt * hscale
    return (fig_width, fig_height)

# PGF with LaTeX
pgf_with_latex = {                      # setup matplotlib to use latex for output
        "pgf.texsystem": "pdflatex",        # change this if using xetex or lautex
        "text.usetex": True,                # use LaTeX to write all text
        "font.family": "serif",
        "font.serif": [],                   # blank entries should cause plots to inherit fonts from the document
        "font.sans-serif": [],
        "font.monospace": [],
        "axes.labelsize": 8,               # LaTeX default is 10pt font.
        "font.size": 8,
        "legend.fontsize": 8,               # Make the legend/label fonts a little smaller
        "xtick.labelsize": 8,
        "ytick.labelsize": 8,
        "figure.figsize": figsize(0.9),     # default fig size of 0.9 textwidth
        "figure.autolayout": True,
        "pgf.preamble": [
            r"\usepackage[utf8x]{inputenc}",    # use utf8 fonts becasue your computer can handle it :)
            r"\usepackage[T1]{fontenc}",        # plots will be generated using this preamble
            r"\usepackage{siunitx}",
            ]
        }

matplotlib.rcParams.update(pgf_with_latex)
sns.set_style('whitegrid', pgf_with_latex)
sns.color_palette('bright')
time_label = r'Normalized Time'
linewidth=1

def plot_uncertainty(time, uncertainty,fname_suffix="", wscale=1, hscale=0.75,
                     pgf_save=False):
    """Plot uncertainty vs. time
    """

    # create a figure
    uncertainty_figure, uncertainty_ax = plt.subplots(1, 1, figsize=scale_figsize(wscale, hscale))
    uncertainty_ax.plot(time/time[-1], uncertainty/uncertainty[0], linewidth=linewidth, linestyle='-', label='Uncertainty')
    uncertainty_ax.set_xlabel(time_label)
    uncertainty_ax.set_ylabel(r'Normalized Uncertainty')

    # save
    if pgf_save:
        fig_handles = (uncertainty_figure,)
        fig_names = ('uncertainty')
        for fig, fname in zip(fig_handles, fig_names):
            plt.figure(fig.number)
            plt.savefig(fname + '_' + fname_suffix + '.pgf')
            plt.savefig(fname + '_' + fname_suffix + '.eps', dpi=1200)

    plt.show()

