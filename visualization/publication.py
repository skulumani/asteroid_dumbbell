"""Publication quality plots for exploration

Author
------
Shankar Kulumani		GWU		skulumani@gwu.edu
"""
from point_cloud import wavefront

import numpy as np

import matplotlib
import seaborn as sns
from matplotlib import pyplot as plt
from matplotlib import colors, cm

from matplotlib2tikz import save as tikz_save

import os

time_label=r'Normalized Time'

def figsize(scale):
    fig_width_pt =  433.62001                           # Get this from LaTeX using \the\textwidth
    inches_per_pt = 1.0/72.27                       # Convert pt to inch
    golden_mean = (np.sqrt(5.0)-1.0)/2.0            # Aesthetic ratio (you could change this)
    fig_width = fig_width_pt*inches_per_pt*scale    # width in inches
    fig_height = fig_width*golden_mean              # height in inches
    fig_size = [fig_width,fig_height]
    return fig_size

def scale_figsize(wscale, hscale):
    fig_width_pt = 433.62001
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
        "axes.labelsize": 10,               # LaTeX default is 10pt font.
        "font.size": 10,
        "legend.fontsize": 10,               # Make the legend/label fonts a little smaller
        "xtick.labelsize": 10,
        "ytick.labelsize": 10,
        "figure.figsize": figsize(1),     # default fig size of 0.9 textwidth
        "figure.autolayout": True,
        "pgf.preamble": [
            r"\usepackage{siunitx}",
            ]
        }

matplotlib.rcParams.update(pgf_with_latex)
sns.set_style('ticks', pgf_with_latex)
# sns.color_palette('bright')
linewidth=3

def plot_uncertainty(time, uncertainty,img_path="/tmp", fname_suffix="", wscale=1, hscale=0.75,
                     pgf_save=False, show=True):
    """Plot uncertainty vs. time
    """

    # create a figure
    uncertainty_figure, uncertainty_ax = plt.subplots(1, 1, figsize=figsize(scale=wscale))
    uncertainty_ax.plot(time/time[-1], uncertainty/uncertainty[0],label='Uncertainty')
    uncertainty_ax.set_xlabel(time_label)
    uncertainty_ax.set_ylabel(r'Normalized Uncertainty')
    uncertainty_ax.set_xlim([0, 1])

    # save
    if pgf_save:
        fig_handles = (uncertainty_figure,)
        fig_names = ('uncertainty',)
        for fig, fname in zip(fig_handles, fig_names):
            plt.figure(fig.number)
            sns.despine()
            # plt.savefig(os.path.join(img_path, fname) +  '.pgf')
            plt.savefig(os.path.join(img_path, fname) +  '.eps', dpi=1200)
            np.savetxt(os.path.join(img_path, fname) + ".csv",np.stack((time/time[-1], uncertainty/uncertainty[0]), axis=1) , delimiter=",",
                       header="NORMALIZED_TIME, NORMALIZED_UNCERTAINTY", comments='')
            # tikz_save(os.path.join(img_path, fname) + '.tex', externalize_tables=True)
    if show:
        plt.show()

def plot_state(time, pos_inertial, pos_asteroid, fname_suffix="", wscale=1, hscale=0.75,
               pgf_save=False):
    """Plot the state trajectory in both three dimensions and mercator plot
    
    State is given in intertial frame around asteroid
    """

    # create the inertial position components plot
    pos_comp_fig, pos_comp_ax = plt.subplots(3, 1, figsize=scale_figsize(wscale, hscale))
    pos_comp_ax[0].plot(time/time[-1], pos_inertial[:, 0], linewidth=linewidth, label='x',
                        linestyle='-')
    pos_comp_ax[0].set_ylabel(r'$x$')
    pos_comp_ax[1].plot(time/time[-1], pos_inertial[:, 1], linewidth=linewidth, label='y',
                        linestyle='-')
    pos_comp_ax[1].set_ylabel(r'$y$')
    pos_comp_ax[2].plot(time/time[-1], pos_inertial[:, 2], linewidth=linewidth, label='z',
                        linestyle='-')
    pos_comp_ax[2].set_ylabel(r'$z$')
    pos_comp_ax[2].set_xlabel(time_label)
    plt.tight_layout()

    # now convert the position to spherical coordinates and plot on a map
    pos_inertial_spherical = wavefront.cartesian2spherical(pos_inertial)
    pos_mercator_inertial_fig, pos_mercator_inertial_ax = plt.subplots(1, 1, 
                                                                       figsize=scale_figsize(wscale, hscale))
    pos_mercator_inertial_ax.plot(np.rad2deg(pos_inertial_spherical[:, 2]),
                                  np.rad2deg(pos_inertial_spherical[:, 1]),
                                  linewidth=linewidth, label='Spherical Trajectory',
                                  linestyle='-')
    pos_mercator_inertial_ax.set_xlabel(r'Longitude')
    pos_mercator_inertial_ax.set_ylabel(r'Latitude')
    
    # plot the radius of the orbit vs. time
    pos_radius_fig, pos_radius_ax = plt.subplots(1, 1,
                                                 figsize=scale_figsize(wscale, hscale))
    pos_radius_ax.plot(time/time[-1], pos_inertial_spherical[:, 0],
                       linewidth=linewidth, label='Satellite radius',
                       linestyle='-')
    pos_radius_ax.set_xlabel(time_label)
    pos_radius_ax.set_ylabel(r'Radius')
    plt.show()

def plot_volume(time, volume, true_volume, img_path="/tmp", fname_suffix="",
                wscale=1, hscale=0.75, pgf_save=False, show=True):
    """Plot the volume over time"""
    
    vol_fig, vol_ax = plt.subplots(1, 1, figsize=figsize(scale=wscale))
    vol_ax.plot(time/time[-1], 100 * (volume-true_volume)/true_volume,
                label='Percent Error')
    # vol_ax.plot(time/time[-1], volume,
    #             linewidth=linewidth, label='Estimate',
    #             linestyle='-')
    # vol_ax.plot(time/time[-1], np.full_like(time, true_volume),
    #             linewidth=linewidth, label='Truth',
    #             linestyle='-')
    vol_ax.set_xlabel(time_label)
    vol_ax.set_ylabel(r'Volume Percent Error')
    vol_ax.set_xlim([0, 1])
    # vol_ax.legend()

    # save
    if pgf_save:
        fig_handles = (vol_fig,)
        fig_names = ('volume',)
        for fig, fname in zip(fig_handles, fig_names):
            plt.figure(fig.number)
            sns.despine()
            # plt.savefig(os.path.join(img_path, fname) +  '.pgf')
            plt.savefig(os.path.join(img_path, fname) +  '.eps', dpi=1200)
            # tikz_save(os.path.join(img_path, fname) + '.tex', externalize_tables=True)
            np.savetxt(os.path.join(img_path, fname) + ".csv",np.stack((time/time[-1], (volume-true_volume)/true_volume)), delimiter=",",
                       header="NORMALIZED_TIME, VOLUME_PERCENT_ERROR", comments='')
    
    if show:
        plt.show()

def plot_refinment_plots():
    """Plot the surface refinement plots"""
    pass



    


