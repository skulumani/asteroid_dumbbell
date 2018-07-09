"""Publication quality plots for exploration
    # grid_dist_masked = grid_dist
    # grid_dist_masked = grid_dist
    # grid_dist_masked[slope_mask] = np.pi

    # fig_dist, ax_dist = plt.subplots(2, 1)
    # img_dist = ax_dist[0].imshow(grid_dist, extent=(-np.pi, np.pi, -np.pi/2, np.pi/2),
    #                           origin="lower")
    # ax_dist[0].set_title("Distance to surface")
    # ax_dist[0].set_xlabel("Longitude")
    # ax_dist[0].set_ylabel("Latitude")
    # fig_dist.colorbar(img_dist, ax=ax_dist[0])
    
    # img_dist_mask = ax_dist[1].imshow(grid_dist_masked, extent=(-np.pi, np.pi, -np.pi/2, np.pi/2),
    #                                origin="lower")
    # ax_dist[1].set_title("Masked distance to surface")
    # ax_dist[1].set_xlabel("Longitude")
    # ax_dist[1].set_ylabel("Latitude")
    # dist_cbar_masked = fig_dist.colorbar(img_dist_mask, ax=ax_dist[1])
    # dist_cbar_masked.set_clim(0, np.pi)
    # grid_dist_masked[slope_mask] = np.pi

    # fig_dist, ax_dist = plt.subplots(2, 1)
    # img_dist = ax_dist[0].imshow(grid_dist, extent=(-np.pi, np.pi, -np.pi/2, np.pi/2),
    #                           origin="lower")
    # ax_dist[0].set_title("Distance to surface")
    # ax_dist[0].set_xlabel("Longitude")
    # ax_dist[0].set_ylabel("Latitude")
    # fig_dist.colorbar(img_dist, ax=ax_dist[0])
    
    # img_dist_mask = ax_dist[1].imshow(grid_dist_masked, extent=(-np.pi, np.pi, -np.pi/2, np.pi/2),
    #                                origin="lower")
    # ax_dist[1].set_title("Masked distance to surface")
    # ax_dist[1].set_xlabel("Longitude")
    # ax_dist[1].set_ylabel("Latitude")
    # dist_cbar_masked = fig_dist.colorbar(img_dist_mask, ax=ax_dist[1])
    # dist_cbar_masked.set_clim(0, np.pi)

Author
------
Shankar Kulumani		GWU		skulumani@gwu.edu
"""
from point_cloud import wavefront

import numpy as np

import matplotlib
import seaborn as sns
from matplotlib import pyplot as plt
from mpl_toolkits.axes_grid1 import make_axes_locatable
from matplotlib import colors, cm
from scipy import integrate, interpolate, ndimage

from matplotlib2tikz import save as tikz_save

import os

time_label=r'Normalized Time'
cmap = 'viridis'

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
                       header="NORMALIZED_TIME, NORMALIZED_UNCERTAINTY", comments='',
                       fmt="%6.3f")
            # tikz_save(os.path.join(img_path, fname) + '.tex', externalize_tables=True)
    if show:
        plt.show()

def plot_state(time, pos_inertial, pos_asteroid, fname_suffix="",
               img_path="/tmp", wscale=1, hscale=0.75,
               pgf_save=True, show=True):
    """Plot the state trajectory in both three dimensions and mercator plot
    
    State is given in intertial frame around asteroid
    """

    # create the inertial position components plot
    pos_comp_fig, pos_comp_ax = plt.subplots(3, 1, figsize=figsize(wscale))
    pos_comp_ax[0].plot(time/time[-1], pos_inertial[:, 0])
    pos_comp_ax[0].set_ylabel(r'$x$')
    pos_comp_ax[1].plot(time/time[-1], pos_inertial[:, 1])
    pos_comp_ax[1].set_ylabel(r'$y$')
    pos_comp_ax[2].plot(time/time[-1], pos_inertial[:, 2])
    pos_comp_ax[2].set_ylabel(r'$z$')
    pos_comp_ax[2].set_xlabel(time_label)
    pos_comp_ax[0].set_title(r'Inertial Position')

    ast_comp_fig, ast_comp_ax = plt.subplots(3, 1, figsize=figsize(wscale))
    ast_comp_ax[0].plot(time/time[-1], pos_asteroid[:, 0])
    ast_comp_ax[0].set_ylabel(r'$x$')
    ast_comp_ax[1].plot(time/time[-1], pos_asteroid[:, 1])
    ast_comp_ax[1].set_ylabel(r'$y$')
    ast_comp_ax[2].plot(time/time[-1], pos_asteroid[:, 2])
    ast_comp_ax[2].set_ylabel(r'$z$')
    ast_comp_ax[2].set_xlabel(time_label)
    ast_comp_ax[0].set_title(r'Asteroid Position')

    # now convert the position to spherical coordinates and plot on a map
    pos_inertial_spherical = wavefront.cartesian2spherical(pos_inertial)
    pos_mercator_inertial_fig, pos_mercator_inertial_ax = plt.subplots(1, 1, 
                                                                       figsize=figsize(wscale))
    pos_mercator_inertial_ax.plot(np.rad2deg(pos_inertial_spherical[:, 2]),
                                  np.rad2deg(pos_inertial_spherical[:, 1]))
    pos_mercator_inertial_ax.set_xlabel(r'Longitude')
    pos_mercator_inertial_ax.set_ylabel(r'Latitude')
    pos_mercator_inertial_ax.set_title(r'Inertial Position')    

    ast_spherical = wavefront.cartesian2spherical(pos_asteroid)
    ast_mercator_fig, ast_mercator_ax = plt.subplots(1, 1, 
                                                              figsize=figsize(wscale))
    ast_mercator_ax.plot(np.rad2deg(ast_spherical[:, 2]),
                         np.rad2deg(ast_spherical[:, 1]))
    ast_mercator_ax.set_xlabel(r'Longitude')
    ast_mercator_ax.set_ylabel(r'Latitude')
    ast_mercator_ax.set_title(r'Asteroid Position')    

    # plot the radius of the orbit vs. time
    pos_radius_fig, pos_radius_ax = plt.subplots(1, 1,
                                                 figsize=figsize(scale=wscale))
    pos_radius_ax.plot(time/time[-1], pos_inertial_spherical[:, 0])
    pos_radius_ax.set_xlabel(time_label)
    pos_radius_ax.set_ylabel(r'Radius')
    pos_radius_ax.set_title(r'Radius')    

    np.savetxt(os.path.join(img_path, 'state_trajectory') + ".csv",np.stack((time/time[-1],
                                                                             pos_inertial[:, 0],
                                                                             pos_inertial[:, 1],
                                                                             pos_inertial[:, 2],
                                                                             pos_asteroid[:, 0],
                                                                             pos_asteroid[:, 1],
                                                                             pos_asteroid[:, 2],
                                                                             pos_inertial_spherical[:, 0]), axis=1) , delimiter=",",
               header="NORMALIZED_TIME, INERTIAL_X, INERTIAL_Y, INERTIAL_Z, ASTEROID_X, ASTEROID_Y, ASTEROID_Z, RADIUS", comments='',
               fmt="%6.3f")
    

    if pgf_save:
        fig_handles = (pos_comp_fig, ast_comp_fig, pos_mercator_inertial_fig,
                       ast_mercator_fig, pos_radius_fig,)
        fig_names = ('inertial_comp', 'ast_comp', 'inertial_spherical',
                     'ast_spherical', 'radius')
        for fig, fname in zip(fig_handles, fig_names):
            plt.figure(fig.number)
            sns.despine()
            # plt.savefig(os.path.join(img_path, fname) +  '.pgf')
            plt.savefig(os.path.join(img_path, fname) +  '.eps', dpi=1200)
            # tikz_save(os.path.join(img_path, fname) + '.tex', externalize_tables=True)
    if show:
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
            np.savetxt(os.path.join(img_path, fname) + ".csv",np.stack((time/time[-1], (volume-true_volume)/true_volume), axis=1), delimiter=",",
                       header="NORMALIZED_TIME, VOLUME_PERCENT_ERROR", comments='',
                       fmt="%6.3f")
    
    if show:
        plt.show()

def plot_refinement_plots(spherical_vertices, grid_long, grid_lat, delta_angle,
                          grid_slope, grid_dist, img_path="/tmp", wscale=1, max_slope=5, 
                          show=True, pgf_save=False):
    """Plot the surface refinement plots"""
    r = spherical_vertices[:, 0]
    lat = spherical_vertices[:, 1]
    long = spherical_vertices[:, 2]
    grid_r = interpolate.griddata(np.vstack((long, lat)).T, r, (grid_long, grid_lat), method='nearest')
    grid_r_smooth = ndimage.gaussian_filter(grid_r, sigma=10*delta_angle)
    
    # Vertex density
    fig_density, ax_density = plt.subplots(1, 1, figsize=figsize(scale=wscale))
    # ax_density.plot(long, lat, 'k.', ms=1)
    d = ax_density.hist2d(long, lat, grid_long.shape,cmap=cmap )[3]
    fig_density.colorbar(d, orientation='vertical')
    ax_density.set_title(r'Vertex density')
    ax_density.set_xlabel(r'Longitude')
    ax_density.set_ylabel(r'Latitude')

    slope_mask = grid_slope > max_slope;

    grid_slope_masked = np.copy(grid_slope)
    grid_slope_masked[slope_mask] = max_slope
    
    # slope surface plot
    fig_slope, ax_slope = plt.subplots(1, 1, figsize=figsize(scale=wscale))
    # ax_slope.contour(grid_long, grid_lat, grid_slope)
    img_slope = ax_slope.imshow(grid_slope, extent=(-np.pi, np.pi, -np.pi/2, np.pi/2),
                                origin="lower", cmap=cmap)
    ax_slope.set_title(r'Surface Slope')
    ax_slope.set_xlabel(r'Longitude')
    ax_slope.set_ylabel(r'Latitude')
    fig_slope.colorbar(img_slope, ax=ax_slope)
    
    fig_slope_masked, ax_slope_masked = plt.subplots(1, 1, figsize=figsize(scale=wscale))
    img_slope_masked = ax_slope_masked.imshow(grid_slope_masked, extent=(-np.pi, np.pi, -np.pi/2, np.pi/2),
                                              origin="lower",cmap=cmap)
    ax_slope_masked.set_title(r'Surface Slope Masked')
    ax_slope_masked.set_xlabel(r'Longitude')
    ax_slope_masked.set_ylabel(r'Latitude')
    slope_cbar_masked = fig_slope_masked.colorbar(img_slope_masked, ax=ax_slope_masked,
                                                  cmap=cmap)
    slope_cbar_masked.set_clim(0, 5)
    
    # distance to the surface
    grid_dist_masked = np.copy(grid_dist)
    grid_dist_masked[slope_mask] = np.pi

    fig_dist, ax_dist = plt.subplots(1, 1, figsize=figsize(scale=wscale))
    img_dist = ax_dist.imshow(grid_dist, extent=(-np.pi, np.pi, -np.pi/2, np.pi/2),
                                 origin="lower", cmap=cmap)
    ax_dist.set_title("Distance to surface")
    ax_dist.set_xlabel("Longitude")
    ax_dist.set_ylabel("Latitude")
    fig_dist.colorbar(img_dist, ax=ax_dist)
    
    fig_dist_masked, ax_dist_masked = plt.subplots(1, 1, figsize=figsize(scale=wscale))
    img_dist_mask = ax_dist_masked.imshow(grid_dist_masked, extent=(-np.pi, np.pi,
                                                                -np.pi/2,
                                                                np.pi/2),
                                      origin="lower", cmap=cmap)
    ax_dist_masked.set_title("Masked distance to surface")
    ax_dist_masked.set_xlabel("Longitude")
    ax_dist_masked.set_ylabel("Latitude")
    dist_cbar_masked = fig_dist_masked.colorbar(img_dist_mask, ax=ax_dist_masked)
    dist_cbar_masked.set_clim(0, np.pi)

    # build an image of random science value over entire surface
    np.random.seed(9)
    grid_science = np.random.rand(grid_dist.shape[0], grid_dist.shape[1])
    grid_science = ndimage.gaussian_filter(grid_science, 50*delta_angle)
    grid_science_masked = np.copy(grid_science)
    grid_science_masked[slope_mask] = 0

    fig_science, ax_science = plt.subplots(1, 1, figsize=figsize(scale=wscale))
    img_science = ax_science.imshow(grid_science, extent=(-np.pi, np.pi, -np.pi/2, np.pi/2),
                                       origin="lower", cmap=cmap)
    ax_science.set_title("Science Value")
    ax_science.set_xlabel("Longitude")
    ax_science.set_ylabel("Latitude")
    fig_science.colorbar(img_science, ax=ax_science)

    fig_science_masked, ax_science_masked = plt.subplots(1, 1, figsize=figsize(scale=wscale))
    img_science_mask = ax_science_masked.imshow(grid_science_masked, extent=(-np.pi, np.pi, -np.pi/2, np.pi/2),
                                            origin="lower", cmap=cmap)
    ax_science_masked.set_title("Masked Science Value")
    ax_science_masked.set_xlabel("Longitude")
    ax_science_masked.set_ylabel("Latitude")
    fig_science.colorbar(img_science_mask, ax=ax_science_masked)

    # normalize all the cost arrays, sum and plot together then find the minimum
    total_cost = (-grid_science_masked / np.max(grid_science_masked) +
                  2 * grid_dist_masked / np.max(grid_dist_masked)
                  + grid_slope_masked/ np.max(grid_slope_masked)) / 2
    total_cost_smooth = ndimage.gaussian_filter(total_cost, 10*delta_angle)
    total_cost_smooth_masked = np.copy(total_cost_smooth)
    total_cost_smooth_masked[slope_mask] = 3

    # find minimum
    min_index = np.unravel_index(total_cost_smooth_masked.argmin(),
                                 total_cost_smooth_masked.shape)
    desired_pos_spherical = np.array([grid_r[min_index[0], min_index[1]],
                                      grid_lat[min_index[0], min_index[1]],
                                      grid_long[min_index[0], min_index[1]]])
    desired_pos_cartesian = wavefront.spherical2cartesian(desired_pos_spherical)

    fig_cost, ax_cost = plt.subplots(1, 1, figsize=figsize(scale=wscale))
    img_cost = ax_cost.imshow(total_cost_smooth_masked, extent=(-np.pi, np.pi, -np.pi/2, np.pi/2),
                               origin="lower", cmap=cmap)
    ax_cost.set_title("Landing cost")
    ax_cost.set_xlabel("Longitude")
    ax_cost.set_ylabel("Latitude")
    fig_cost.colorbar(img_cost)
    ax_cost.plot(grid_long[min_index[0], min_index[1]],
                 grid_lat[min_index[0], min_index[1]],
                 marker='o', color='blue')

    if pgf_save:
        fig_handles = (fig_density, fig_slope, fig_slope_masked,
                       fig_dist, fig_dist_masked, fig_science, fig_science_masked,
                       fig_cost)
        fig_names = ('density', 'slope', 'slope_masked', 'dist', 'dist_masked',
                     'science', 'science_masked', 'cost',)
        for fig, fname in zip(fig_handles, fig_names):
            plt.figure(fig.number)
            # plt.savefig(os.path.join(img_path, fname) +  '.pgf')
            plt.savefig(os.path.join(img_path, fname) +  '.eps', dpi=1200)
            # tikz_save(os.path.join(img_path, fname) + '.tex', externalize_tables=True)
    
    if show:
        plt.show()

    return desired_pos_cartesian



    


