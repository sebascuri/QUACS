#!/usr/bin/env python
import numpy as np
from scipy.signal.waveforms import chirp, sweep_poly
from numpy import poly1d

from pylab import figure, plot, show, xlabel, ylabel, subplot, grid, title, yscale, savefig, clf

FIG_SIZE = (7.5, 3.75)

def make_linear(f0, t1, f1, filename=None, fig_size=FIG_SIZE):
    t = np.linspace(0, t1, 5001)
    w = chirp(t, f0=f0, f1=f1, t1=t1, method='linear')

    figure(1, figsize=fig_size)
    clf()

    subplot(2,1,1)
    plot(t, w)
    tstr = "Linear Chirp, f(0)=%g, f(%g)=%g" % (f0, t1, f1)
    title(tstr)

    subplot(2,1,2)
    plot(t, f0 + (f1-f0)*t/t1, 'r')
    grid(True)
    ylabel('Frequency (Hz)')
    xlabel('time (sec)')
    if filename is None:
        show()
    else:
        savefig(filename)

def make_quadratic(f0, t1, f1, filename=None, fig_size=FIG_SIZE):
    t = np.linspace(0, t1, 5001)
    w = chirp(t, f0=f0, f1=f1, t1=t1, method='quadratic')

    figure(1, figsize=fig_size)
    clf()

    subplot(2,1,1)
    plot(t, w)
    tstr = "Quadratic Chirp, f(0)=%g, f(%g)=%g" % (f0, t1, f1)
    title(tstr)

    subplot(2,1,2)
    plot(t, f0 + (f1-f0)*t**2/t1**2, 'r')
    grid(True)
    ylabel('Frequency (Hz)')
    xlabel('time (sec)')
    if filename is None:
        show()
    else:
        savefig(filename)

def make_quadratic_v0false(f0, t1, f1, filename=None, fig_size=FIG_SIZE):
    t = np.linspace(0, t1, 5001)
    w = chirp(t, f0=f0, f1=f1, t1=t1, method='quadratic', vertex_zero=False)

    figure(1, figsize=fig_size)
    clf()

    subplot(2,1,1)
    plot(t, w)
    tstr = "Quadratic Chirp, f(0)=%g, f(%g)=%g (vertex_zero=False)" % (f0, t1, f1)
    title(tstr)

    subplot(2,1,2)
    plot(t, f1 - (f1-f0)*(t1-t)**2/t1**2, 'r')
    grid(True)
    ylabel('Frequency (Hz)')
    xlabel('time (sec)')
    if filename is None:
        show()
    else:
        savefig(filename)

def make_logarithmic(f0, t1, f1, filename=None, fig_size=FIG_SIZE):
    t = np.linspace(0, t1, 5001)
    w = chirp(t, f0=f0, f1=f1, t1=t1, method='logarithmic')

    figure(1, figsize=fig_size)
    clf()

    subplot(2,1,1)
    plot(t, w)
    tstr = "Logarithmic Chirp, f(0)=%g, f(%g)=%g" % (f0, t1, f1)
    title(tstr)

    subplot(2,1,2)
    plot(t, f0 * (f1/f0)**(t/t1), 'r')
    # yscale('log')
    grid(True)
    ylabel('Frequency (Hz)')
    xlabel('time (sec)')
    if filename is None:
        show()
    else:
        savefig(filename)

def make_hyperbolic(f0, t1, f1, filename=None, fig_size=FIG_SIZE):
    t = np.linspace(0, t1, 5001)
    w = chirp(t, f0=f0, f1=f1, t1=t1, method='hyperbolic')

    figure(1, figsize=fig_size)
    clf()
    
    subplot(2,1,1)
    plot(t, w)
    tstr = "Hyperbolic Chirp, f(0)=%g, f(%g)=%g" % (f0, t1, f1)
    title(tstr)

    subplot(2,1,2)
    plot(t, f0 * f1 * t1 / ((f0 - f1)*t + f1*t1), 'r')
    grid(True)
    ylabel('Frequency (Hz)')
    xlabel('time (sec)')
    if filename is None:
        show()
    else:
        savefig(filename)

def make_sweep_poly(filename=None, fig_size=FIG_SIZE):
    p = poly1d([0.05, -0.75, 2.5, 5.0])
     
    t = np.linspace(0, t1, 5001)
    w = sweep_poly(t, p)

    figure(1, figsize=fig_size)
    clf()
    
    subplot(2,1,1)
    plot(t, w)
    tstr = "Sweep Poly, $f(t) = 0.05t^3 - 0.75t^2 + 2.5t + 5$"
    title(tstr)

    subplot(2,1,2)
    plot(t, p(t), 'r')
    grid(True)
    ylabel('Frequency (Hz)')
    xlabel('time (sec)')
    if filename is None:
        show()
    else:
        savefig(filename)

    
if __name__ == "__main__":
    f0 = 0.0
    t1 = 10.0
    f1 = 10.0
    make_linear(f0, t1, f1, 'chirp_linear.png')
    make_quadratic(f0, t1, f1, 'chirp_quadratic.png')
    #make_quadratic_v0false(f0, t1, f1, 'chirp_quadratic_v0false.png')
    #make_hyperbolic(f0, t1, f1, 'chirp_hyperbolic.png')
    #make_logarithmic(f0, t1, f1, 'chirp_logarithmic.png')

    #make_sweep_poly(filename='sweep_poly.png')