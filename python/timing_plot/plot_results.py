#!/usr/bin/env python3
import os
import argparse
import pandas as pd
import matplotlib.pyplot as plt

def plot(data, key, description, show=False, color=None, legend=None, label=None, scale=0.1, outfile=None):
    if label: # We want all matches as a single channel
        x = []
        y = []
        for k in list(filter(lambda d: d.startswith(key), data.keys())):
            df = pd.DataFrame(data[k].position, columns=['x', 'y', 'z'])
            x.extend(df['x'])
            y.extend(df['y'])
        plt.scatter(x=x, y=y, label=label, color=color, s=scale)
    else: # We want individual channels
        for k in list(filter(lambda d: key in d, data.keys())):
            df = pd.DataFrame(data[k].position, columns=['x', 'y', 'z'])
            plt.scatter(x=df['x'], y=df['y'], label=k, color=color, s=scale)

    plt.xlabel('X-axis (m)')
    plt.xlim([-70, 70])
    plt.ylabel('Y-axis (m)')
    plt.ylim([-70, 70])
    plt.title('Location of ' + r"$\bf{" + description + '}$ in the global frame')

    if legend:
        plt.legend(ncol=3)
    if outfile:
        os.makedirs(os.path.dirname(outfile), exist_ok=True)
        plt.savefig(outfile, bbox_inches="tight")
    if show:
        plt.show()

def main(args):
    data = pd.read_json('file://localhost{}'.format(args.filename))
    ## Show the individual components of each model type
    for item in ['vol', 'rock', 'scout', 'hauler', 'excavator', 'processing', 'cube']:
        ## Show just the volatiles
        plt.figure()
        plot(data, item, item, show=args.show, color=None, legend=args.legend, outfile=os.path.join(args.outdir, item+'.png') if args.outdir else None)

    ## Show all the different models on the same plot in different colors
    plt.figure()
    for key, color, scale in [['vol', 'Blue', 0.1], ['rock', 'Red', 0.1], ['scout', 'Black', 2.0], ['hauler', 'Black', 2.0], ['excavator', 'Black', 2.0], ['cube', 'Orange', 2.0], ['processing', 'Cyan', 2.0]]:
        plot(data, key, 'everything', show=False, color=color, legend=args.legend, label=key, scale=scale, outfile=os.path.join(args.outdir, 'overview.png') if args.outdir else None)
    if args.show:
        plt.show()

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Plot the simulation initial conditions.')
    parser.add_argument('filename', help='The filename of the JSON simulation results', type=str)
    parser.add_argument('-o', '--outdir', help='Which directory to save the images to', default=None, type=str)
    parser.add_argument('--show', help='Show the plots as they are being built', action='store_true', default=False)
    parser.add_argument('--legend', help='Show the legend on the plots', action='store_true', default=False)

    args = parser.parse_args()

    ## Expand the filenames
    if args.filename:
        args.filename = os.path.abspath(os.path.expanduser(args.filename))
    if args.outdir:
        args.outdir   = os.path.abspath(os.path.expanduser(args.outdir))

    main(args)
