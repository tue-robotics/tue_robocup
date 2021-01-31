#! /usr/bin/env python

import argparse
from dateutil.parser import parse as parse_date
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np


RESULT_FIELDS = [ 'timestamp', 'robot',
                  'start_waypoint',
                  'expected_class', 'observed_class',
                  'id',
                  'inspect_duration',
                  'grab_duration',
                  'x', 'y', 'z']

Z_HIST_STEP = 0.05


def analyse(results_file, start=None, end=None, plot=False):
    """

    :param results_file: an open .csv file
    :param start: a datetime after which to consider the results
    :param end: a datetime before which to consider the results
    :param plot: Open a window with graphical plots
    """
    fig, axs = plt.subplots(2, 1)

    df = pd.read_csv(results_file)
    assert df.columns.tolist() == RESULT_FIELDS, \
        "CSV file need fields {}".format(','.join(RESULT_FIELDS))

    df.index = pd.to_datetime(df['timestamp'], format='%Y-%m-%d %H:%M:%S')

    if not start:
        start = df.index[0]
    if not end:
        end = df.index[-1]
    mask = (df.index > start) & (df.index <= end)
    df = df.loc[mask]

    print(df.describe())

    zs = df['z'].dropna(how='any')
    z_histogram_success = zs.hist(bins=int(np.ceil(abs(max(zs) - min(zs)) / Z_HIST_STEP)),
                                  grid=True,
                                  ax=axs[1])
    if plot:
        axs[0].title.set_text('Duration through time')
        df[['inspect_duration', 'grab_duration']].plot(ax=axs[0], grid=True)

        axs[1].title.set_text('Grasp height frequency')
        z_histogram_success.plot(x=z_histogram_success)
        plt.xlabel('z')
        plt.ylabel('# grasps')
        plt.show()


if __name__ == "__main__":

    parser = argparse.ArgumentParser(description='Analyse results of grasping benchmark')
    parser.add_argument("--output", default="grasp_benchmark.csv",
                        help="Output of the benchmark results (input for analysis)")
    parser.add_argument("--plot", action='store_true',
                        help="Make a plot of the results")
    parser.add_argument("--start", help="Take only measurement since the start into account. Specify timestamp as in .csv file")
    parser.add_argument("--end", help="Take only measurement until the end into account. Specify timestamp as in .csv file")

    args = parser.parse_args()

    with open(args.output) as csv_file:
        analyse(csv_file,
                plot=args.plot,
                start=parse_date(args.start) if args.start else None,
                end=parse_date(args.end) if args.end else None)
