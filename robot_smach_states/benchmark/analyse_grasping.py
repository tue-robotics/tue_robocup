#! /usr/bin/env python

import argparse
from dateutil.parser import parse as parse_date
import matplotlib.pyplot as plt
import pandas as pd

RESULT_FIELDS = [ 'timestamp', 'robot',
                  'start_waypoint',
                  'expected_class', 'observed_class',
                  'id',
                  'inspect_duration',
                  'grab_duration',
                  'x', 'y', 'z']

def analyse(results_file, start=None, end=None, plot=False):
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

    if plot:
        df.plot()
        plt.show()


if __name__ == "__main__":

    parser = argparse.ArgumentParser(description='Benchmark grasping, for a single item or multiple items at various locations'
                                                 'By specifying ANY as the class, anything class of object will '
                                                 'be picked up without looking at the object type'
                                                 'By specifying a specific class but --non-strict-class as well, '
                                                 'the robot will grab something else if the specified thing is not available')
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
