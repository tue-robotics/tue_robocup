#! /usr/bin/env python

import argparse
import pandas as pd
import matplotlib.pyplot as plt

RESULT_FIELDS = [ 'timestamp', 'robot',
                  'start_waypoint',
                  'expected_class', 'observed_class',
                  'id',
                  'inspect_duration',
                  'grab_duration',
                  'x', 'y', 'z']

def analyse(results_file, plot=False):
    df = pd.read_csv(results_file)
    assert df.columns.tolist() == RESULT_FIELDS, \
        "CSV file need fields {}".format(','.join(RESULT_FIELDS))
    df.index = df['timestamp']
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

    args = parser.parse_args()

    with open(args.output) as csv_file:
        analyse(csv_file, plot=args.plot)
