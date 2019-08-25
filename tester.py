import sys
import math
from support.robot_config import make_robot_config_from_ee1
from support.problem_spec import ProblemSpec

"""
Tester script.

Use this script to test whether your output files are valid solutions. This script takes 2 arguments - an input file
and your solution file.

You should avoid modifying this file directly. You may use code from functions in this file (e.g. collision checking)
either directly or by copying into your own file. Note that the implementations in this file may be inefficient.

COMP3702 2019 Assignment 2 Support Code

Last updated by njc 24/08/19
"""


def load_output(filename):
    # return a list of RobotConfig objects
    robot_configs = []
    f = open(filename, 'r')
    for line in f:
        ee1_xy_str, ee1_angles_str, lengths_str = line.strip().split(';')
        ee1x, ee1y = tuple([float(i) for i in ee1_xy_str.split(' ')])
        ee1_angles = [float(i) * math.pi / 180 for i in ee1_angles_str.strip().split(' ')]
        lengths = [float(i) for i in lengths_str.strip().split(' ')]
        robot_configs.append(make_robot_config_from_ee1(ee1x, ee1y, ee1_angles, lengths))
    return robot_configs


def main(arglist):
    input_file = arglist[0]
    soln_file = arglist[1]

    spec = ProblemSpec(input_file)
    robot_configs = load_output(soln_file)


if __name__ == '__main__':
    main(sys.argv[1:])


