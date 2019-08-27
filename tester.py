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


def test_bounding_box(p1, q1, p2, q2):
    # return true for collision possible, false otherwise
    p1x, p1y = p1
    q1x, q1y = q1
    p2x, p2y = p2
    q2x, q2y = q2
    x1_min = min(p1x, q1x)
    x1_max = max(p1x, q1x)
    x2_min = min(p2x, q2x)
    x2_max = max(p2x, q2x)
    if x1_max < x2_min or x2_max < x1_min:
        return False

    y1_min = min(p1y, q1y)
    y1_max = max(p1y, q1y)
    y2_min = min(p2y, q2y)
    y2_max = max(p2y, q2y)
    if y1_max < y2_min or y2_max < y1_min:
        return False

    return True


def determinant(a, b, c, d):
    return (a * d) - (b * c)


def triangle_orientation(a, b, c):
    ax, ay = a
    bx, by = b
    cx, cy = c
    area = determinant(bx, by, cx, cy) - determinant(ax, ay, cx, cy) + determinant(ax, ay, bx, by)
    if area > 0:
        return 1
    elif area < 0:
        return -1
    else:
        return 0


def test_orientation(p1, q1, p2, q2):
    # return true for collision, false otherwise
    if triangle_orientation(p1, q1, p2) == triangle_orientation(p1, q1, q2):
        return False
    if triangle_orientation(p2, q2, p1) == triangle_orientation(p2, q2, q1):
        return False
    return True


def test_line_collision(line1, line2):
    # return true for collision, false otherwise
    p1, q1 = line1
    p2, q2 = line2
    if not test_bounding_box(p1, q1, p2, q2):
        return False
    return test_orientation(p1, q1, p2, q2)


def test_angle_constraints(config):
    # return true for pass, false for fail
    for a in config.ee1_angles:
        if 11 * math.pi / 12 < a < 13 * math.pi / 12:
            # internal angle tighter than 15 degrees
            return False
    return True


def test_length_constraints(config, spec):
    # return true for pass, false for fail
    for i in range(spec.num_segments):
        if config.lengths[i] < spec.min_lengths[i] or config.lengths[i] > spec.max_lengths[i]:
            return False
    return True


def point_is_close(x1, y1, x2, y2, tolerance):
    return abs(x2 - x1) + abs(y2 - y1) < tolerance


def test_grapple_point_constraint(config, spec):
    # return true for pass, false for fail
    ee1x, ee1y = config.points[0]
    ee2x, ee2y = config.points[-1]
    for gpx, gpy in spec.grapple_points:
        if point_is_close(ee1x, ee1y, gpx, gpy, spec.TOLERANCE) or point_is_close(ee2x, ee2y, gpx, gpy, spec.TOLERANCE):
            return True
    return False


def test_internal_collision(config, spec):
    # return true for pass, false for fail
    for i in range(spec.num_segments - 1):
        p1 = config.points[i]
        q1 = config.points[i+1]

        for j in range(i + 1, spec.num_segments):
            p2 = config.points[j]
            q2 = config.points[j+1]

            if test_line_collision((p1, q1), (p2, q2)):
                return False

    return True


def test_obstacle_collision(config, spec):
    pass


def test_robot_config(config, spec):
    # check simplest first
    pass


def main(arglist):
    input_file = arglist[0]
    soln_file = arglist[1]

    spec = ProblemSpec(input_file)
    robot_configs = load_output(soln_file)


if __name__ == '__main__':
    main(sys.argv[1:])


