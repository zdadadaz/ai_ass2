import sys
import math
from support.robot_config import RobotConfig, make_robot_config_from_ee1
from support.problem_spec import ProblemSpec
from support.obstacle import Obstacle
from support.angle import Angle

class test_robot:
    def __init__(self,spec):
        self.spec = spec

    def load_output(self,filename):
    # return a list of RobotConfig objects
        robot_configs = []
        f = open(filename, 'r')
        for line in f:
            ee1_xy_str, ee1_angles_str, lengths_str = line.strip().split(';')
            ee1x, ee1y = tuple([float(i) for i in ee1_xy_str.split(' ')])
            ee1_angles = [Angle(degrees=float(i)) for i in ee1_angles_str.strip().split(' ')]
            lengths = [float(i) for i in lengths_str.strip().split(' ')]
            robot_configs.append(make_robot_config_from_ee1(ee1x, ee1y, ee1_angles, lengths))
        return robot_configs


    def test_bounding_box(self,p1, q1, p2, q2):
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


    def determinant(self,a, b, c, d):
        return (a * d) - (b * c)


    def triangle_orientation(self,a, b, c):
        ax, ay = a
        bx, by = b
        cx, cy = c
        area = self.determinant(bx, by, cx, cy) - self.determinant(ax, ay, cx, cy) + self.determinant(ax, ay, bx, by)
        if area > 0:
            return 1
        elif area < 0:
            return -1
        else:
            return 0


    def test_orientation(self,p1, q1, p2, q2):
        # return true for collision, false otherwise
        if self.triangle_orientation(p1, q1, p2) == self.triangle_orientation(p1, q1, q2):
            return False
        if self.triangle_orientation(p2, q2, p1) == self.triangle_orientation(p2, q2, q1):
            return False
        return True


    def test_line_collision(self,line1, line2):
        # return true for collision, false otherwise
        p1, q1 = line1
        p2, q2 = line2
        if not self.test_bounding_box(p1, q1, p2, q2):
            return False
        return self.test_orientation(p1, q1, p2, q2)


    def test_environment_bounds(self,config):
        # return true for pass, false for fail
        for x, y in config.points:
            if not 0.0 <= x <= 1.0:
                return False
            if not 0.0 <= y <= 1.0:
                return False
        return True


    def test_angle_constraints(self,config, spec):
        # return true for pass, false for fail
        for i in range(1, spec.num_segments):
            a = config.ee1_angles[i]
            if not ((-11 * math.pi / 12) - spec.TOLERANCE < a < (11 * math.pi / 12) + spec.TOLERANCE):
                # internal angle tighter than 15 degrees
                return False
        return True


    def test_length_constraints(self,config, spec):
        # return true for pass, false for fail
        for i in range(spec.num_segments):
            if config.lengths[i] < spec.min_lengths[i] - spec.TOLERANCE or \
                    config.lengths[i] > spec.max_lengths[i] + spec.TOLERANCE:
                return False
        return True


    def point_is_close(self,x1, y1, x2, y2, tolerance):
        return abs(x2 - x1) + abs(y2 - y1) < tolerance


    def test_grapple_point_constraint(self,config, spec):
        # return true for pass, false for fail
        ee1x, ee1y = config.points[0]
        ee2x, ee2y = config.points[-1]
        for gpx, gpy in spec.grapple_points:
            if (self.point_is_close(ee1x, ee1y, gpx, gpy, spec.TOLERANCE) or
                self.point_is_close(ee2x, ee2y, gpx, gpy, spec.TOLERANCE)) and \
                    not self.point_is_close(ee1x, ee1y, ee2x, ee2y, spec.TOLERANCE):
                return True
        return False


    def test_self_collision(self,config, spec):
        # return true for pass, false for fail
        if spec.num_segments < 3:
            # collision impossible with less than 3 segments
            return True
        # do full check
        for i in range(spec.num_segments - 1):
            p1 = config.points[i]
            q1 = config.points[i+1]

            for j in range(i + 2, spec.num_segments):
                p2 = config.points[j]
                q2 = config.points[j+1]

                if self.test_line_collision((p1, q1), (p2, q2)):
                    return False

        return True



    def get_obstacles(self,spec):
        """
        This method should only be used by tester. To avoid unexpected errors in your solution caused by floating point
        noise, you should not use this method in your solver.
        """
        # shrink all obstacles by TOLERANCE
        obstacles = []
        for o in spec.obstacles:
            obstacles.append(o)
        return obstacles


    def test_obstacle_collision(self,config, spec, obstacles):
        # return true for pass, false for fail
        for i in range(spec.num_segments):
            p = config.points[i]
            q = config.points[i+1]
            for o in obstacles:
                # bounding box check
                if not self.test_bounding_box(p, q, (o.x1, o.y1), (o.x2, o.y2)):
                    continue

                # full edge check
                for e in o.edges:
                    if self.test_line_collision((p, q), e):
                        # collision between robot segment and obstacle edge
                        return False
        return True


    def test_config_equality(self,c1, c2, spec):
        """
        Check for equality between robot config objects.
        :param other: object for comparison
        :return: True if equal (i.e. all points match), false otherwise
        """
        if not isinstance(c1, RobotConfig) or not isinstance(c2, RobotConfig):
            return False
        for i in range(spec.num_segments + 1):
            if not point_is_close(c1.points[i][0], c1.points[i][1], c2.points[i][0], c2.points[i][1], spec.TOLERANCE):
                return False
        for i in range(spec.num_segments):
            if abs(c2.lengths[i] - c1.lengths[i]) > spec.TOLERANCE:
                return False
        return True


    def test_config_distance(self,c1, c2, spec, tau):
        # return maximum distance between 2 configurations
        max_delta = 0

        max_ee1_delta = 0
        max_ee2_delta = 0
        for i in range(spec.num_segments):
            if abs((c2.ee1_angles[i] - c1.ee1_angles[i]).in_radians()) > max_ee1_delta:
                max_ee1_delta = abs((c2.ee1_angles[i] - c1.ee1_angles[i]).in_radians())

            if abs((c2.ee2_angles[i] - c1.ee2_angles[i]).in_radians()) > max_ee2_delta:
                max_ee2_delta = abs((c2.ee2_angles[i] - c1.ee2_angles[i]).in_radians())

        # measure leniently - allow compliance from EE1 or EE2
        max_delta = min(max_ee1_delta, max_ee2_delta)

        for i in range(spec.num_segments):
            if abs(c2.lengths[i] - c1.lengths[i]) > max_delta:
                max_delta = abs(c2.lengths[i] - c1.lengths[i])

        if (max_delta > spec.PRIMITIVE_STEP + spec.TOLERANCE) and (max_delta > tau):
            return False
        return True

    def test_config_distance_tau(self,c1, c2, spec, tau):
        # return maximum distance between 2 configurations
        max_delta = 0

        max_ee1_delta = 0
        max_ee2_delta = 0
        for i in range(spec.num_segments):
            if abs((c2.ee1_angles[i] - c1.ee1_angles[i]).in_radians()) > max_ee1_delta:
                max_ee1_delta = abs((c2.ee1_angles[i] - c1.ee1_angles[i]).in_radians())

            if abs((c2.ee2_angles[i] - c1.ee2_angles[i]).in_radians()) > max_ee2_delta:
                max_ee2_delta = abs((c2.ee2_angles[i] - c1.ee2_angles[i]).in_radians())

        # measure leniently - allow compliance from EE1 or EE2
        max_delta = min(max_ee1_delta, max_ee2_delta)

        for i in range(spec.num_segments):
            if abs(c2.lengths[i] - c1.lengths[i]) > max_delta:
                max_delta = abs(c2.lengths[i] - c1.lengths[i])

        if (max_delta > tau):
            return False
        return True

    def test_bounding_box_collision_4sampling(self,config, spec, obstacles):
        # return true for collision, false for pass
        for i in range(spec.num_segments):
            p = config.points[i]
            q = config.points[i+1]
            for o in obstacles:
                # bounding box check
                # true->collision, false->ok
                if self.test_bounding_box(p, q, (o.x1, o.y1), (o.x2, o.y2)):
                    return True                
                else:
                    continue
        return False

    def collision_test(self, m, q, tau):
        spec = self.spec
        obstacles = self.get_obstacles(spec)
        if self.test_config_distance_tau(m, q, spec, tau) and \
        self.test_obstacle_collision(q, spec, obstacles) and \
        self.test_environment_bounds(q) and \
        self.test_angle_constraints(q, spec) and \
        self.test_length_constraints(q, spec) and \
        self.test_grapple_point_constraint(q, spec) and \
        self.test_self_collision(q, spec):
            return True
        else:
            return False

    def self_collision_test(self, q):
        spec = self.spec
        obstacles = self.get_obstacles(spec)
        if self.test_obstacle_collision(q, spec, obstacles) and \
        self.test_environment_bounds(q) and \
        self.test_angle_constraints(q, spec) and \
        self.test_length_constraints(q, spec) and \
        self.test_grapple_point_constraint(q, spec) and \
        self.test_self_collision(q, spec):
            return True
        else:
            return False

    # True -> pass, False -> fail
    def self_obstacle_test(self,q):
        spec = self.spec
        obstacles = self.get_obstacles(spec)
        if self.test_obstacle_collision(q, spec, obstacles):
            return True
        else:
            return False

        # True -> collision, False -> pass
    def self_bounding_collision_test(self,q):
        spec = self.spec
        obstacles = self.get_obstacles(spec)
        # return true for collision, false for pass
        if self.test_bounding_box_collision_4sampling(q, spec, obstacles):
            return True
        else:
            return False
        

    

