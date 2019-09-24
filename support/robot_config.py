import math
from support.angle import Angle

class RobotConfig:
    """
    Class representing a configuration of the robot. You may add to this class if you wish, but you should not modify
    the existing functions or variable names.

    COMP3702 2019 Assignment 2 Support Code

    Last updated by njc 24/08/19
    """

    def __init__(self, lengths, ee1x=None, ee1y=None, ee1_angles=None, ee2x=None, ee2y=None, ee2_angles=None,
                 ee1_grappled=False, ee2_grappled=False):
        """
        Constructor for RobotConfig - we suggest using make_robot_config_from_ee1() or make_robot_config_from_ee2()
        to construct new instances of RobotConfig rather than calling this function directly.
        """
        self.lengths = lengths
        self.ee1_grappled = ee1_grappled
        self.ee2_grappled = ee2_grappled
        if ee1x is not None and ee1y is not None and ee1_angles is not None:
            points = [(ee1x, ee1y)]
            net_angle = Angle(radians=0)
            for i in range(len(ee1_angles)):
                x, y = points[-1]
                net_angle = net_angle + ee1_angles[i]
                x_new = x + (lengths[i] * math.cos(net_angle.in_radians()))
                y_new = y + (lengths[i] * math.sin(net_angle.in_radians()))
                points.append((x_new, y_new))

            self.ee1_angles = ee1_angles
            # 1st angle is last angle of e1_angles + pi, others are all -1 * e1_angles (in reverse order)
            self.ee2_angles = [math.pi + net_angle] + \
                              [-ee1_angles[i] for i in range(len(ee1_angles) - 1, 0, -1)]
            self.points = points

        elif ee2x is not None and ee2y is not None and ee2_angles is not None:
            points = [(ee2x, ee2y)]
            net_angle = Angle(radians=0)
            for i in range(len(ee2_angles)):
                x, y = points[0]
                net_angle = net_angle + ee2_angles[i]
                x_new = x + (lengths[-i - 1] * math.cos(net_angle.in_radians()))
                y_new = y + (lengths[-i - 1] * math.sin(net_angle.in_radians()))
                points.insert(0, (x_new, y_new))

            # 1st angle is last angle of e2_angles + pi, others are all -1 * e2_angles (in reverse order)
            self.ee1_angles = [math.pi + sum(ee2_angles)] + \
                              [-ee2_angles[i] for i in range(len(ee2_angles) - 1, 0, -1)]
            self.ee2_angles = ee2_angles
            self.points = points

        else:
            raise Exception("Could not create RobotConfig - Insufficient information given")

    # def __str__(self):
    #     """
    #     Output string representation of RobotConfig. Use this functionality by calling str() on any RobotConfig object.

    #     Note: Angles are printed as degrees (but are internally stored as radians)
    #     :return: "ee1x ee2x; ee1_angle_1 ee1_angle_2 ... ee1_angle_n; length_1 length_2 ... length_n"
    #     """
    #     # add ee1 position
    #     s = str(round(self.points[0][0], 8)) + ' ' + str(round(self.points[0][1], 8)) + '; '
    #     # add angles from ee1
    #     s += ' '.join([str(round(a.in_degrees(), 8)) for a in self.ee1_angles]) + '; '
    #     # add lengths
    #     s += ' '.join([str(round(l, 8)) for l in self.lengths]) + '; '
    #     if self.ee1_grappled:
    #         s += str(1)
    #     else:
    #         s += str(2)
    #     return s
    
    def strEE1out(self):
        # add ee1 position
        s = str(round(self.points[0][0], 8)) + ' ' + str(round(self.points[0][1], 8)) + '; '
        # add angles from ee1
        s += ' '.join([str(round(a.in_degrees(), 8)) for a in self.ee1_angles]) + '; '
        # add lengths
        s += ' '.join([str(round(l, 8)) for l in self.lengths]) + '; '
        if self.ee1_grappled:
            s += str(1)
        else:
            s += str(2)
        return s


    def __str__(self):
        """
        Output string representation of RobotConfig. Use this functionality by calling str() on any RobotConfig object.

        Note: Angles are printed as degrees (but are internally stored as radians)
        :return: "ee1x ee2x; ee1_angle_1 ee1_angle_2 ... ee1_angle_n; length_1 length_2 ... length_n"
        """
        if self.ee1_grappled == True or (self.ee1_grappled == False and self.ee2_grappled == False):
            # add ee1 position
            s = str(round(self.points[0][0], 8)) + ' ' + str(round(self.points[0][1], 8)) + '; '
            # add angles from ee1
            s += ' '.join([str(round(a.in_degrees(), 8)) for a in self.ee1_angles]) + '; '
            # add lengths
            s += ' '.join([str(round(l, 8)) for l in self.lengths]) + '; '
            if self.ee1_grappled:
                s += str(1)
            else:
                s += str(2)
            return s
        else:
            # add ee2 position
            s = str(round(self.points[-1][0], 8)) + ' ' + str(round(self.points[-1][1], 8)) + '; '
            # add angles from ee1
            s += ' '.join([str(round(a.in_degrees(), 8)) for a in self.ee2_angles]) + '; '
            # add lengths
            s += ' '.join([str(round(l, 8)) for l in self.lengths]) + '; '
            if self.ee1_grappled:
                s += str(1)
            else:
                s += str(2)
            return s

    def str2list(self):
        if self.ee1_grappled:
            arr  = []
            arr.append(self.points[0][0])
            arr.append(self.points[0][1])
            for i in self.ee1_angles:
                arr.append(float(str(round(i.in_degrees(), 8))))
            for i in self.lengths:
                arr.append(float(str(round(i, 8))))
            if self.ee1_grappled:
                arr.append(1)
            else:
                arr.append(2)
            return arr
    # def str2list_ee2(self):    
        else:
            arr  = []
            arr.append(self.points[-1][0])
            arr.append(self.points[-1][1])
            for i in self.ee2_angles:
                arr.append(float(str(round(i.in_degrees(), 8))))
            for i in self.lengths:
                arr.append(float(str(round(i, 8))))
            if self.ee1_grappled:
                arr.append(1)
            else:
                arr.append(2)
            return arr


    def get_HeadeePos(self):
        if self.ee1_grappled:
            return self.points[0]
        else:
            return self.points[-1]            


    def get_EndeePos(self):
        if self.ee1_grappled:
            return self.points[-1]
        else:
            return self.points[0]  

    def get_position(self):
        if self.ee1_grappled:
            return self.points
        else:
            arr = []
            for i in reversed(self.points):
                arr.append(i)
            return arr
        

    def get_ee1(self):
        """
        Return the position of end effector 1.
        :return: (ee1x, ee1y)
        """
        return self.points[0]



    def get_ee2(self):
        """
        Return the position of end effector 2.
        :return: (ee2x, ee2y)
        """
        return self.points[-1]

    def get_angle(self):
        if self.ee1_grappled:
            return self.ee1_angles
        else:
            return self.ee2_angles

    def get_length(self):
        return self.lengths

    def get_grapple_ee1(self):
        if (self.ee1_grappled):
            return 1
        else:
            return 2

def make_robot_config_from_ee1(x, y, angles, lengths, ee1_grappled=False, ee2_grappled=False):
    """
    Create a robot configuration from the position of end effector 1, with angles relative to end effector 1.
    :param x: horizontal position of end 1
    :param y: vertical position of end 1
    :param angles: list angles of each joint starting from end 1 (in radians)
    :param lengths: list of lengths of each segment
    :param ee1_grappled: whether the robot is grappled at end 1
    :param ee2_grappled: whether the robot is grappled at end 2
    :return: RobotConfig instance
    """
    return RobotConfig(lengths, ee1x=x, ee1y=y, ee1_angles=angles, ee1_grappled=ee1_grappled, ee2_grappled=ee2_grappled)


def make_robot_config_from_ee2(x, y, angles, lengths, ee1_grappled=False, ee2_grappled=False):
    """
    Create a robot configuration from the position of end effector 2, with angles relative to end effector 2.
    :param x: horizontal position of end 2
    :param y: vertical position of end 2
    :param angles: list angles of each joint starting from end 2 (in radians)
    :param lengths: list of lengths of each segment
    :param ee1_grappled: whether the robot is grappled at end 1
    :param ee2_grappled: whether the robot is grappled at end 2
    :return: RobotConfig instance
    """
    return RobotConfig(lengths, ee2x=x, ee2y=y, ee2_angles=angles, ee1_grappled=ee1_grappled, ee2_grappled=ee2_grappled)


def write_robot_config_list_to_file(filename, robot_config_list):
    """
    Write an output file for the given list of RobotConfigs. We recommend using this method to generate your output
    file.
    :param filename: name of output file (e.g. 2nd argument given to program)
    :param robot_config_list: list of RobotConfig objects forming a path
    """
    f = open(filename, 'w')
    for rc in robot_config_list:
        f.write(str(rc) + '\n')
    f.close()

def make_robot_config_with_arr(x, y, anglesArr, lengthsArr, grappled):
    # grappled = 1-> ee1, 2 -> ee2
    angles = [Angle(degrees=float(i)) for i in anglesArr]
        
    if grappled == 1:
        return make_robot_config_from_ee1(x, y, angles, lengthsArr, ee1_grappled=True)
    else:
        return make_robot_config_from_ee2(x, y, angles, lengthsArr, ee2_grappled=True)

