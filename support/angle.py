import math


class Angle:
    """
    Class representing an angle. Behaves like a normal floating point number, supporting addition, subtraction,
    negation, equality and comparison (not multiplication). Constructor accepts degrees or radians, and value can be
    accessed as degrees or radians. Automatically keeps value in the range of -pi and pi.

    COMP3702 2019 Assignment 2 Support Code

    Last updated by njc 01/09/19
    """

    def __init__(self, radians=None, degrees=None):
        if radians is None:
            radians = degrees * math.pi / 180
        # get in range (-pi, pi)
        if radians > 0:
            while radians > math.pi:
                radians -= 2 * math.pi
        else:
            while radians < -math.pi:
                radians += 2 * math.pi
        self.radians = radians

    def in_radians(self):
        return self.radians

    def in_degrees(self):
        return self.radians * 180 / math.pi

    def __add__(self, other):
        if isinstance(other, Angle):
            radians = self.radians + other.radians
        elif isinstance(other, (int, float)):
            radians = self.radians + other
        else:
            raise Exception("+ operation between Angle and " + str(type) + " not supported.")
        # get in range (-pi, pi)
        if radians > 0:
            while radians > math.pi:
                radians -= 2 * math.pi
        else:
            while radians < -math.pi:
                radians += 2 * math.pi
        return Angle(radians=radians)

    def __sub__(self, other):
        if isinstance(other, Angle):
            radians = self.radians - other.radians
        elif isinstance(other, (int, float)):
            radians = self.radians - other
        else:
            raise Exception("- operation between Angle and " + str(type) + " not supported.")
        # get in range (-pi, pi)
        if radians > 0:
            while radians > math.pi:
                radians -= 2 * math.pi
        else:
            while radians < -math.pi:
                radians += 2 * math.pi
        return Angle(radians=radians)

    def __radd__(self, other):
        if isinstance(other, Angle):
            radians = self.radians + other.radians
        elif isinstance(other, (int, float)):
            radians = self.radians + other
        else:
            raise Exception("Reverse Add operation between Angle and " + str(type) + " not supported.")
        # get in range (-pi, pi)
        if radians > 0:
            while radians > math.pi:
                radians -= 2 * math.pi
        else:
            while radians < -math.pi:
                radians += 2 * math.pi
        return Angle(radians=radians)

    def __neg__(self):
        return Angle(radians=-self.radians)

    def __eq__(self, other):
        if isinstance(other, Angle):
            return abs(self.radians - other.radians) < 1e-8
        elif isinstance(other, (int, float)):
            return abs(self.radians - other) < 1e-8
        else:
            raise Exception("== operation between Angle and " + str(type) + " not supported.")

    def __ne__(self, other):
        if isinstance(other, Angle):
            return abs(self.radians - other.radians) > 1e-8
        elif isinstance(other, (int, float)):
            return abs(self.radians - other) > 1e-8
        else:
            raise Exception("!= operation between Angle and " + str(type) + " not supported.")

    def __lt__(self, other):
        if isinstance(other, Angle):
            return self.radians < other.radians
        elif isinstance(other, (int, float)):
            return self.radians < other
        else:
            raise Exception("< operation between Angle and " + str(type) + " not supported.")

    def __le__(self, other):
        if isinstance(other, Angle):
            return self.radians <= other.radians
        elif isinstance(other, (int, float)):
            return self.radians <= other
        else:
            raise Exception("<= operation between Angle and " + str(type) + " not supported.")

    def __gt__(self, other):
        if isinstance(other, Angle):
            return self.radians > other.radians
        elif isinstance(other, (int, float)):
            return self.radians > other
        else:
            raise Exception("> operation between Angle and " + str(type) + " not supported.")

    def __ge__(self, other):
        if isinstance(other, Angle):
            return self.radians >= other.radians
        elif isinstance(other, (int, float)):
            return self.radians >= other
        else:
            raise Exception(">= operation between Angle and " + str(type) + " not supported.")

    def __str__(self):
        return str(round(self.radians * 180 / math.pi, 8))

    def __hash__(self):
        return hash(self.radians)

