

class Obstacle:
    """
    Class representing a rectangular obstacle. You may add to this class if you wish, but you should not modify the
    existing functions or variable names.

    COMP3702 2019 Assignment 2 Support Code

    Last updated by njc 24/08/19
    """

    def __init__(self, x1, y1, x2, y2):
        self.x1 = x1
        self.y1 = y1
        self.x2 = x2
        self.y2 = y2
        assert x1 < x2, "For a valid obstacle, mush have x1 < x2"
        assert y1 < y2, "For a valid obstacle, mush have y1 < y2"

        self.corners = [(x1, y1), (x1, y2), (x2, y2), (x2, y1)]
        self.edges = [(self.corners[i], self.corners[(i + 1) % 4]) for i in range(4)]

    def check_in_obstacle(self, x,y):
        if x >= self.x1 and x <= self.x2 and y >= self.y1 and y <= self.y2:
            return True
        else:
            return False

    def check_in_obstacle_range(self, x,y):
        if x >= (self.x1-0.05) and x <= (self.x2+0.05) and y >= (self.y1-0.05) and y <= (self.y2+0.05):
            return True
        else:
            return False

    def diff_from_obstacle(self, x,y):
        return abs(x - self.x1) + abs(x - self.x2) + abs(y - self.y1) + abs(y - self.y2) 
