import numpy as np
from test_robot import test_robot
from support.angle import Angle
# from support.robot_config import RobotConfigs
# from support.robot_config import make_robot_config_with_arr 

import math

class Interpolation:
    
    def __init__(self,wholepath):
        self.allv = wholepath
        
    def run_Interpolate(self):
        robots = []
        stepslength = 1e-3
        
        for i in range(len(self.allv)-1):
            robot1 = self.Coefstr2arr(self.allv[i])
            robot2 = self.Coefstr2arr(self.allv[i+1])
            arr = self.interpolation2robOne(robot1,robot2,stepslength)
            if i == 0:
                wholeArray = arr
            else:
                wholeArray = np.vstack([wholeArray, arr])
            
        numSeg = int((len(robot2)-2)/2)
        for j in range(numSeg):
            tmpAng = Angle(radians=robot2[j+2])
            robot2[2+j] = tmpAng.in_degrees()
            
        wholeArray = np.vstack([wholeArray, np.asarray(robot2)])
        return wholeArray

    # rob1: list with px,py,ang,length
    def interpolation2robOne(self,rob1,rob2, stepslength):
        # stepslength = 1e-3
        # if (rob1[0]-rob2[0] != 0 and rob1[1]-rob2[1] != 0):
        #     return rob1
        rob1 = np.asarray(rob1)
        rob2 = np.asarray(rob2)
        
        numSeg = int((len(rob1)-2)/2)

        diff = abs(rob1 - rob2)
        step = (np.floor(diff/stepslength))
        step = step.astype(int)
        maxStep = max(step)
        stepslengthlist = diff/maxStep
        for i in range(len(stepslengthlist)):
            stepslength = stepslengthlist[i]
            stepslength = (-1)*stepslength if rob1[i] > rob2[i] else stepslength
            arr = []
            if (step[i] ==0):
                for j in range(maxStep):
                    if i > 1 and i<2+numSeg:
                        tmpAng = Angle(radians=rob1[i])
                        arr.append(tmpAng.in_degrees())
                    else:
                        arr.append(rob1[i])
            else:
                for j in range(maxStep):
                    if i>1 and i<2+numSeg:
                        tmpAng = Angle(radians=(rob1[i]+ j * stepslength))
                        arr.append(tmpAng.in_degrees())
                    else:
                        arr.append(rob1[i] + j * stepslength)
            if i == 0:
                whole = np.asarray(arr)
            else:
                whole = np.vstack([whole, arr])
        r,c = whole.shape
        whole = np.transpose(whole)
        # whole = self.interpolation2robAddxy(rob1,whole)
        return whole

    def interpolation2robAddxy(self,rob1,myarray):
        r,c = myarray.shape
        xyArr = np.kron(np.ones((1,c)), rob1[0])
        xyArr = np.vstack([xyArr, np.kron(np.ones((1,c)), rob1[1])])
        xyArr = np.vstack([xyArr, myarray])
        xyArr = np.transpose(xyArr)
        return xyArr

    def Coefstr2arr(self,stringInput):
        ee1_xy_str, ee1_angles_str, lengths_str, ee1_grappled = stringInput.strip().split(';')
        ee1x, ee1y = tuple([float(i) for i in ee1_xy_str.split(' ')])
        ee1_angles = [Angle(degrees=float(i)) for i in ee1_angles_str.strip().split(' ')]
        lengths = [float(i) for i in lengths_str.strip().split(' ')]
        output = []
        output.append(ee1x)
        output.append(ee1y)
        for i in ee1_angles:
            # output.append(i.in_degrees())
            output.append(i.in_radians())
        for i in lengths:
            output.append(i)
        
        # output.append(int(ee1_grappled))
        return output


# def main():
#     file = './testcases/3g1_m1.txt'
#     prm = PRM(file)    
#     aa = test_robot(prm)
#     qq = aa.load_output('output.txt')
#     aa = []
#     for i in qq:
#         aa.append(str(i))
#     # print(aa)
#     gginterpolat = Interpolation(aa)
#     gg = gginterpolat.run_Interpolate()
#     write_sampling_config('interpolation.txt',3,gg)

# if __name__ == '__main__':
#     main()