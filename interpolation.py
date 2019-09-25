import numpy as np
from test_robot import test_robot
from support.angle import Angle

# from support.robot_config import RobotConfigs
# from support.robot_config import make_robot_config_with_arr 

import math

class Interpolation:
    
    def __init__(self,wholepath,ee1flag):
        self.allv = wholepath
        self.ee1flag = ee1flag
        
    def run_Interpolate(self):
        robots = []
        stepslength = 1e-3
        # stepslength = 0.00001
        for i in range(len(self.allv)-1):
            robot1 = self.Coefstr2arr(self.allv[i])
            robot2 = self.Coefstr2arr(self.allv[i+1])

            if self.ee1flag[i] == 2 and self.ee1flag[i+1] == 2:
                ee2rob1 = self.ee1toee2(robot1)
                ee2rob2 = self.ee1toee2(robot2)
                arr = self.interpolation2robOne_radian(ee2rob1,ee2rob2,stepslength)
                for i in range(len(arr)):
                    arr[i] = self.ee2toee1(arr[i])
                
                # tmp,pt = self.ee1toee2(robot2)
                # tmp1,pt1 = self.ee2toee1(tmp)
                # print(robot2)
                # print(tmp)
                # print(tmp1)
                # qq =1
            else:
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
        step = (np.ceil(diff/stepslength))
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
                        ang = round(tmpAng.in_degrees(), 8)
                        arr.append(ang)
                    else:
                        tmp = round(rob1[i] + j * stepslength, 8)
                        arr.append(tmp)
            if i == 0:
                whole = np.asarray(arr)
            else:
                whole = np.vstack([whole, arr])
        whole = np.transpose(whole)
        # whole = self.interpolation2robAddxy(rob1,whole)
        return whole
        
    def interpolation2robOne_radian(self,rob1,rob2, stepslength):
        # stepslength = 1e-3
        # if (rob1[0]-rob2[0] != 0 and rob1[1]-rob2[1] != 0):
        #     return rob1
        rob1 = np.asarray(rob1)
        rob2 = np.asarray(rob2)
        
        numSeg = int((len(rob1)-2)/2)

        diff = abs(rob1 - rob2)
        step = (np.ceil(diff/stepslength))
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
                        arr.append(tmpAng.in_radians())
                    else:
                        arr.append(rob1[i])
            else:
                for j in range(maxStep):
                    if i>1 and i<2+numSeg:
                        tmpAng = Angle(radians=(rob1[i]+ j * stepslength))
                        ang = tmpAng.in_radians()
                        arr.append(ang)
                    else:
                        tmp = rob1[i] + j * stepslength
                        arr.append(tmp)
            if i == 0:
                whole = np.asarray(arr)
            else:
                whole = np.vstack([whole, arr])
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
    
    def ee1toee2(self,arr):
        numSeg = int((len(arr)-2)/2)
        ee1x = arr[0]
        ee1y = arr[1]
        points = [(ee1x, ee1y)]
        lengths = arr[(2+numSeg):(2+numSeg*2)]
        ee1_angles = []
        for i in arr[2:(2+numSeg)]:
            ee1_angles.append(Angle(radians=i))

        net_angle = Angle(radians=0)
        for i in range(len(ee1_angles)):
            x, y = points[-1]
            net_angle = net_angle + ee1_angles[i]
            x_new = x + (lengths[i] * math.cos(net_angle.in_radians()))
            y_new = y + (lengths[i] * math.sin(net_angle.in_radians()))
            points.append((x_new, y_new))

        # 1st angle is last angle of e2_angles + pi, others are all -1 * e2_angles (in reverse order)
        ee2_angles = [math.pi + sum(ee1_angles)] + \
                            [-ee1_angles[i] for i in range(len(ee1_angles) - 1, 0, -1)]
        output= []
        output.append(round(points[-1][0],8))
        output.append(round(points[-1][1],8))
        for i in range(len(ee2_angles)):
            output.append(ee2_angles[i].in_radians())
            # output.append(round(ee2_angles[i].in_radians(),8))
        for i in range(len(ee2_angles)):
            output.append(arr[(2+numSeg)+i])
            # output.append(round(arr[(2+numSeg)+1],8))
        return output

    def ee2toee1(self,arr):
        numSeg = int((len(arr)-2)/2)
        ee1x = arr[0]
        ee1y = arr[1]
        points = [(ee1x, ee1y)]
        lengths = arr[(2+numSeg):(2+numSeg*2)]
        ee2_angles = []
        for i in arr[2:(2+numSeg)]:
            ee2_angles.append(Angle(radians=i))

        net_angle = Angle(radians=0)
        for i in range(len(ee2_angles)):
            x, y = points[0]
            net_angle = net_angle + ee2_angles[i]
            x_new = x + (lengths[-i - 1] * math.cos(net_angle.in_radians()))
            y_new = y + (lengths[-i - 1] * math.sin(net_angle.in_radians()))
            points.insert(0, (x_new, y_new))

        # 1st angle is last angle of e2_angles + pi, others are all -1 * e2_angles (in reverse order)
        ee1_angles = [math.pi + sum(ee2_angles)] + \
                            [-ee2_angles[i] for i in range(len(ee2_angles) - 1, 0, -1)]
        output= []
        output.append(round(points[0][0],8))
        output.append(round(points[0][1],8))
        for i in range(len(ee1_angles)):
            output.append(ee1_angles[i].in_degrees())
            # output.append(round(ee1_angles[i].in_radians(),8))
        for i in range(len(ee1_angles)):
            output.append(arr[(2+numSeg)+i])
            # output.append(round(arr[(2+numSeg)+i],8))
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