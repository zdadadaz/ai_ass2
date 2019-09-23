from support.problem_spec import ProblemSpec
from support.robot_config import RobotConfig
from support.robot_config import make_robot_config_from_ee1 as rob_conf_ee1
from support.robot_config import make_robot_config_from_ee2 as rob_conf_ee2
from support.robot_config import make_robot_config_with_arr 
from support.robot_config import write_robot_config_list_to_file
from support.angle import Angle
from shortestPath import astar

from test_robot import test_robot
from util import Graph,Vertex

import random
from visualiser import Visualiser 
import numpy as np
import math



class EST(ProblemSpec):
    def __init__(self, input_file):
        super(EST,self).__init__(input_file)
        self.gInit = Graph()
        self.gGoal = Graph()
        np.random.seed(30)
        # random.seed(27492)

#   False-> no collision, True -> have collision
#   input A,B array
    def collision_check(self,A,B, n,checkList):
        if n ==0:
            return False
        # check in the same ee grapple 
        if (A[0] != B[0] or A[1] != B[1] or A[-1] != B[-1]):
            return False
        tester = test_robot(self)
        mid = (A+B)/2
        mid[-1] = A[-1]
        midRobot = make_robot_config_with_arr(A[0],A[1],mid[2:(2+self.num_segments)],mid[(2+self.num_segments):(2+2*self.num_segments)],A[-1])
        # print(midRobot)
        checkList.append(str(midRobot))
        if not tester.self_obstacle_env_test(midRobot):
            return True # have collision
        self.collision_check(A  ,mid, n-1,checkList)
        self.collision_check(mid,B, n-1,checkList)
    
    def sampling_eexy(self,eexy,numSampling,ee1Flag):
        # np.random.seed(249)
        minMax = self.get_min_max_len()
        numSeg = self.get_num_segment()

        if ee1Flag:
            grapples_tmp = np.zeros((numSampling,1))
        else:
            grapples_tmp = np.ones((numSampling,1))

        grapples = []
        angles = np.zeros((numSampling,numSeg))
        lengths = np.zeros((numSampling,numSeg))
        for i in range(numSeg):
            if i == 0:
                angles[:,i] = np.random.rand(1,numSampling)*360 - 180
            else:
                angles[:,i] = np.random.rand(1,numSampling)*330 - 165
            tmp = np.random.rand(1,numSampling) * (minMax[1][i] - minMax[0][i])
            lengths[:,i] = tmp + minMax[0][i] 
        for i in range(numSampling):
            grapples.append(list(eexy))
        output = np.append(np.asarray(grapples), angles, axis=1)
        output = np.append(output, lengths, axis=1)
        output = np.append(output,grapples_tmp,axis=1)
        return output

    def sampling(self, numSampling):
        # np.random.seed(30)
        minMax = self.get_min_max_len()
        numSeg = self.get_num_segment()
        grapple_point = self.get_grapple_points()

        grapples_tmp = (np.floor(np.random.rand(1,numSampling)*len(grapple_point)))
        grapples = []
        angles = np.zeros((numSampling,numSeg))
        lengths = np.zeros((numSampling,numSeg))
        for i in range(numSeg):
            if i == 0:
                # angles[:,i] = np.random.rand(1,numSampling)*360 - 180
                angles[:,i] = np.random.rand(1,numSampling)*180
            else:
                angles[:,i] = np.random.rand(1,numSampling)*330 - 165
            tmp = np.random.rand(1,numSampling) * (minMax[1][i] - minMax[0][i])
            lengths[:,i] = tmp + minMax[0][i] 
        for i in range(numSampling):
            grapples.append(list(grapple_point[int(grapples_tmp[0][i])]))
        output = np.append(np.asarray(grapples), angles, axis=1)
        output = np.append(output, lengths, axis=1)
        grapples_tmp = grapples_tmp.reshape(numSampling,1)
        output = np.append(output,grapples_tmp,axis=1)
        # print(output)
        return output
    
    def bridgingCases(self):
        angConstraint=[]
        points=[]
        flag = True
        if self.num_grapple_points == 2 and self.num_segments==3:
            points = self.grapple_points
            xdiff = points[0][0] - points[1][0]
            ydiff = points[0][1] - points[1][1]
            angConstraint = [[0, 90],[0,-90],[0,-90]]
            flag = True
            return angConstraint,points,flag
        return angConstraint,points,flag
        
    def sampling_ang_constrain(self,numSampling,angConstraint,eexy,ee1Flag):
        minMax = self.get_min_max_len()
        numSeg = self.get_num_segment()

        if ee1Flag:
            grapples_tmp = np.zeros((numSampling,1))
        else:
            grapples_tmp = np.ones((numSampling,1))

        grapples = []
        angles = np.zeros((numSampling,numSeg))
        lengths = np.zeros((numSampling,numSeg))
        for i in range(numSeg):
            if i == 0:
                # horizontal can be 0 ~ 180
                # vertical can be -90 ~ 90
                angles[:,i] = np.random.rand(1,numSampling)*(angConstraint[i][1]-angConstraint[i][0]) + angConstraint[i][0]
                angles[angles>180] = 180
                angles[angles<-180] = -180
            else:
                angles[:,i] = np.random.rand(1,numSampling)*(angConstraint[i][1]-angConstraint[i][0]) + angConstraint[i][0]
                angles[angles>165] = 165
                angles[angles<-165] = -165
            tmp = np.random.rand(1,numSampling) * (minMax[1][i] - minMax[0][i])
            lengths[:,i] = tmp + minMax[0][i] 
        for i in range(numSampling):
            grapples.append(list(eexy))
        output = np.append(np.asarray(grapples), angles, axis=1)
        output = np.append(output, lengths, axis=1)
        output = np.append(output,grapples_tmp,axis=1)
        return output

    def sampling_bridge(self):
        numberSamples = 10000
        angConstraint,eexy,flag = self.bridgingCases()
        for p in range(len(eexy)-1):
            samples = self.sampling_ang_constrain(numberSamples,angConstraint,eexy[p],flag)
            for i in range(numberSamples):
                rob = self.assign_config(samples,i)
                arr = self.run_checking_sampling_bridge(rob,eexy[p],eexy[p+1],flag)
                print(arr)
        return arr

    # rob from grapple1 to grapple2
    def run_checking_sampling_bridge(self,rob,grapple1,grapple2,ee1flag):
        tolerate_error= 1e-5
        minMax = self.get_min_max_len()
        numSeg = self.get_num_segment()
        robPos = rob.points
        headPos = robPos[0]
        endPos = rob.get_EndeePos()
    
        arr = []
        con = True

        while con:
            flag, arr = self.check_sampling_bridge(rob,grapple1,grapple2,ee1flag)
            if flag:
                return arr
            else:
                # print(rob)
                # print(robPos)
                xlen = (robPos[-2][0] - grapple2[0])
                ylen = (robPos[-2][1] - grapple2[1])
                difflen = math.sqrt(xlen * xlen + ylen * ylen)
                if (difflen >= minMax[0][-1] and difflen <= minMax[1][-1]):
                    newAng = Angle.atan2(ylen,xlen)
                    if ee1flag:
                        robArr = rob.str2list()
                        robArr[2+numSeg-1] = newAng
                        robArr[2+numSeg*2 -1] = difflen
                        robNew= make_robot_config_with_arr(robArr[0],robArr[1],robArr[2:(2+numSeg)],robArr[(2+numSeg):(2+numSeg*2)],robArr[-1]) 
                        flagNew, arrNew = self.check_sampling_bridge(robNew,grapple1,grapple2,ee1flag)
                        if flagNew:
                            return arrNew
                    else:
                        #===== here might have problem when grapple is ee2 ====
                        robArr = rob.str2list()
                        robArr[2+numSeg-1] = newAng
                        robArr[2+numSeg*2 -1] = difflen
                        robNew= make_robot_config_with_arr(robArr[0],robArr[1],robArr[2:(2+numSeg)],robArr[(2+numSeg):(2+numSeg*2)],robArr[-1]) 
                        flagNew, arrNew = self.check_sampling_bridge(robNew,grapple1,grapple2,ee1flag)
                        if flagNew:
                            return arrNew
        return arr

    def check_sampling_bridge(self,rob,grapple1,grapple2 , ee1flag):
        tolerate_error= 1e-5
        minMax = self.get_min_max_len()
        numSeg = self.get_num_segment()
        robPos = rob.points
        headPos = robPos[0]
        endPos = rob.get_EndeePos()
    
        arr = []
        if ee1flag:
            if abs(endPos[0] - grapple2[0])< tolerate_error and abs(endPos[1] - grapple2[1])< tolerate_error:
                robInverse = rob_conf_ee2(endPos[0],endPos[1],rob.ee2_angles,rob.lengths,ee2_grappled=True)
                arr.append(rob)
                arr.append(robInverse)
                return True,arr
        else:
            if abs(endPos[0] - grapple1[0])< tolerate_error and abs(endPos[1] - grapple1[1])< tolerate_error:
                robInverse = rob_conf_ee1(endPos[0],endPos[1],rob.ee1_angles,rob.lengths,ee1_grappled=True)
                arr.append(rob)
                arr.append(robInverse)
                return True,arr
        return False,arr

    def sampling_withinD(self, robot,D, numSampling):
        minMax = self.get_min_max_len()
        numSeg = self.get_num_segment()
        grapple_point = self.get_grapple_points()

        limitAngle = Angle(radians=D[0])
        limitLength = D[1]
        # read robot angle and length
        robot_ang = robot.get_angle()
        robot_len = robot.get_length()

        # sampling angles
        angles = np.zeros((numSampling,numSeg))
        lengths = np.zeros((numSampling,numSeg))
        for i in range(numSeg):
            tmpAng = np.random.rand(1,numSampling)*limitAngle.in_degrees()*2 \
                    - limitAngle.in_degrees() +robot_ang[i].in_degrees()     
            if i == 0:
                tmpAng[tmpAng>180] = 180
                tmpAng[tmpAng<-180] = -180
                angles[:,i] = tmpAng
            else:
                tmpAng[tmpAng>165] = 165
                tmpAng[tmpAng<-165] = -165
                angles[:,i] = tmpAng
            # # sampling length
            if (minMax[1][i] - minMax[0][i]) != 0:
                tmp = np.random.rand(1,numSampling) * (D[1]*2)-D[1] + robot_len[i]
                tmp[tmp > minMax[1][i]] = minMax[1][i]
                tmp[tmp < minMax[0][i]] = minMax[0][i]
            else:
                tmp = np.zeros((1,numSampling)) + robot_len[i]
            lengths[:,i] = tmp

        # grapple
        grapples = []
        grapples_tmp = (np.floor(np.random.rand(1,numSampling)*len(grapple_point)))
        for i in range(numSampling):
            grapples.append(list(grapple_point[int(grapples_tmp[0][i])]))
        output = np.append(np.asarray(grapples), angles, axis=1)
        output = np.append(output, lengths, axis=1)
        grapples_tmp = grapples_tmp.reshape(numSampling,1)
        output = np.append(output,grapples_tmp,axis=1)
        
        return output

    def assign_config(self, sample_config, id):
        numSeg = self.get_num_segment()
        x = sample_config[id][0]
        y = sample_config[id][1]
        angles = []
        lengths = []
        for i in range(numSeg):
            angles.append(Angle(degrees=sample_config[id][2+i]) )
            lengths.append(sample_config[id][2+numSeg+i])
        if (int(sample_config[id][-1]) == 0):
            return rob_conf_ee1(x, y, angles, lengths,ee1_grappled=True)
        else:
            return rob_conf_ee2(x, y, angles, lengths,ee2_grappled=True)

    def str2robotConfig(self, stringInput):
        ee1_xy_str, ee1_angles_str, lengths_str, ee1_grappled = stringInput.strip().split(';')
        ee1x, ee1y = tuple([float(i) for i in ee1_xy_str.split(' ')])
        ee1_angles = [Angle(degrees=float(i)) for i in ee1_angles_str.strip().split(' ')]
        lengths = [float(i) for i in lengths_str.strip().split(' ')]
        if int(ee1_grappled) == 1:
            return rob_conf_ee1(ee1x, ee1y, ee1_angles, lengths, ee1_grappled=True)
        else: 
            return rob_conf_ee2(ee1x, ee1y, ee1_angles, lengths, ee2_grappled=True)

    
    def run_EST(self, outPath):
        init = self.get_init_state();
        goal = self.get_goal_state();

        self.gInit.addVertex(str(init))
        self.gGoal.addVertex(str(goal))
        s = 100000
        for i in range(s):
            added_m,flagInit = self.expandTree()
            connected,q  = self.connectTree(added_m,flagInit)
            # haven't checked path
            if connected:
                # print(added_m)
                print("traverse from middle point back to init and goal")
                robot_config_list = self.traverseBack(flagInit,added_m,q)
                for rc in range(len(robot_config_list)):
                    robot_config_list[rc] = robot_config_list[rc][:-3]
                write_robot_config_list_to_file(outPath,robot_config_list)
                return True
        return False
    
    # flagInit = 0 -> added_m in init, otherwise  added_m in goal
    def traverseBack(self,flagInit,added_m,q):
        if flagInit == 0:
            path1 = self.traverse(self.gInit, added_m,1)
            path2 = self.traverse(self.gGoal, q,0)
        else:
            path1 = self.traverse(self.gInit, q,1)
            path2 = self.traverse(self.gGoal, added_m,0)
        out = []
        out.extend(path1)
        out.extend(path2)
        return out
    
    def traverse(self, graph, node, reverseFlag):
        x = graph.getVertex(str(node))
        statelist = []
        while x.getParents() is not None:
            statelist.append(x.getId())
            aa = x.getParents()
            x = graph.getVertex(x.getParents().getId())
            # print(x.getParents())
        statelist.append(x.getId())
        if(reverseFlag == 1):
            arr = []
            for i in reversed(statelist):
                arr.append(i)
            return arr
        return statelist

    def expandTree(self):
        k = 1
        D = [0.4,1e-2]
        tau = 0.4
        pr = round(random.random())
        T = self.gGoal if pr == 1 else self.gInit
        tester = test_robot(self)
        
        while True:
            prVertex = math.floor(random.random() * T.getNumbVertices())
            mVertex = T.getVertex(T.getVerticeByInt(prVertex))
            m = self.str2robotConfig(mVertex.getId())
            # m need to become robot
            robot_conf = self.sampling_withinD(m,D,k)
            for i in range(k):
                # print(robot_conf[i])
                q = self.assign_config(robot_conf,i)
                # check q is collision or not
                if(tester.self_collision_test(q)) and tester.test_config_distance_tau(m,q,self,tau):
                    print(str(q)[:-3])
                    f = open("tmp_output.txt", "a")
                    f.write(str(q)[:-3] + '\n')
                    f.close()
                    T.addEdge(str(m),str(q))
                    return q,pr
                # else: 
                #     output = self.obstacle_sampling(T,m,q,D,k,tester,tau)
                #     if output is not None:
                #         return output,pr
        

    def obstacle_sampling(self,T,m,q,D,k,tester,tau):
        q_test = self.sampling_withinD(q,D,k)
        for i in range(k):
            q_test_conf = self.assign_config(q_test,i)
            # find q_test in the distance of D from q
            if(tester.self_collision_test(q_test_conf)) and tester.test_config_distance_tau(m,q_test_conf,self,tau):
                T.addEdge(str(m),str(q_test_conf))
                return q_test_conf
            else: # find middle point
                tmpA = np.asarray(q.str2list())
                tmpB = np.asarray(q_test_conf.str2list())
                tmpC = (tmpA+tmpB)/2
                midRobot = make_robot_config_with_arr(tmpA[0],tmpA[1],tmpC[2:(2+self.num_segments)],tmpC[2+self.num_segments:(2+2*self.num_segments)],tmpA[-1])
                if(tester.self_collision_test(midRobot)) and tester.test_config_distance_tau(m,midRobot,self,tau):
                    T.addEdge(str(m),str(midRobot))
                    return midRobot
        return None




    def connectTree(self,added_m, flagInit):
        tester = test_robot(self)
        tau = 0.4
        # check added_m in the other tree
        T = self.gInit if flagInit == 1 else self.gGoal
        visited = set()
        for i in range(T.getNumbVertices()):
            VertexTmp = T.getVertex(i)
            prVertex = math.floor(random.random() * T.getNumbVertices())
            if (prVertex not in visited):
                visited.add(prVertex)
                mVertex = T.getVertex(T.getVerticeByInt(prVertex))
                q = self.str2robotConfig(mVertex.getId())
                
                # print("comp:",added_m)
                # print("other tree:",q)
                # should find the minimum q to calculate the distance, but not implement now
                if tester.test_config_distance_tau(added_m, q, self, tau):
                    # print(added_m)
                    # print(q)
                    # self.combineTwoGraph(flagInit,added_m,q)                
                    return True,q
        return False,None

    def combineTwoGraph(self,flagInit,added_m,q):
        # added_m in the self tree
        initNode = added_m if flagInit == 0 else q
        goalNode = added_m if flagInit == 1 else q
        self.gInit.addEdge(str(initNode),str(goalNode))
        goalTmp = self.gGoal.getVertex(str(goalNode))
        while goalTmp.getParent():
            self.gInit.addEdge(str(goalTmp), str(goalTmp.getParent()))
            goalTmp = self.gGoal.getVertex(goalTmp.getParent())



def main():
# def main(arglist):
    file = './testcases/4g1_m1.txt'
    outfile = 'out/4g1_m1_output.txt'
    # file = arglist[0]
    # outfile = arglist[1]
    
    prm = EST(file)
    # config = prm.sampling(1000)
    # robot = prm.get_init_state();
    f = open("tmp_output.txt", "w")
    f.write("")
    f.close()
    prm.run_EST(outfile)
    # D = [1e-3,1e-3]
    # sampleRobot = prm.sampling_withinD(robot,D,100)
    # robot = prm.assign_config(config.tolist(),0)
    # print(sampleRobot)
    # print(config.tolist())
    # config = config[:,:-1]
    # prm.write_config('test.txt',config.tolist())
    
    aa = test_robot(prm)
    qq = aa.load_output('output.txt')
    vis = Visualiser(prm, qq)
    # print(qq)

if __name__ == '__main__':
    main()
    # main(sys.argv[1:])
    

    

