from support.problem_spec import ProblemSpec
from support.robot_config import RobotConfig
from support.robot_config import make_robot_config_from_ee1 as rob_conf_ee1
from support.robot_config import make_robot_config_from_ee2 as rob_conf_ee2
from support.robot_config import write_robot_config_list_to_file
from support.angle import Angle

from shortestPath import astar

from test_robot import test_robot
from util import Graph,Vertex

import random
from visualiser import Visualiser 
import random as rd
import numpy as np
import math



class EST(ProblemSpec):
    def __init__(self, input_file):
        super(EST,self).__init__(input_file)
        self.gInit = Graph()
        self.gGoal = Graph()

    def generateConnectedGrapple(self):
        numSeg = self.get_num_segment()
        points = self.get_grapple_points()


    def sampling(self, numSampling):
        minMax = self.get_min_max_len()
        numSeg = self.get_num_segment()
        grapple_point = self.get_grapple_points()

        grapples_tmp = (np.floor(np.random.rand(1,numSampling)*len(grapple_point)))
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
            grapples.append(list(grapple_point[int(grapples_tmp[0][i])]))
        output = np.append(np.asarray(grapples), angles, axis=1)
        output = np.append(output, lengths, axis=1)
        grapples_tmp = grapples_tmp.reshape(numSampling,1)
        output = np.append(output,grapples_tmp,axis=1)
        # print(output)
        return output
    
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
                tmp += minMax[0][i] 
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
            lengths.append(sample_config[id][5+i])
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

    
    def run_EST(self):
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
                write_robot_config_list_to_file('output.txt',robot_config_list)
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
        k = 1000
        D = [0.6,1e-3]
        tau = 0.5
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
                if(tester.self_collision_test(q)) and tester.test_config_distance_tau(m,q,self,tau/(i+1)):
                    # print(str(q)[:-3])
                    f = open("tmp_output.txt", "a")
                    f.write(str(q)[:-3] + '\n')
                    f.close()
                    T.addEdge(str(m),str(q))
                    return q,pr
    
    def expandTree_NotRandom(self):
        k = 1000
        D = [0.6,1e-3]
        tau = 0.5
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
                if(tester.self_collision_test(q)) and tester.test_config_distance_tau(m,q,self,tau/(i+1)):
                    # print(str(q)[:-3])
                    f = open("tmp_output.txt", "a")
                    f.write(str(q)[:-3] + '\n')
                    f.close()
                    T.addEdge(str(m),str(q))
                    return q,pr

    def connectTree(self,added_m, flagInit):
        tester = test_robot(self)
        tau = 0.2
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
    file = './testcases/3g1_m1.txt'
    # outfile = 'deadlock1_out.txt'
    # file = arglist[0]
    # outfile = arglist[1]
    
    prm = EST(file)
    # config = prm.sampling(1000)
    # robot = prm.get_init_state();
    prm.run_EST()
    # D = [1e-3,1e-3]
    # sampleRobot = prm.sampling_withinD(robot,D,100)
    # robot = prm.assign_config(config.tolist(),0)
    # print(sampleRobot)
    # print(config.tolist())
    # config = config[:,:-1]
    # prm.write_config('test.txt',config.tolist())
    f = open("tmp_output.txt", "w")
    f.write("")
    f.close()
    aa = test_robot(prm)
    qq = aa.load_output('output.txt')
    vis = Visualiser(prm, qq)
    # print(qq)

if __name__ == '__main__':
    main()
    # main(sys.argv[1:])
    

    

