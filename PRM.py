from EST import EST
from util import Graph,Vertex
from sklearn.neighbors import KDTree
import numpy as np
from test_robot import test_robot
from shortestPath import astar
from support.robot_config import write_robot_config_list_to_file
from visualiser import Visualiser 
from interpolation import Interpolation
from util import write_sampling_config
import math
import random

class PRM(EST):
    def __init__(self, input_file):
        super(PRM,self).__init__(input_file)
        self.gPrm = Graph()
        self.collision_checkList = {}
        self.gDict = {}
        gPoints =self.get_grapple_points()
        initState = self.get_init_state();
        initPos = initState.points[0]
        goalState = self.get_goal_state();    
        goalPos = goalState.points[0]
        for i in range(self.num_grapple_points):
            self.gDict[gPoints[i]] = Graph()
            if (initPos[0] == gPoints[i][0] and initPos[1] == gPoints[i][1]):
                self.gDict[gPoints[i]].addVertex(str(initState))
            if (goalPos[0] == gPoints[i][0] and goalPos[1] == gPoints[i][1]):
                self.gDict[gPoints[i]].addVertex(str(goalState))

    def run_PRM(self,outPath):
        numberSamples_global = 10000
        numberSamples_local = 100
        knearest = 20
        tau = 0.4
        # print("build global graph")
        # self.buildGraph(self.gPrm,numberSamples_global,knearest,tau)
        print("build local graph")
        self.PRM_expansionBuildGraph(numberSamples_local,tau)
        connectVertex(knearest,tau)
        
        # print("global collision check")
        # self.PRM_collision_check(self.gPrm)
        print("local collision check")
        gPoints =self.get_grapple_points()
        for i in range(self.num_grapple_points):
            self.PRM_collision_check(self.gDict[gPoints[i]])
        
        init = self.get_init_state()
        state = str(init)
        search = astar(state)
        print("find shortest path")
        graph = self.gDict[gPoints[0]]
        # graph = self.gPrm
        path = search.astar_run(graph,str(self.get_goal_state()),self)
        print("shortest path")
        arr =[]
        # print collision check point in output
        # self.add_collision2output(self.gPrm,self.collision_checkList,arr,path)
        for i in range(len(path)):
                arr.append(path[i][0][:-3])
        write_robot_config_list_to_file(outPath,arr) 
        

    def buildGraph(self,graph,numberSamples,knearest,tau):
        tester = test_robot(self)
        samples = self.sampling(numberSamples)
        
        count =0
        for sp in range(numberSamples):
            rob = self.assign_config(samples,sp)
            if tester.self_collision_test(rob):
                graph.addVertex(str(rob))
                points = rob.str2list()[:-1]
                # points = rob.points
                myarray = np.asarray(points)
                # r,c = myarray.shape
                # myarray = myarray.reshape(1,c*r)
                if count == 0:
                    output = myarray
                    count+=1
                else:
                    output = np.vstack([output, myarray])
                    count+=1
        
        init = self.get_init_state()
        graph.addVertex(str(init))
        myarray = np.asarray(init.str2list()[:-1])
        # myarray = np.asarray(init.points)
        # r,c = myarray.shape
        # myarray = myarray.reshape(1,c*r)
        output = np.vstack([output, myarray])
        goal = self.get_goal_state()
        
        graph.addVertex(str(goal))
        myarray = np.asarray(goal.str2list()[:-1])
        # myarray = np.asarray(goal.points)
        # r,c = myarray.shape
        # myarray = myarray.reshape(1,c*r)
        output = np.vstack([output, myarray])
        r,c = output.shape
        tree = KDTree(output,leaf_size=2)
        # print(output)
        for sp in range(r):
            if sp >= r-2:
                knearest = 2
            dist, ind = tree.query(output[sp:sp+1], k=knearest) 
            curNode = graph.getVerticeByInt(sp)
            m = self.str2robotConfig(curNode)
            for kn in range(1,knearest):
                knNode = graph.getVerticeByInt(ind[0][kn])
                q = self.str2robotConfig(knNode)
                if (sp < r-2):
                    if(tester.test_config_distance(m,q,self,tau)):
                        graph.addEdge(curNode,knNode)
                else:
                    graph.addEdge(curNode,knNode)

    def PRM_expansionBuildGraph(self,numberOfsampling,tau):
        # numberOfsampling = 100
        success_limit = 10
        D = [0.4,0.3]
        # D = [0.4,1e-3]
        # tau = 0.5
        k =1
        pr = int(math.floor(random.random() * self.num_grapple_points))
        gPoints = self.get_grapple_points()
        for pr in range(len(gPoints)):
            T = self.gDict[gPoints[pr]]
            tester = test_robot(self)
            successCount = 0
            prevRobot = None
            while T.getNumbVertices() < numberOfsampling:
                # over success_limit success, change node
                if prevRobot is not None and successCount != 0:
                    m = prevRobot
                else:
                    if T.getNumbVertices() == 0:
                        samples = self.sampling_eexy(gPoints[pr],1)
                        m = self.assign_config(samples,0)
                    else:
                        prVertex = math.floor(random.random() * T.getNumbVertices())
                        mVertex = T.getVertex(T.getVerticeByInt(prVertex))
                        m = self.str2robotConfig(mVertex.getId())
                
                # m need to become robot
                robot_conf = self.sampling_withinD(m,D,k)
                q = self.assign_config(robot_conf,0)
                # check q is collision or not
                if(tester.self_collision_test(q)) and tester.test_config_distance_tau(m,q,self,tau):
                    # print(str(q)[:-3])
                    # f = open("tmp_output.txt", "a")
                    # f.write(str(q)[:-3] + '\n')
                    # f.close()
                    T.addEdge(str(m),str(q))
                    successCount += 1
                    prevRobot = q
                    return q,pr
                else:  # obstacle sampling
                    successCount = 0
                    output = self.obstacle_sampling(T,m,q,D,k,tester,tau)
                    if output is not None:
                        successCount += 1
                        prevRobot = output
                        return output,pr
                if successCount > success_limit:
                    successCount = 0

    def connectVertex(self,knearest,tau):
        # knearest = 10
        initState = self.get_init_state()
        initPos = initState.points[0]
        goalState = self.get_goal_state()
        goalPos = goalState.points[0]
        gPoints =self.get_grapple_points()
        for i in range(len(gPoints)):
            graph = self.gDict[gPoints[i]]
            arr = graph.getAllkeylist()
            tree = self.KDtreeVertex(graph,arr,knearest,tau)
            # attach init state on graph
            # if (initPos[0] == gPoints[i][0] and initPos[1] == gPoints[i][1]):
            #     initArr = np.asarray(initState.str2list())
            #     initArr = initArr[:-1]
            #     # initArr might be error, need array[[]]
            #     addStartFinish(self,tree,graph,initArr,initState,knearest)
            # # attach goal state on graph
            # if (goalPos[0] == gPoints[i][0] and goalPos[1] == gPoints[i][1]):
            #     goalArr = np.asarray(goalState.str2list())
            #     goalArr = goalArr[:-1]
            #     # initArr might be error, need array[[]]
            #     addStartFinish(self,tree,graph,goalArr,goalState,knearest)

    def addStartFinish(self,tree,graph,init,curNode,knearest):
        dist, ind = tree.query(init, k=knearest) 
        m = self.str2robotConfig(curNode)
        for kn in range(1,knearest):
                knNode = graph.getVerticeByInt(ind[0][kn])
                q = self.str2robotConfig(knNode)
                graph.addEdge(curNode,knNode)

    def KDtreeVertex(self,graph,arr,knearest,tau):
        tester = test_robot(self)
        tree = KDTree(arr,leaf_size=2)
        r,c = arr.shape
        # print(output)
        for sp in range(r):
            dist, ind = tree.query(arr[sp:sp+1], k=knearest) 
            curNode = graph.getVerticeByInt(sp)
            mVer = graph.getVertex(curNode)
            m = self.str2robotConfig(curNode)
            for kn in range(1,knearest):
                knNode = graph.getVerticeByInt(ind[0][kn])
                q = self.str2robotConfig(knNode)
                conns = mVer.getConnections()
                if(q not in conns and tester.test_config_distance(m,q,self,tau)):
                    graph.addEdge(curNode,knNode)
        return tree
    def PRM_collision_check(self,gaph):
        numLayer = 2
        tester = test_robot(self)
        visited = set()
        verlists = gaph.getVertices()
        # count = 0
        for ver in verlists:
            verA = gaph.getVertex(ver)
            connects = verA.getConnections()
            robA = self.str2robotConfig(verA.getId())
            for conn in connects:
                conn_key = conn.getId()
                if ((ver,conn_key) not in visited) and ((conn_key,ver) not in visited):
                    visited.add((ver,conn_key))
                    visited.add((conn_key,ver))
                    verB = conn
                    robB = self.str2robotConfig(verB.getId())
                    # bounding box check, can decrease checking times a lot
                    if(not tester.self_bounding_collision_test(robA)) and (not tester.self_bounding_collision_test(robB)):
                        continue
                    #   False-> no collision, True -> have collision
                    # count += 1
                    checkList = []
                    flag = self.collision_check(np.asarray(robA.str2list()),np.asarray(robB.str2list()),numLayer,checkList)
                    self.collision_checkList[(ver,conn_key)] = checkList
                    self.collision_checkList[(conn_key,ver)] = checkList
                    if flag:
                    # if (self.collision_check(np.asarray(robA.str2list()),np.asarray(robB.str2list()),numLayer)):
                        verA.delete_connect_id(conn_key)
                        verB.delete_connect_id(ver)
        
        # print("====  collision check times ==== ", count)
    def add_collision2output(self,graph, collision_dict,arr,path):
        for i in range(len(path)):
            if i != len(path) -1:
                # print(path[i])
                A = graph.getVertex(path[i][0])
                B = graph.getVertex(path[i+1][0])
                arr.append(path[i][0][:-3])
                if (A.getId(),B.getId()) in collision_dict:
                    clists = collision_dict[(A.getId(),B.getId())] 
                    for j in clists:
                        arr.append(j[:-3])
            else:
                arr.append(path[i][0][:-3])


    

# def main():
#     file = './testcases/3g1_m1.txt'
#     prm = PRM(file)
#     prm.run_PRM()
#     aa = test_robot(prm)
#     qq = aa.load_output('output.txt')
#     vis = Visualiser(prm, qq)
   
#   interpolation
    # aa = test_robot(prm)
    # qq = aa.load_output('output.txt')
    # aa = []
    # for i in qq:
    #     aa.append(str(i))
    # gginterpolat = Interpolation(aa)
    # gg = gginterpolat.run_Interpolate()
    # write_sampling_config('interpolation.txt',3,gg)

# if __name__ == '__main__':
#     main()
