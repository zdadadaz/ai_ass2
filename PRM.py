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
            # if (initPos[0] == gPoints[i][0] and initPos[1] == gPoints[i][1]):
            #     self.gDict[gPoints[i]].addVertex(str(initState))
            # if (goalPos[0] == gPoints[i][0] and goalPos[1] == gPoints[i][1]):
            #     self.gDict[gPoints[i]].addVertex(str(goalState))

        # setting
        # self.numberSamples_global = 10000
        # self.numberSamples_local = 10000
        # self.knearest = 20
        # self.expand_tau = 0.4
        # self.conn_tau = 0.4
        self.numlayer =2

    def run_PRM(self,outPath):
        numberSamples_global = 1000
        numberSamples_local = 500
        knearest = 20
        expand_tau = 0.4
        conn_tau = 0.4
        global_tau =0.4
        clayer =2

        pathTot = []
        # for g in range(self.num_grapple_points):
        g = 0
        gPoints =self.get_grapple_points()
        graph = self.gDict[gPoints[g]]
        count_ite = 0
        addState = 1
        while True:
            count_ite += 1
            print("build global graph")
            self.buildGraph(graph,numberSamples_global,knearest,global_tau,addState)
            addState = 0
            
            print("build local graph")
            self.PRM_expansionBuildGraph(graph,numberSamples_local,expand_tau)
            self.connectVertex(knearest,conn_tau)
            
            print("global collision check")
            self.PRM_collision_check(graph,clayer)
            # print("local collision check")
            # for i in range(self.num_grapple_points):
            #     self.PRM_collision_check(self.gDict[gPoints[i]],clayer)
            
            print("total vertex number:", graph.getNumbVertices())

            arr = []
            arrTmp = graph.vertArr
            for i in range(len(arrTmp)):
                arr.append(arrTmp[i][:-3])
            write_robot_config_list_to_file("tmp_output.txt",arr) 
        

            init = self.get_init_state()
            state = str(init)
            search = astar(state)
            print("find shortest path")
            path = search.astar_run(graph,str(self.get_goal_state()),self)
            if len(path) == 0:
                print("==== Can't find the shortest path ====")
                # return False
            else:
                print("Found shortest path")
                pathTot.append(path)
                break
    
        arr =[]
        # print collision check point in output
        # self.add_collision2output(self.gPrm,self.collision_checkList,arr,path)
        # normal printout
        for p in pathTot:
            for i in range(len(p)):
                arr.append(p[i][0][:-3])
        write_robot_config_list_to_file(outPath,arr) 
        return True

    def buildGraph(self,graph,numberSamples,knearest,tau,addState):
        tester = test_robot(self)
        points = self.grapple_points
        # samples = self.sampling_eexy(points[0],numberSamples,True)
        # numberSamples_tmp = 1000000
        numberSamples_tmp = numberSamples

        count =0
        loop_count = 0
        # arr = self.sampling_bridge()

        while count<numberSamples:
        # while graph.getNumbVertices()<(numberSamples):
            samples = self.sampling(numberSamples_tmp)
            # print("loop_count = ",loop_count)
            # print("count number Vertices = ",count)
            loop_count += 1
            addCount = 0
            for sp in range(numberSamples_tmp):
                addCount +=1
                rob = self.assign_config(samples,sp)
                if tester.self_collision_test(rob):
                    graph.addVertex(str(rob))
                    # points = rob.str2list()[:-1]
                    points = rob.points
                    myarray = np.asarray(points)
                    r,c = myarray.shape
                    myarray = myarray.reshape(1,c*r)
                    if count == 0:
                        output = myarray
                        count+=1
                    else:
                        output = np.vstack([output, myarray])
                        count+=1
                    if count > numberSamples:
                        break
            # print("addCount = ",addCount)
        if addState==1:
            init = self.get_init_state()
            graph.addVertex(str(init))
            # myarray = np.asarray(init.str2list()[:-1])
            myarray = np.asarray(init.points)
            r,c = myarray.shape
            myarray = myarray.reshape(1,c*r)
            output = np.vstack([output, myarray])
            goal = self.get_goal_state()
            
            graph.addVertex(str(goal))
            # myarray = np.asarray(goal.str2list()[:-1])
            myarray = np.asarray(goal.points)
            r,c = myarray.shape
            myarray = myarray.reshape(1,c*r)
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
        else: 
            r,c = output.shape
            tree = KDTree(output,leaf_size=2)
            # print(output)
            for sp in range(r):
                dist, ind = tree.query(output[sp:sp+1], k=knearest) 
                curNode = graph.getVerticeByInt(sp)
                m = self.str2robotConfig(curNode)
                for kn in range(1,knearest):
                    knNode = graph.getVerticeByInt(ind[0][kn])
                    q = self.str2robotConfig(knNode)
                    if(tester.test_config_distance(m,q,self,tau)):
                        graph.addEdge(curNode,knNode)
                
    def PRM_expansionBuildGraph(self,graph,numberOfsampling,tau):
        # numberOfsampling = 100
        success_limit = 10
        D = [tau,0.1]
        # D = [0.4,1e-3]
        # tau = 0.5
        k =1
        # pr = int(math.floor(random.random() * self.num_grapple_points))
        gPoints = self.get_grapple_points()
        # for pr in range(len(gPoints)):
        T = graph
        tester = test_robot(self)
        successCount = 0
        prevRobot = None
        loop_count = 0
        count_node =0
        while count_node < numberOfsampling:
            loop_count+=1
            # print("loop_count=",loop_count)
            # print("T.getNumbVertices()=",T.getNumbVertices())
            # print(T.getNumbVertices())
            # over success_limit success, change node
            if prevRobot is not None and successCount != 0:
                m = prevRobot
            else:
                # if T.getNumbVertices() == 0:
                #     samples = self.sampling_eexy(gPoints[pr],10, True)
                #     m = self.assign_config(samples,0)
                # else:
                prVertex = math.floor(random.random() * T.getNumbVertices())
                mVertex = T.getVertex(T.getVerticeByInt(prVertex))
                m = self.str2robotConfig(mVertex.getId())
            
            # m need to become robot
            robot_conf = self.sampling_withinD(m,D,k)
            q = self.assign_config(robot_conf,0)

            
            # check q is collision or not
            # if(tester.self_collision_test(q)) and tester.test_config_pos_distance_tau(m,q,tau):
            if(tester.self_collision_test(q)) and tester.test_config_distance_tau(m,q,self,tau):
                # print(str(q)[:-3])
                # f = open("tmp_output.txt", "a")
                # f.write(str(q)[:-3] + '\n')
                # f.close()
                count_node+=1
                T.addEdge(str(m),str(q))
                # T.addVertex(str(q))
                successCount += 1
                prevRobot = q
                
            else:  # obstacle sampling
                successCount = 0
                output = self.obstacle_sampling(T,m,q,D,k,tester,tau)
                if output is not None:
                    count_node+=1
                    successCount += 1
                    prevRobot = output

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
            arr = np.asarray(graph.getAllkeylist())
            # find nearest vertex and connect with collision check
            tree = self.KDtreeVertex(graph,arr,knearest,tau)
            # attach init state on graph
            # if (initPos[0] == gPoints[i][0] and initPos[1] == gPoints[i][1]):
            #     myarray = np.asarray(initState.points)
            #     r,c = myarray.shape
            #     initArr = myarray.reshape(1,c*r)
            #     self.addState2Graph(tree,graph,initArr,str(initState),2)
            # # attach goal state on graph
            # if (goalPos[0] == gPoints[i][0] and goalPos[1] == gPoints[i][1]):
            #     myarray = np.asarray(goalState.points)
            #     r,c = myarray.shape
            #     goalArr = myarray.reshape(1,c*r)
            #     # initArr might be error, need array[[]]
            #     self.addState2Graph(tree,graph,goalArr,str(goalState),2)

    def addState2Graph(self,tree,graph,arr,curNode,knearest):
        dist, ind = tree.query(arr, k=knearest) 
        # m = self.str2robotConfig(str(curNode))
        for kn in range(1,knearest):
                knNode = graph.getVerticeByInt(ind[0][kn])
                # q = self.str2robotConfig(str(knNode))
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
                conns = mVer.getConnectionsIds()
                if(knNode not in conns and tester.test_config_distance(m,q,self,tau)):
                # if(tester.test_config_distance(m,q,self,tau) and (not self.collision_check_one(m,q,self.numlayer))):
                    graph.addEdge(curNode,knNode)
        return tree

    def collision_check_one(self,robA,robB,numLayer):
        tester = test_robot(self)
        checkList1 =[]
        flag = self.collision_check(np.asarray(robA.str2list()),np.asarray(robB.str2list()),1,checkList1)
        if(not tester.self_bounding_collision_test(robA)) and (not tester.self_bounding_collision_test(robB)) and (not flag):
            return False
        #   False-> no collision, True -> have collision
        checkList = []
        flag = self.collision_check(np.asarray(robA.str2list()),np.asarray(robB.str2list()),numLayer,checkList)
        # self.collision_checkList[(ver,conn_key)] = checkList
        # self.collision_checkList[(conn_key,ver)] = checkList
        if flag:
            return True
        else:
            return False

    def PRM_collision_check(self,gaph,numLayer):
        # numLayer = 2
        distB = 0.1
        tester = test_robot(self)
        visited = set()
        verlists = gaph.getVertices()
        count = 0
        for ver in verlists:
            verA = gaph.getVertex(ver)
            connects = verA.getConnections()
            robA = self.str2robotConfig(verA.getId())
            queue4delete = []
            for conn in connects:
                conn_key = conn
                if ((ver,conn_key) not in visited) and ((conn_key,ver) not in visited):
                    visited.add((ver,conn_key))
                    visited.add((conn_key,ver))
                    verB = gaph.getVertex(conn)
                    robB = self.str2robotConfig(verB.getId())
                    # if (ver == "0.5 0.3; 21.27553374 43.33302171 38.52536431 2.34357269; 0.14400488 0.18625804 0.2 0.1; 1") or \
                    #     (verB.getId() == "0.5 0.3; 21.27553374 43.33302171 38.52536431 2.34357269; 0.14400488 0.18625804 0.2 0.1; 1"):
                    #     qq=1
                    #     print("error")
                    # robA = self.str2robotConfig("0.5 0.3; 21.27553374 43.33302171 38.52536431 2.34357269; 0.14400488 0.18625804 0.2 0.1; 1")
                    # robB = self.str2robotConfig("0.5 0.3; 41.90413911 65.32649907 57.55880139 18.82617511; 0.12183253 0.19787794 0.1014939 0.1556962; 1")
            
                    # bounding box check, can decrease checking times a lot
                    checkList1 =[]
                    flag = self.collision_check(np.asarray(robA.str2list()),np.asarray(robB.str2list()),1,checkList1)
                    if(not tester.self_bounding_collision_test(robA)) and (not tester.self_bounding_collision_test(robB)) and (not flag):
                        continue
                    #   False-> no collision, True -> have collision
                    
                    count += 1
                    checkList = []
                    flag = self.collision_check(np.asarray(robA.str2list()),np.asarray(robB.str2list()),numLayer,checkList)
                    self.collision_checkList[(ver,conn_key)] = checkList
                    self.collision_checkList[(conn_key,ver)] = checkList
                    if flag:
                        # verA.delete_connect_id(conn_key)
                        # verB.delete_connect_id(ver)
                        queue4delete.append(conn_key)
            for dd in queue4delete:
                verA.delete_connect_id(dd)
                verB = gaph.getVertex(dd)
                verB.delete_connect_id(verA.getId())

        print("====  collision check times ==== ", count)
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
#     file = './testcases/3g1_m2.txt'
#     sol = './out/'+ file[:-4] + '_output.txt'
#     prm = PRM(file)
#     prm.run_PRM(sol)
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
    # main()
