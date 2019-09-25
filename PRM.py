from EST import EST
from util import Graph,Vertex
from sklearn.neighbors import KDTree
import numpy as np
from test_robot import test_robot
from shortestPath import astar
from support.robot_config import write_robot_config_list_to_file
from visualiser import Visualiser 
# from interpolation import Interpolation
from util import write_sampling_config
import math
import random
from support.robot_config import make_robot_config_with_arr 
import sys
from interpolation import Interpolation
import time

class PRM(EST):
    def __init__(self, input_file):
        super(PRM,self).__init__(input_file)
        self.input_file = input_file
        self.gPrm = Graph()
        self.collision_checkList = {}
        self.gDict = {}
        self.visitedCollide =set()
        gPoints =self.get_grapple_points()
        # for i in range(self.num_grapple_points):
        self.gDict[gPoints[0]] = Graph()
        
        np.random.seed(self.Setting['np-rd'])
        random.seed(self.Setting['random'])

        
    def run_PRM(self,outPath):
        start = time.time()
               
        numberSamples_global = self.Setting['numberSamples_global']
        numberSamples_local = self.Setting['numberSamples_local']
        numBridge = 50
        knearest = 20
        expand_tau = self.Setting['tau']
        conn_tau = self.Setting['tau']
        global_tau =self.Setting['tau']
        clayer =self.Setting['layer']
 
            
        change = self.Setting['numChange']
        count_change = 0
        pathTot = []
        # for g in range(self.num_grapple_points):
        g = 0
        gPoints =self.get_grapple_points()
        count_ite = 0
        addState = 1
        graph = self.gDict[gPoints[0]]
        ee1flags = self.Setting['ee1Flag']
        while True:
            timescript =  time.time()- start
            # if (timescript > 120.0):
            #     print("Out of time")
            #     print("Remove me before submit")
            #     break
            for g in range(self.num_grapple_points):
                count_ite += 1
                eexy = gPoints[g]
                if not numberSamples_global == 0:
                    print("build global graph")
                    self.buildGraph(graph,numberSamples_global,knearest,global_tau,addState,eexy,ee1flags[g])
                    addState = 0     
                
                if self.num_grapple_points > 1:
                    if count_change<change and ee1flags[g+1] != ee1flags[g]:
                        print("build bridge")
                        self.sampling_bridge(graph,numBridge,eexy,gPoints[g+1],ee1flags[g])
                        count_change += 1

                if not numberSamples_local == 0:
                    print("build local graph")
                    self.PRM_expansionBuildGraph(graph,numberSamples_local,expand_tau,eexy,ee1flags[g])
                    self.connectVertex(graph,eexy,knearest,conn_tau)
                
                

            print("global collision check")
            self.PRM_collision_check(graph,clayer)
            
            print("total vertex number:", graph.getNumbVertices())

            # arr = []
            # arrTmp = graph.vertArr
            # for i in range(len(arrTmp)):
            #     arr.append(arrTmp[i][:-3])
            #     ver = graph.getVertex(arrTmp[i])
            #     arr.append("connections below")
            #     for conn in ver.getConnections():
            #         arr.append(conn)
            # write_robot_config_list_to_file("./tmp_output.txt",arr) 
        
        
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
        # self.add_collision2output(graph,self.collision_checkList,arr,pathTot[0])
        # normal printout
        out = []
        for p in pathTot:
            for i in range(len(p)):
                if int(p[i][0][-1]) == 2:
                    rob = self.str2robotConfig(p[i][0])
                    arr.append(rob.strEE1out()[:-3])
                else:
                    arr.append(p[i][0][:-3])
                out.append(int(p[i][0][-1]))
                    
        write_robot_config_list_to_file(outPath,arr) 
        return True,out

    
    def buildGraph(self,graph,numberSamples,knearest,tau,addState,eexy,ee1Flag):
        tester = test_robot(self)
        points = self.grapple_points
        numberSamples_tmp = numberSamples
        D = [tau,0.1]
        count =0
        loop_count = 0
        
        diffAng = self.checkhv_ee(eexy)

        while count<numberSamples:
            samples = self.sampling_eexy(eexy,numberSamples,ee1Flag,diffAng)
            loop_count += 1
            addCount = 0
            for sp in range(numberSamples_tmp):
                addCount +=1
                rob = self.assign_config(samples,sp)
                if tester.self_collision_test(rob):
                    graph.addVertex(str(rob))
                    # points = rob.str2list()[:-1]
                    # points = rob.points
                    points = rob.get_position()
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
                else:
                    self.obstacle_sampling_near_only(graph,rob,D,1,tau)
                    
            
        if addState==1:
            init = self.get_init_state()
            graph.addVertex(str(init))
            # myarray = np.asarray(init.str2list()[:-1])
            myarray = np.asarray(init.get_position())
            r,c = myarray.shape
            myarray = myarray.reshape(1,c*r)
            output = np.vstack([output, myarray])
            goal = self.get_goal_state()
            
            graph.addVertex(str(goal))
            # myarray = np.asarray(goal.str2list()[:-1])
            myarray = np.asarray(goal.get_position())
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
                            if curNode[-1] == knNode[-1]:
                                if int(curNode[-1]) == 1:
                                    if curNode.split(' ')[0] == knNode.split(' ')[0] and curNode.split(' ')[1] == knNode.split(' ')[1]:                        
                                        graph.addEdge(curNode,knNode)                                
                                else:
                                    if curNode.split(' ')[0] == knNode.split(' ')[0] and curNode.split(' ')[1] == knNode.split(' ')[1]:                        
                                        graph.addEdge(curNode,knNode)
                    else:
                        if curNode[-1] == knNode[-1]:
                            if int(curNode[-1]) == 1:
                                if curNode.split(' ')[0] == knNode.split(' ')[0] and curNode.split(' ')[1] == knNode.split(' ')[1]:
                                    graph.addEdge(curNode,knNode)
                            else:
                                if curNode.split(' ')[0] == knNode.split(' ')[0] and curNode.split(' ')[1] == knNode.split(' ')[1]:
                                    graph.addEdge(curNode,knNode)

        # else: 
        #     r,c = output.shape
        #     tree = KDTree(output,leaf_size=2)
        #     # print(output)
        #     for sp in range(r):
        #         dist, ind = tree.query(output[sp:sp+1], k=knearest) 
        #         curNode = graph.getVerticeByInt(sp)
        #         m = self.str2robotConfig(curNode)
        #         for kn in range(1,knearest):
        #             knNode = graph.getVerticeByInt(ind[0][kn])
        #             q = self.str2robotConfig(knNode)
        #             if(tester.test_config_distance(m,q,self,tau)):
        #                 if curNode[-1] == knNode[-1]:
        #                     if int(curNode[-1]) == 1:
        #                         if curNode.split(' ')[0] == knNode.split(' ')[0] and curNode.split(' ')[1] == knNode.split(' ')[1]:
        #                             graph.addEdge(curNode,knNode)
        #                     else:
        #                         graph.addEdge(curNode,knNode)

    def obstacle_sampling_near_only(self,T,q,D,k,tau):
        tester = test_robot(self)
        q_test = self.sampling_withinD(q,D,k,q.get_HeadeePos(),q.ee1_grappled)
        for i in range(k):
            q_test_conf = self.assign_config(q_test,i)
            # find q_test in the distance of D from q
            if(tester.self_collision_test(q_test_conf)):
                curNode = str(q)
                knNode = str(q_test_conf)
                if curNode[-1] == knNode[-1]:
                    if int(curNode[-1]) == 1:
                        if curNode.split(' ')[0] == knNode.split(' ')[0] and curNode.split(' ')[1] == knNode.split(' ')[1]:
                            T.addVertex(str(q_test_conf))
                            return q_test_conf
                    else:
                        if curNode.split(' ')[0] == knNode.split(' ')[0] and curNode.split(' ')[1] == knNode.split(' ')[1]:                        
                            T.addVertex(str(q_test_conf))
                            return q_test_conf

   
    def PRM_expansionBuildGraph(self,graph,numberOfsampling,tau,eexy,ee1Flag):
        # numberOfsampling = 100
        success_limit = 10
        D = [tau,0.1]
        # D = [0.4,1e-3]
        # tau = 0.5
        k =1
        gPoints = self.get_grapple_points()
        T = graph
        tester = test_robot(self)
        successCount = 0
        prevRobot = None
        loop_count = 0
        count_node =0
        while count_node < numberOfsampling:
            loop_count+=1
            # over success_limit success, change node
            if prevRobot is not None and successCount != 0:
                m = prevRobot
            else:
                prVertex = math.floor(random.random() * T.getNumbVertices())
                mVertex = T.getVertex(T.getVerticeByInt(prVertex))
                # if(not isinstance(mVertex.getId(), str)):
                #     qq = 1
                m = self.str2robotConfig(mVertex.getId())
            
            # m need to become robot
            robot_conf = self.sampling_withinD(m,D,k,eexy,ee1Flag)
            q = self.assign_config(robot_conf,0)

            
            # check q is collision or not
            if(tester.self_collision_test(q)) and tester.test_config_distance_tau(m,q,self,tau):
                # print(str(q)[:-3])
                # f = open("tmp_output.txt", "a")
                # f.write(str(q)[:-3] + '\n')
                # f.close()
                curNode = str(m)
                knNode = str(q)
                if curNode[-1] == knNode[-1]:
                    if int(curNode[-1]) == 1:
                        if curNode.split(' ')[0] == knNode.split(' ')[0] and curNode.split(' ')[1] == knNode.split(' ')[1]:
                            count_node+=1
                            T.addEdge(str(m),str(q))
                            successCount += 1
                            prevRobot = q
                    else:
                        if curNode.split(' ')[0] == knNode.split(' ')[0] and curNode.split(' ')[1] == knNode.split(' ')[1]:
                            count_node+=1
                            T.addEdge(str(m),str(q))
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

    def connectVertex(self,graph,gPoints,knearest,tau):
        # knearest = 10
        initState = self.get_init_state()
        initPos = initState.get_HeadeePos()
        goalState = self.get_goal_state()
        goalPos = goalState.get_HeadeePos()
        arr = np.asarray(graph.getAllkeylist())
        # find nearest vertex and connect with collision check
        tree = self.KDtreeVertex(graph,arr,knearest,tau)
        # attach init state on graph
        verInit = graph.getVertex(str(initState)) 
        if not verInit.checkConnections():
            # myarray = np.asarray(initState.get_position())
            # r,c = myarray.shape
            # initArr = myarray.reshape(1,c*r)
            initArr = np.asarray([initState.str2list_radians()])
            self.addState2Graph(tree,graph,initArr,str(initState),20)
        # attach goal state on graph
        verGoal = graph.getVertex(str(goalState)) 
        if not verGoal.checkConnections():
            # myarray = np.asarray(goalState.get_position())
            # r,c = myarray.shape
            # goalArr = myarray.reshape(1,c*r)
            goalArr = np.asarray([goalState.str2list_radians()])
            self.addState2Graph(tree,graph,goalArr,str(goalState),20)

    def addState2Graph(self,tree,graph,arr,curNode,knearest):
        arr[0][0] = arr[0][0]*10000
        arr[0][1] = arr[0][1]*10000
        dist, ind = tree.query(arr, k=knearest) 
        # m = self.str2robotConfig(str(curNode))
        for kn in range(0,knearest):
                knNode = graph.getVerticeByInt(ind[0][kn])
                # aa = graph.getVerticeByInt(ind[0][0])
                # q = self.str2robotConfig(str(knNode))
                # if curNode[0] == knNode[0] and curNode[1] == knNode[1]:
                # if curNode == "0.5 0.1; 8.27754461 14.87806786 139.17317948; 0.1753264 0.1 0.10378391; 1" or knNode == "0.5 0.1; 8.27754461 14.87806786 139.17317948; 0.1753264 0.1 0.10378391":
                #     qq=1
                if curNode != knNode:
                    if curNode[-1] == knNode[-1]:
                        if int(curNode[-1]) == 1:
                            if curNode.split(' ')[0] == knNode.split(' ')[0] and curNode.split(' ')[1] == knNode.split(' ')[1]:
                                graph.addEdge(curNode,knNode)
                                break
                        else:
                            if curNode.split(' ')[0] == knNode.split(' ')[0] and curNode.split(' ')[1] == knNode.split(' ')[1]:
                                graph.addEdge(curNode,knNode)
                                break


    def KDtreeVertex(self,graph,arr,knearest,tau):
        tester = test_robot(self)
        tree = KDTree(arr,leaf_size=2)
        r,c = arr.shape
        visited = set()
        
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
                if ((knNode,curNode) not in visited and (curNode,knNode) not in visited):
                    if(knNode not in conns and tester.test_config_distance(m,q,self,tau)):
                        # if knNode == "0.5 0.1; 8.27754461 14.87806786 139.17317948; 0.1753264 0.1 0.10378391; 1" or knNode == "0.5 0.9; -4.82905941 12.32042486 127.10152851; 0.2 0.1 0.12652526; 1":
                        #     qq=1
                        if curNode[-1]==knNode[-1]:
                            if int(curNode[-1])==1:
                                if curNode.split(' ')[0] == knNode.split(' ')[0] and curNode.split(' ')[1] == knNode.split(' ')[1]:
                                    graph.addEdge(curNode,knNode)
                                    graph.addEdge(knNode,curNode)
                                    visited.add((curNode,knNode))
                                    visited.add((knNode,curNode))
                            else:
                                if curNode.split(' ')[0] == knNode.split(' ')[0] and curNode.split(' ')[1] == knNode.split(' ')[1]:
                                    graph.addEdge(curNode,knNode)
                                    graph.addEdge(knNode,curNode)
                                    visited.add((curNode,knNode))
                                    visited.add((knNode,curNode))
        return tree

    def check_obstacle_primitive(self,A,B):
        stepslength = 0.01
        tester = test_robot(self)

        ee1flag = A[-1]
        rob1 = np.asarray(A[:-1])
        rob2 = np.asarray(B[:-1])
        
        numSeg = int((len(rob1)-2)/2)

        diff = abs(rob1 - rob2)
        step = (np.ceil(diff/stepslength))
        step = step.astype(int)
        maxStep = max(step)
        stepslengthlist = diff/maxStep
        maxSteplist = [1,maxStep-1]
        for i in range(len(stepslengthlist)):
            stepslength = stepslengthlist[i]
            stepslength = (-1)*stepslength if rob1[i] > rob2[i] else stepslength
            arr = []
            if (step[i] ==0):
                # for j in maxSteplist:
                for j in range(1,maxStep):
                    if i > 1 and i<2+numSeg:
                        tmpAng = rob1[i]
                        arr.append(tmpAng)
                    else:
                        arr.append(rob1[i])
            else:
                # for j in maxSteplist:
                for j in range(1,maxStep):
                    if i>1 and i<2+numSeg:
                        tmpAng = rob1[i]+ j * stepslength
                        arr.append(tmpAng)
                    else:
                        tmp = rob1[i] + j * stepslength
                        arr.append(tmp)
            if i == 0:
                whole = np.asarray(arr)
            else:
                whole = np.vstack([whole, arr])
        whole = np.transpose(whole)
        for pr in whole:
            pr_rob = make_robot_config_with_arr(A[0],A[1],pr[2:(2+self.num_segments)],pr[(2+self.num_segments):(2+2*self.num_segments)],ee1flag)
            if not tester.self_obstacle_env_test(pr_rob):
                return True # have collision
        return False # no collision



    def PRM_collision_check(self,gaph,numLayer):
        # numLayer = 2
        dist_tau = 100
        tester = test_robot(self)
        verlists = gaph.getVertices()
        count = 0
        for ver in gaph.vertArr:
        # for ver in verlists:
            verA = gaph.getVertex(ver)
            connects = verA.getConnections()
            robA = self.str2robotConfig(verA.getId())
            
            # find whose the farest conn, do more collision check
            # adapt = []
            # if verA.checkConnections():
            #     arrA = verA.getAllconnkeylist()
            #     adapt = np.zeros((1,len(arrA)))
            #     tree = KDTree(arrA,leaf_size=2)
            #     myarray = np.asarray([robA.str2list_radians()])
            #     myarray[0][0] = myarray[0][0]*10000
            #     myarray[0][1] = myarray[0][1]*10000
            #     dist, ind = tree.query(myarray, k=len(arrA)) 
            #     # check which one is greater than threshold
            #     for s in range(len(arrA)):
            #         if dist[0][s] > dist_tau:
            #             adapt[0][ind[0][s]] = 1
                
            queue4delete = []
            s = 0
            for conn in connects:
                conn_key = conn
                # if adapt[0][s] == 1:
                #     numLayer_tmp = numLayer*4
                # else:
                #     numLayer_tmp = numLayer
                # s+=1

                if ((ver,conn_key) not in self.visitedCollide) or ((conn_key,ver) not in self.visitedCollide):
                    self.visitedCollide.add((ver,conn_key))
                    self.visitedCollide.add((conn_key,ver))
                    verB = gaph.getVertex(conn)
                    robB = self.str2robotConfig(verB.getId())
                
                    
                    # # bounding box check, can decrease checking times a lot
                    checkList =[]
                    flag = self.collision_check(np.asarray(robA.str2list()),np.asarray(robB.str2list()),numLayer,checkList)
                    if(not tester.self_bounding_collision_test(robA)) and (not tester.self_bounding_collision_test(robB)) and (not flag):
                        continue
                    #   False-> no collision, True -> have collision
                    
                    count += 1
                    # checkList = []
                    # flag = self.collision_check(np.asarray(robA.str2list()),np.asarray(robB.str2list()),numLayer,checkList)
                    # self.collision_checkList[(ver,conn_key)] = checkList
                    # self.collision_checkList[(conn_key,ver)] = checkList
                    if flag:
                        # verA.delete_connect_id(conn_key)
                        # verB.delete_connect_id(ver)
                        queue4delete.append(conn_key)
            for dd in queue4delete:
                verA.delete_connect_id(dd)
                verB = gaph.getVertex(dd)
                verB.delete_connect_id(verA.getId())

        print("====  collision violate times ==== ", count)
    def add_collision2output(self,graph, collision_dict,arr,path):
        for i in range(len(path)):
            if i != len(path) -1:
                # print(path[i])
                A = graph.getVertex(path[i][0])
                B = graph.getVertex(path[i+1][0])
                if int(path[i][0][-1]) == 2:
                    rob = self.str2robotConfig(path[i][0])
                    arr.append(rob.strEE1out()[:-3])
                else:
                    arr.append(path[i][0][:-3])
                # arr.append(path[i][0][:-3])
                if (A.getId(),B.getId()) in collision_dict:
                    clists = collision_dict[(A.getId(),B.getId())] 
                    for j in clists:
                        if int(j[-1]) == 2:
                            rob = self.str2robotConfig(j)
                            arr.append(rob.strEE1out()[:-3])
                        else:
                            arr.append(j[:-3])
                        # arr.append(j[:-3])
            else:
                arr.append(path[i][0][:-3])

    
    
def main(arglist):
# def main():
    inputfilename = arglist[0]
    sol = inputfilename + '_output.txt'
    # interpolation_path = sol[:-4] +'_ip.txt'
    interpolation_path = arglist[1]
    prm = PRM(inputfilename)
    flagPRM,ee1flag = prm.run_PRM(sol)
    # flagPRM=True
    aa = test_robot(prm)
    qq = aa.load_output(sol)
    strlist = []
    for j in qq:
        strlist.append(str(j))
    # ee1flag=[]
    gginterpolat = Interpolation(strlist,ee1flag)
    print("Run interpolation")
    gg = gginterpolat.run_Interpolate()
    write_sampling_config(interpolation_path,prm.num_segments,gg)
    
if __name__ == '__main__':
    # main()
    main(sys.argv[1:])
