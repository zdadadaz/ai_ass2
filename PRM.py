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


    def run_PRM(self,outPath):
        numberSamples = 10000
        knearest = 20
        tau = 0.4
        print("build graph")
        self.buildGraph(numberSamples,knearest,tau)
        print("collision check")
        self.PRM_collision_check()
        
        init = self.get_init_state()
        state = str(init)
        search = astar(state)
        print("find shortest path")
        path = search.astar_run(self.gPrm,str(self.get_goal_state()),self)
        print("shortest path")
        arr =[]
        for i in path:
            arr.append(i[0][:-3])
        write_robot_config_list_to_file(outPath,arr) 
        
    
    def buildGraph(self,numberSamples,knearest,tau):
        tester = test_robot(self)
        samples = self.sampling(numberSamples)
        
        count =0
        for sp in range(numberSamples):
            rob = self.assign_config(samples,sp)
            if tester.self_collision_test(rob):
                self.gPrm.addVertex(str(rob))
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
        self.gPrm.addVertex(str(init))
        myarray = np.asarray(init.str2list()[:-1])
        # myarray = np.asarray(init.points)
        # r,c = myarray.shape
        # myarray = myarray.reshape(1,c*r)
        output = np.vstack([output, myarray])
        goal = self.get_goal_state()
        
        self.gPrm.addVertex(str(goal))
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
            curNode = self.gPrm.getVerticeByInt(sp)
            m = self.str2robotConfig(curNode)
            for kn in range(1,knearest):
                knNode = self.gPrm.getVerticeByInt(ind[0][kn])
                q = self.str2robotConfig(knNode)
                if (sp < r-2):
                    if(tester.test_config_distance(m,q,self,tau)):
                        self.gPrm.addEdge(curNode,knNode)
                else:
                    self.gPrm.addEdge(curNode,knNode)

    def PRM_collision_check(self):
        numLayer = 5
        visited = set()
        verlists = self.gPrm.getVertices()
        for ver in verlists:
            verA = self.gPrm.getVertex(ver)
            connects = verA.getConnections()
            robA = self.str2robotConfig(verA.getId())
            for conn in connects:
                conn_key = conn.getId()
                if ((ver,conn_key) not in visited) and ((conn_key,ver) not in visited):
                    visited.add((ver,conn_key))
                    visited.add((conn_key,ver))
                    verB = conn
                    robB = self.str2robotConfig(verB.getId())
                    #   False-> no collision, True -> have collision
                    if (self.collision_check(np.asarray(robA.str2list()),np.asarray(robB.str2list()),numLayer)):
                        verA.delete_connect_id(conn_key)
                        verB.delete_connect_id(ver)


    

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
