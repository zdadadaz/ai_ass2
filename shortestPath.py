from prioritize import Prioritize
from queue import PriorityQueue
from support.robot_config_m import make_robot_config_from_ee1 as rob_conf_ee1
from support.robot_config_m import make_robot_config_from_ee2 as rob_conf_ee2
from support.angle import Angle
from tester_m import test_config_distance,test_config_distance_out

class astar():
    def __init__(self,state = None, parent = None,depth = 0,cost=0):
        self.state = state
        self.parent = parent
        self.depth = depth
        self.sofar_cost = cost
        self.actioncost = 1
        self.name = 'astar'
        # self.vertex = None

    def traverse(self,node):
        x = node
        statelist = []
        while x.parent:
            statelist.append((x.state,x.depth))
            x = x.parent
        statelist.append((x.state,x.depth))
        arr = []
        for i in reversed(statelist):
            arr.append(i)
        return arr

    def heuristic_func(self,graph,nextNode,finish,spec):
        # print(self.difference(nextNode,finish))
        return self.difference(graph,nextNode,finish,spec)
        # return 0
    
    def str2robotConfigqq(self, stringInput):
        ee1_xy_str, ee1_angles_str, lengths_str, ee1_grappled = stringInput.strip().split(';')
        ee1x, ee1y = tuple([float(i) for i in ee1_xy_str.split(' ')])
        ee1_angles = [Angle(degrees=float(i)) for i in ee1_angles_str.strip().split(' ')]
        lengths = [float(i) for i in lengths_str.strip().split(' ')]
        if int(ee1_grappled) == 1:
            return rob_conf_ee1(ee1x, ee1y, ee1_angles, lengths, ee1_grappled=True)
        else: 
            return rob_conf_ee2(ee1x, ee1y, ee1_angles, lengths, ee2_grappled=True)

    def difference(self,graph,strA,strB,spec):
        if strA[-1] != strB[-1]:
            return 10000
        if strA[-1] == strB[-1]:
            if strA[-1]==1:
                if strA.split(' ')[0] == strB.split(' ')[0] and strA.split(' ')[1] == strA.split(' ')[1]:
                    robotA = self.str2robotConfigqq(strA)
                    robotB = self.str2robotConfigqq(strB)
                    return test_config_distance_out(robotA,robotB,spec)
                else:
                    return 10000
                    # return (float)verB.connNum /(float)graph.getNumbVertices()
            else:
                if strA.split(' ')[0] == strB.split(' ')[0] and strA.split(' ')[1] == strA.split(' ')[1]:
                    robotA = self.str2robotConfigqq(strA)
                    robotB = self.str2robotConfigqq(strB)
                    return test_config_distance_out(robotA,robotB,spec)
                else:
                    return 10000
                    # return (float)verB.connNum /(float)graph.getNumbVertices()
            
    def differenceTF(self,strA,strB,spec):
        if strA[-1] != strB[-1]:
            return False
        if strA[-1] == strB[-1]:
            if strA[-1]==1:
                if strA.split(' ')[0] == strB.split(' ')[0] and strA.split(' ')[1] == strA.split(' ')[1]:
                    robotA = self.str2robotConfigqq(strA)
                    robotB = self.str2robotConfigqq(strB)
                    return test_config_distance(robotA,robotB,spec)
                else:
                    return False
            else:
                if strA.split(' ')[0] == strB.split(' ')[0] and strA.split(' ')[1] == strA.split(' ')[1]:
                    robotA = self.str2robotConfigqq(strA)
                    robotB = self.str2robotConfigqq(strB)
                    return test_config_distance(robotA,robotB,spec)
                else:
                    return False

    def astar_run(self,graph,finish, spec):
        queue = PriorityQueue()
        self.sofar_cost = 0
        queue.put(Prioritize(0,self))
        visited = set()
        weights = {}
        count = 0
        while not queue.empty():
            curItem = queue.get()   
            curNode = curItem.item
            curVertexId = curNode.state
            weights[curVertexId] = curNode
            visited.add(curVertexId)
            # print(curVertexId)
            # print(finish)
            count += 1
            # if(count == 6):
            #     qq =1
            # print(self.difference(curVertexId, finish,spec))
            if (self.differenceTF(curVertexId, finish,spec)):
                # traverse
                return self.traverse(curNode)

            curVertex = graph.getVertex(curVertexId)
            if curVertex.checkConnections():
                for i in curVertex.getConnections():
                    newCost = curNode.sofar_cost + self.actioncost
                    # if i.getId() not in visited or (newCost < weights[i.getId()].sofar_cost):
                    if i not in visited:
                        newNode = astar(state=i,parent=curNode,depth = curNode.depth+1,cost =newCost)
                        weights[i] = newNode
                        weights[i].sofar_cost = newCost
                        priority_cost = newCost + self.heuristic_func(graph,i,finish,spec)
                        queue.put(Prioritize(int(priority_cost),weights[i]))
                    elif newCost < weights[i].sofar_cost:
                        weights[i].sofar_cost = newCost
                        priority_cost = newCost + self.heuristic_func(graph,i,finish,spec)
                        queue.put(Prioritize(int(priority_cost),weights[i]))
        return []

    def astar_run_mutipleGraph(self,graph,finish, spec):
        queue = PriorityQueue()
        self.sofar_cost = 0
        queue.put(Prioritize(0,self))
        visited = set()
        weights = {}
        count = 0
        while not queue.empty():
            curItem = queue.get()   
            curNode = curItem.item
            curVertexId = curNode.state
            weights[curVertexId] = curNode
            visited.add(curVertexId)
            # print(curVertexId)
            # print(finish)
            count += 1
            # if(count == 6):
            #     qq =1
            # print(self.difference(curVertexId, finish,spec))
            if (self.differenceTF(curVertexId, finish,spec)):
                # traverse
                return self.traverse(curNode)

            curVertex = graph.getVertex(curVertexId)
            if curVertex.checkConnections():
                for i in curVertex.getConnections():
                    newCost = curNode.sofar_cost + self.actioncost
                    # if i.getId() not in visited or (newCost < weights[i.getId()].sofar_cost):
                    if i not in visited:
                        newNode = astar(state=i,parent=curNode,depth = curNode.depth+1,cost =newCost)
                        weights[i] = newNode
                        weights[i].sofar_cost = newCost
                        priority_cost = newCost + self.heuristic_func(i,finish,spec)
                        queue.put(Prioritize(int(priority_cost),weights[i]))
                    elif newCost < weights[i].sofar_cost:
                        newNode = astar(state=i.getId(),parent=curNode,depth = curNode.depth+1,cost =newCost)
                        weights[i] = newNode
                        weights[i].sofar_cost = newCost
                        priority_cost = newCost + self.heuristic_func(graph,i,finish,spec)
                        queue.put(Prioritize(int(priority_cost),weights[i]))
        return []