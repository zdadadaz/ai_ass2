
class Vertex:
    def __init__(self,key, number):
        self.id = key
        self.connectedTo = {}
        # self.parentIs = {}
        self.parentIs = None
        self.num = number

    def key2list(self):
        arr = []
        tmp = self.id.split(' ')
        for i in tmp:
            arr.append(i.replace(';',''))
        return arr

    def delete_connect_id(self,key):
        self.connectedTo.pop(key, None)  
        
    def addNeighbor(self,nbr,weight=0):
        self.connectedTo[nbr] = weight
    
    # def addParent(self,nbr,weight=0):
        # self.parentIs[nbr] = weight
    def addParent(self,nbr,weight=0):
        self.parentIs= nbr


    def __str__(self):
        return str(self.id) + ' connectedTo: ' + str([x.id for x in self.connectedTo])

#   False -> empty
    def checkConnections(self):
        return bool(self.connectedTo)

    def getConnections(self):
        return self.connectedTo.keys()

    def getParents(self):
        return self.parentIs
        # return self.parentIs.keys()

    def getId(self):
        return self.id

    def getWeight(self,nbr):
        return self.connectedTo[nbr]

class Graph:
    def __init__(self):
        self.vertList = {}
        self.numVertices = 0
        self.vertArr = []

    def getAllkeylist(self):
        arr = []
        for i in range(self.numVertices):
            Verid = self.vertArr[i]
            Ver = self.getVertex(Verid)
            arr.append(Ver.key2list())
        return arr

    def getNumbVertices(self):
        return self.numVertices

    def getVerticeByInt(self, n):
        return self.vertArr[n]

    def addVertex(self,key):
        self.numVertices = self.numVertices + 1
        newVertex = Vertex(key,self.numVertices)
        self.vertArr.append(key)
        self.vertList[key] = newVertex
        return newVertex

    def getVertex(self,n):
        if n in self.vertList:
            return self.vertList[n]
        else:
            return None

    def __contains__(self,n):
        return n in self.vertList

    def addEdge(self,f,t,cost=0):
        if f not in self.vertList:
            nv = self.addVertex(f)
        if t not in self.vertList:
            nv = self.addVertex(t)
        self.vertList[f].addNeighbor(self.vertList[t], cost)
        self.vertList[t].addParent(self.vertList[f],cost)

    def getVertices(self):
        return self.vertList.keys()

    def __iter__(self):
        return iter(self.vertList.values())


def write_sampling_config(filename,numSeg, robot_config_list):
    # numSeg = self.get_num_segment()
    f = open(filename, 'w')
    for rc in robot_config_list:
        f.write(str(rc[0]) + ' ')
        f.write(str(rc[1]) + ';')
        for i in range(numSeg):
            f.write(' '+str(rc[2+i]))
        f.write(';')
        for i in range(numSeg):
            f.write(' '+str(rc[5+i]))
        f.write('\n')
    f.close()
# file = './testcases/3g1_m1.txt'
# prm = PRM(file)    
# aa = test_robot(prm)
# qq = aa.load_output('output.txt')
# print(qq)