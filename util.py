from support.angle import Angle

class Vertex:
    def __init__(self,key, number):
        self.id = key
        self.connectedTo = {}
        # self.parentIs = {}
        self.parentIs = None
        self.num = number
        self.connNum = 0

    def key2list(self):
        # not include the last flag of ee
        arr = []
        tmp = self.id.split(' ')
        segNum = int((len(tmp)-3)/2)
        for i in range(len(tmp)-1):
            if i < 2:
                qq = float(tmp[i].replace(';',''))
                qq = qq*10000            
            elif i>1 and i <= 1+segNum :
                qq = float(tmp[i].replace(';',''))
                tmpAng = Angle(degrees=qq)
                qq = tmpAng.in_radians()
            else:
                qq = float(tmp[i].replace(';',''))
            arr.append(qq)
        return arr

    def ass_key2list(self,in_id):
        # not include the last flag of ee
        arr = []
        tmp = in_id.split(' ')
        segNum = int((len(tmp)-3)/2)
        for i in range(len(tmp)-1):
            if i < 2:
                qq = float(tmp[i].replace(';',''))
                qq = qq*10000         
            elif i>1 and i <= 1+segNum :
                qq = float(tmp[i].replace(';',''))
                tmpAng = Angle(degrees=qq)
                qq = tmpAng.in_radians()
            else:
                qq = float(tmp[i].replace(';',''))
            arr.append(qq)
        return arr

    def delete_connect_id(self,key):
        if self.connectedTo.pop(key, None) is not None:
            self.connNum -= 1
    
    def getAllconnkeylist(self):
        arr = []
        for con in self.connectedTo.keys():
            arr.append(self.ass_key2list(con))
        return arr
        
    def addNeighbor(self,nbr,weight=0):
        self.connNum += 1
        self.connectedTo[nbr] = weight
    
    # def addParent(self,nbr,weight=0):
        # self.parentIs[nbr] = weight
    def addParent(self,nbr,weight=0):
        self.parentIs= nbr

    def __str__(self):
        return str(self.id) + ' connectedTo: ' + str([x.id for x in self.connectedTo])

#   False -> empty
    def checkConnections(self):
        if self.connNum <=0:
            return False
        else:
            return True

    def getConnections(self):
        return self.connectedTo.keys()
    
    def getConnectionsIds(self):
        # arr = []
        arr = set()
        for i in self.connectedTo.keys():
            # arr.append(i)
            arr.add(i)
        return arr

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
        if key not in self.vertList:
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
        self.vertList[f].addNeighbor(self.vertList[t].getId(), cost)
        self.vertList[t].addParent(self.vertList[f].getId(),cost)

    def getVertices(self):
        return self.vertList.keys()

    def __iter__(self):
        return iter(self.vertList.values())


def write_sampling_config(filename,numSeg, robot_config_list):
    # numSeg = self.get_num_segment()
    f = open(filename, 'w')
    for rc in robot_config_list:
        f.write(str(round(rc[0],8)) + ' ')
        f.write(str(round(rc[1],8)) + ';')
        for i in range(numSeg):
            f.write(' '+str(round(rc[2+i],8)))
        f.write(';')
        for i in range(numSeg):
            f.write(' '+str(round(rc[2+numSeg+i],8)))
        f.write('\n')
    f.close()
# file = './testcases/3g1_m1.txt'
# prm = PRM(file)    
# aa = test_robot(prm)
# qq = aa.load_output('output.txt')
# print(qq)