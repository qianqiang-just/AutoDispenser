import ctypes


class DeformationGraphDll(ctypes.Structure):
    _fields_ = [
        ("scaleNorm", ctypes.c_float),
        ('scaleRatio', ctypes.c_float),
        ('nodeSize', ctypes.c_int),
        ('nodeIndicesInNodeContainer', ctypes.POINTER(ctypes.c_int)),
        ('vertexIndicesInNodeContainer', ctypes.POINTER(ctypes.c_int)),
        ('nodeEdgeNum', ctypes.c_int),
        ('nodeIndicesInNodeGraph', ctypes.POINTER(ctypes.c_int)),
        ('neighborNodeIndicesInNodeGraph', ctypes.POINTER(ctypes.c_int)),
        ('weightInNodeGraph', ctypes.POINTER(ctypes.c_float)),
        ('verticesNum', ctypes.c_int),
        ('neighborNodeSum', ctypes.c_int),
        ('vertexIndicesInVertexGraph', ctypes.POINTER(ctypes.c_int)),
        ('neighborNodeIndicesInVertexGraph', ctypes.POINTER(ctypes.c_int)),
        ('weightInVertexGraph', ctypes.POINTER(ctypes.c_float))
    ]



class DeformationGraph:
    def __init__(self):
        self.scaleNorm = None
        self.scaleRatio = None
        self.nodeSize = None
        self.nodeIndicesInNodeContainer = []
        self.vertexIndicesInNodeContainer = []
        self.nodeEdgeNum = None
        self.nodeIndicesInNodeGraph = []
        self.neighborNodeIndicesInNodeGraph = []
        self.weightInNodeGraph = []
        self.verticesNum = None
        self.neighborNodeSum = None
        self.vertexIndicesInVertexGraph = []
        self.neighborNodeIndicesInVertexGraph = []
        self.weightInVertexGraph = []
        self.deformationGraphDll = DeformationGraphDll()

    def readData(self, filePath):
        with open(filePath, 'r') as fp:
            self.scaleNorm = (float)(fp.readline().split(' ')[2].split('\n')[0])
            self.scaleRatio = (float)(fp.readline().split(' ')[2].split('\n')[0])
            fp.readline()
            self.nodeSize = (int)(fp.readline().split(' ')[2].split('\n')[0])
            for i in range(self.nodeSize):
                line = fp.readline().split(' ')
                self.nodeIndicesInNodeContainer.append((int)(line[0]))
                self.vertexIndicesInNodeContainer.append((int)(line[1]))
            fp.readline()
            self.nodeEdgeNum = (int)(fp.readline().split(' ')[2].split('\n')[0])
            for i in range(self.nodeEdgeNum):
                line = fp.readline().split(' ')
                self.nodeIndicesInNodeGraph.append((int)(line[0]))
                self.neighborNodeIndicesInNodeGraph.append((int)(line[1]))
                self.weightInNodeGraph.append((float)(line[2]))
            fp.readline()
            self.verticesNum = (int)(fp.readline().split(' ')[2].split('\n')[0])
            self.neighborNodeSum = (int)(fp.readline().split(' ')[2].split('\n')[0])
            for i in range(self.neighborNodeSum):
                line = fp.readline().split(' ')
                self.vertexIndicesInVertexGraph.append((int)(line[0]))
                self.neighborNodeIndicesInVertexGraph.append((int)(line[1]))
                self.weightInVertexGraph.append((float)(line[2]))

    def loadToDll(self):
        self.deformationGraphDll = DeformationGraphDll()
        self.deformationGraphDll.scaleNorm = self.scaleNorm
        self.deformationGraphDll.scaleRatio = self.scaleRatio
        self.deformationGraphDll.nodeSize = self.nodeSize
        self.deformationGraphDll.nodeIndicesInNodeContainer = (ctypes.c_int * self.nodeSize)(
            *self.nodeIndicesInNodeContainer)
        self.deformationGraphDll.vertexIndicesInNodeContainer = (ctypes.c_int * self.nodeSize)(
            *self.vertexIndicesInNodeContainer)

        self.deformationGraphDll.nodeEdgeNum = self.nodeEdgeNum
        self.deformationGraphDll.nodeIndicesInNodeGraph = (ctypes.c_int * self.nodeEdgeNum)(
            *self.nodeIndicesInNodeGraph)
        self.deformationGraphDll.neighborNodeIndicesInNodeGraph = (ctypes.c_int * self.nodeEdgeNum)(
            *self.neighborNodeIndicesInNodeGraph)
        self.deformationGraphDll.weightInNodeGraph = (ctypes.c_float * self.nodeEdgeNum)(
            *self.weightInNodeGraph)

        self.deformationGraphDll.verticesNum = self.verticesNum
        self.deformationGraphDll.neighborNodeSum = self.neighborNodeSum
        self.deformationGraphDll.vertexIndicesInVertexGraph = (ctypes.c_int * self.neighborNodeSum)(
            *self.vertexIndicesInVertexGraph)
        self.deformationGraphDll.neighborNodeIndicesInVertexGraph = (ctypes.c_int * self.neighborNodeSum)(
            *self.neighborNodeIndicesInVertexGraph)
        self.deformationGraphDll.weightInVertexGraph = (ctypes.c_float * self.neighborNodeSum)(
            *self.weightInVertexGraph)
