import ctypes
import dataStructure

autoDispenserDLL = ctypes.CDLL(
    'D:\\nutCloud\\我的坚果云\\workFiles\\projects\\2022Zhangjiagang_Shichun\\codes\\autoDispenser2023\\autoDispenser2023\\x64\\Debug\\autoDispDll.dll', )

getAlignTransMatriceDll = autoDispenserDLL.getAlignTransMatrice
getAlignTransMatriceDll.argtypes = [ctypes.c_char_p, ctypes.c_char_p, ctypes.c_char_p, ctypes.c_char_p,
                                    ctypes.POINTER(ctypes.c_double)]

getTemplateFilesDll = autoDispenserDLL.getTemplateFiles
getTemplateFilesDll.argtypes = [ctypes.c_char_p, ctypes.c_char_p, ctypes.c_char_p, ctypes.c_char_p, ctypes.c_char_p]

generateTrajectoryDll2 = autoDispenserDLL.generateTrajectory2
generateTrajectoryDll2.argtypes = [dataStructure.DeformationGraphDll, ctypes.c_void_p, ctypes.c_void_p, ctypes.c_void_p,
                                   ctypes.c_void_p, ctypes.c_void_p, ctypes.c_char_p, ctypes.c_char_p, ctypes.c_char_p,
                                   ctypes.c_char_p,ctypes.c_char_p]

generateTrajectoryDll = autoDispenserDLL.generateTrajectory
generateTrajectoryDll.argtypes = [dataStructure.DeformationGraphDll, ctypes.c_void_p, ctypes.c_char_p, ctypes.c_char_p,
                                  ctypes.c_char_p, ctypes.c_char_p, ctypes.c_char_p, ctypes.c_char_p, ctypes.c_char_p,
                                  ctypes.c_char_p]

loadTemplateMeshDll = autoDispenserDLL.loadTemplateMesh
loadTemplateMeshDll.argtypes = [ctypes.c_char_p, ctypes.c_double]
loadTemplateMeshDll.restype = ctypes.c_void_p

readO3dFromFileDll = autoDispenserDLL.readO3dFromFile
readO3dFromFileDll.argtypes = [ctypes.c_char_p]
readO3dFromFileDll.restype = ctypes.c_void_p
