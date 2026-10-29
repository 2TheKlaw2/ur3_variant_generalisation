import sys
from OCP.STEPControl import STEPControl_Reader
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.StlAPI import StlAPI_Writer
from OCP.Bnd import Bnd_Box
from OCP.BRepBndLib import BRepBndLib
import math
src,dst,lin,ang=sys.argv[1],sys.argv[2],float(sys.argv[3]),float(sys.argv[4])
r=STEPControl_Reader(); assert r.ReadFile(src)==1; r.TransferRoots(); s=r.OneShape()
BRepMesh_IncrementalMesh(s,lin,False,math.radians(ang),True)
w=StlAPI_Writer(); w.ASCIIMode=False; w.Write(s,dst)
print("ok")
