import sys,os; sys.path.insert(0,'/home/claude/tools')
import numpy as np, sdf
from sdf import Frame, cyl, capsule
S='/home/claude/stl_in/STL/legs/'; OUT='/home/claude/out/legs'; os.makedirs(OUT,exist_ok=True)
C=np.array([25.001,0,-124.12])      # yaw axis point on joint plane (roll-link frame)
TRIM=0.4; DEPTH=32.5; R=70.5/2
def build(fn, fnout, ops, h=0.3):
    T=sdf.load_stl(S+fn); v=T.reshape(-1,3); g=sdf.Grid(v.min(0),v.max(0),h,pad=3)
    s=sdf.Solid(g, sdf.mesh_sdf(T,g,3.0))
    for o in ops: s.cut(o)
    V,F=s.mesh(0.02,single=True); c=sdf.check(V,F); sdf.save_stl(f'{OUT}/{fnout}',V,F); print(fnout,c); return s
for side in ['left','right']:
    F0=Frame(np.eye(3),C)
    ops=[cyl(F0,48,-5,TRIM),                       # clearance gap above the yaw link
         cyl(F0,R,-5,TRIM+DEPTH),                    # actuator pocket (output faces down)
         cyl(F0,10,TRIM+DEPTH-1,TRIM+DEPTH+4),       # motor wire hole through the floor
         capsule(Frame(),C+[0,0,TRIM+DEPTH+2],C+[-60,0,TRIM+DEPTH+2],4.5)]  # wire channel out the back
    for k in range(12):
        a=np.radians(30*k); Fh=Frame(np.eye(3),C+[32*np.cos(a),32*np.sin(a),0])
        ops.append(cyl(Fh,2.0,TRIM+DEPTH-0.1,TRIM+DEPTH+7))   # M3 inserts for the 12 housing through-bolts
    build(f'{side}_hip_roll_link_integrated.stl',f'{side}_hip_roll_link_integrated_yaw.stl',ops)
    ops=[]
    for k in range(6):
        a=np.radians(60*k); Fh=Frame(np.eye(3),[17.3*np.cos(a),17.3*np.sin(a),0])
        ops.append(cyl(Fh,2.0,-7,0.5))                          # M3 inserts for the output-shaft bolts
    ops.append(cyl(Frame(),6.5,-4,0.5))                         # centre clearance under the output hub
    build(f'{side}_hip_yaw_link_knee_integrated.stl',f'{side}_hip_yaw_link_knee_integrated_yaw.stl',ops,h=0.4)
