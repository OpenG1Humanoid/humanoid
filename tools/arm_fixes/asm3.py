import sys; sys.path.insert(0,'/home/claude/work')
import os, numpy as np, stlio, newyaw as N
side=sys.argv[1] if len(sys.argv)>1 else 'left'
O=os.environ.get('NEWYAW_OUT', '/home/claude/final3/STL/arms') + '/'
def tr(T,M): return (T.reshape(-1,3)@M[:3,:3].T+M[:3,3]).reshape(-1,3,3)
def parts(yaw_deg=0.0):
    Rz=N.Rz(yaw_deg); Y2R=N.Y2R(side)
    P={'roll':stlio.load(O+f'{side}/links/{side}_shoulder_roll_link.stl'),
       'yaw':tr(stlio.load(O+f'{side}/links/{side}_shoulder_yaw_link.stl'),Y2R@Rz),
       'cart':tr(stlio.load(O+f'{side}/actuators/shoulder_yaw/{side}_shoulder_yaw_cartridge.stl'),Y2R@Rz@N.G_c(side)),
       'horn':tr(stlio.load(O+f'{side}/actuators/shoulder_yaw/{side}_shoulder_yaw_hornplate.stl'),Y2R@N.G_h(side))}
    return P
