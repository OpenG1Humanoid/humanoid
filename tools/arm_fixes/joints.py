import numpy as np, sys
import os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from asm import mods
from stlio import load
ROOT=os.environ.get('ARMS_SRC', os.path.join(os.path.dirname(os.path.abspath(__file__)), 'original_arms')) + '/'
# joint -> (host link module suffix, servo type)
JOINTS={'shoulder_roll':('shoulder_pitch_link','std'),'shoulder_yaw':('shoulder_roll_link','micro'),
        'elbow':('shoulder_yaw_link','std'),'wrist_roll':('elbow_link','micro'),
        'wrist_pitch':('wrist_pitch_link','micro'),'wrist_yaw':('wrist_pitch_link','micro')}
def parts_in_cart(side,joint):
    D=mods(side); host=D[f'{side}_{JOINTS[joint][0]}']
    Pc=host['parts'][f'{side}_{joint}_cartridge']; Ci=np.linalg.inv(Pc)
    out={}
    out['link']=(ROOT+f'{side}/links/{side}_{JOINTS[joint][0]}.stl', Ci)
    for pn,P in host['parts'].items():
        if pn.startswith(f'{side}_{joint}_'):
            out[pn[len(side)+1+len(joint)+1:]]=(ROOT+f'{side}/actuators/{joint}/{pn}.stl', Ci@P)
    return out
def xf(T,M): 
    V=T.reshape(-1,3)@M[:3,:3].T+M[:3,3]; return V.reshape(-1,3,3)
