import sys; sys.path.insert(0,'/home/claude/work')
import numpy as np, stlio, newyaw as N, asm3
from reinforce import occupancy
from asm import mods
side=sys.argv[1]; asm3.side=side
h=0.2; y0=N.Y0[side]; lo=np.array([-40,y0-40,-165.]); n=np.array([400,400,600])
def occ(T): return occupancy(T,lo,n,h)
P=asm3.parts(0); O={k:occ(T) for k,T in P.items()}
pairs=[('roll','yaw'),('roll','cart'),('roll','horn'),('yaw','cart'),('yaw','horn'),('cart','horn')]
print('static overlaps mm3:',{f'{a}/{b}':round((O[a]&O[b]).sum()*h**3,2) for a,b in pairs})
for yd in [60,150,-150]:
    Q=asm3.parts(yd); Oy=occ(Q['yaw']); Oc=occ(Q['cart'])
    print('yaw',yd,'roll/yaw',round((O['roll']&Oy).sum()*h**3,2),'roll/cart',round((O['roll']&Oc).sum()*h**3,2),'horn/cart',round((O['horn']&Oc).sum()*h**3,2),'horn/yaw',round((O['horn']&Oy).sum()*h**3,2))
def shift(A,k):
    B=np.zeros_like(A)
    if k>0: B[:,:,k:]=A[:,:,:-k]
    else: B[:,:,:k]=A[:,:,-k:]
    return B
w=0
for k in range(1,int(45/h),3): w=max(w,(shift(O['cart'],k)&O['yaw']).sum()*h**3)
print('cartridge lifted out of upper arm, worst clash mm3',round(w,2))
w=0
for k in range(1,int(12/h)): w=max(w,(shift(O['horn'],-k)&O['roll']).sum()*h**3)
print('horn plate key pulled down out of roll socket, worst clash mm3',round(w,2))
# elbow parts
m=mods(side); yl=m[f'{side}_shoulder_yaw_link']
def tr(T,M): return (T.reshape(-1,3)@M[:3,:3].T+M[:3,3]).reshape(-1,3,3)
for k in ['elbow_cartridge','elbow_cap']:
    f=f'/home/claude/micro/STL/arms/{side}/actuators/elbow/{side}_{k}.stl'
    import os
    if not os.path.exists(f): f=f'/home/claude/openg1humanoid/humanoid/STL/arms/{side}/actuators/elbow/{side}_{k}.stl'
    E=occ(tr(stlio.load(f),N.Y2R(side)@yl['parts'][f'{side}_{k}']))
    print(k,'vs yaw link',round((E&O['yaw']).sum()*h**3,2),'vs shoulder cartridge',round((E&O['cart']).sum()*h**3,2))
