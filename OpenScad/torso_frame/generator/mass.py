"""Framed torso mass properties (torso_link frame, metres) -> torso_mass.json"""
import numpy as np, json, os
import layout as L
from prims import WORK, TF
d=np.load(os.path.join(WORK,'parts_0.5.npz')); lo=d['lo']; h=float(d['h']); scut=np.load(os.path.join(WORK,'aux.npz'))['scut']
n=d['front_panel'].shape; ys=lo[1]+h*np.arange(n[1])
RHO=1100.0   # kg/m3: 3-4 mm walls print almost solid (PLA 1240 / PETG 1270 at ~90 %)
KGM=0.50     # 2020 extrusion, kg/m
items=[]
def vox_item(m,mass):
    P=(lo+np.argwhere(m)*h)/1000; c=P.mean(0); dP=P-c
    return mass,c,mass/len(P)*(np.eye(3)*(dP*dP).sum()-dP.T@dP)
def box_item(b,mass):
    x0,x1,y0,y1,z0,z1=[v/1000 for v in b]; a,bb,cc=x1-x0,y1-y0,z1-z0
    return mass,np.array([(x0+x1)/2,(y0+y1)/2,(z0+z1)/2]),mass/12*np.diag([bb*bb+cc*cc,a*a+cc*cc,a*a+bb*bb])
pm={}
for k in d.files:
    if k in ('lo','h'): continue
    pm[k]=d[k].sum()*(h/1000)**3*RHO; items.append(vox_item(d[k],pm[k]))
alu_len=0
for nm,b,ax in L.members():
    ln={'x':b[1]-b[0],'y':b[3]-b[2],'z':b[5]-b[4]}[ax]; alu_len+=ln; items.append(box_item(b,KGM*ln/1000))
for s in (1,-1):                                   # DS3235 60 g in each shoulder block
    c=(lo+np.argwhere(scut&((ys[None,:,None]*s)>0))*h).mean(0)
    items.append(box_item((c[0]-10,c[0]+10,c[1]-20,c[1]+20,c[2]-19,c[2]+19),0.060))
items.append(box_item((L.XB-10,L.XF+10,-L.YP-10,L.YP+10,L.Z_PLATE[1],L.Z_TOPBAR[1]),0.25))   # screws, T-nuts, brackets
M=sum(i[0] for i in items); com=sum(i[0]*i[1] for i in items)/M; I=np.zeros((3,3))
for m_,c,Ii in items:
    dd=c-com; I+=Ii+m_*(np.eye(3)*(dd@dd)-np.outer(dd,dd))
out=dict(mass=M,com=com.tolist(),I=I.tolist(),printed_kg={k:round(v,4) for k,v in pm.items()},printed_total_kg=sum(pm.values()),
         aluminium_m=alu_len/1000,aluminium_kg=KGM*alu_len/1000,servos_kg=0.12,fasteners_kg=0.25,
         note="framed torso: printed parts at 1.1 g/cm3, 2020 at 0.5 kg/m, 2x DS3235, fasteners/brackets 0.25 kg. Electronics NOT included.")
json.dump(out,open(os.path.join(TF,'torso_mass.json'),'w'),indent=1)
print('total %.3f kg  printed %.3f  alu %.3f m / %.3f kg'%(M,sum(pm.values()),alu_len/1000,KGM*alu_len/1000)); print('com',np.round(com,4)); print('I eig',np.linalg.eigvalsh(I)); print(np.round(I,5))
