"""Outer envelope E of the original torso (filled), at h mm. Saves E.npz"""
import numpy as np, sys, os
from scipy import ndimage
from prims import torso_tris, occupancy, WORK
h=float(sys.argv[1]) if len(sys.argv)>1 else 0.5
T=torso_tris()
lo=np.array([-72,-112,-14.]); hi=np.array([90,112,316.])
occ=occupancy(T,lo,hi,h)
# seal small gaps, then fill each z-slice (the skin is a closed ring in most slices)
st=ndimage.generate_binary_structure(2,1)
cl=ndimage.binary_closing(occ, structure=np.ones((3,3,3),bool), iterations=2)|occ
E=np.zeros_like(occ)
for k in range(occ.shape[2]): E[:,:,k]=ndimage.binary_fill_holes(cl[:,:,k])
# also fill along x and y slices (top/bottom caps, side pockets)
for i in range(occ.shape[0]): E[i]|=ndimage.binary_fill_holes(cl[i])
E=ndimage.binary_fill_holes(E)
# keep the original surface detail: closing/fill only counts >1.5 mm below the outside
Do=ndimage.distance_transform_edt(E)*h
E=occ|(E&(Do>1.5)); del Do
E=ndimage.binary_fill_holes(E)
np.savez_compressed(os.path.join(WORK,f'E_{h}.npz'),E=E,occ=occ,lo=lo+h/2,h=h)   # lo = position of sample [0,0,0]
print(occ.shape, 'occ cm3',occ.sum()*h**3/1000,'E cm3',E.sum()*h**3/1000)
