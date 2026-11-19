import numpy as np, sys, os
from scipy import ndimage
from skimage import measure
from voxmesh import write_stl
from prims import WORK, TF
d=np.load(os.path.join(WORK,'parts_0.5.npz')); lo=d['lo']; h=float(d['h'])
E=np.load(os.path.join(WORK,'E_0.5.npz'))['E']; SO=np.load(os.path.join(WORK,'outer_sdf.npy'))
OUT=os.path.join(TF,'stl')
SIG=0.7; K=1/(np.sqrt(2*np.pi)*SIG)          # slope of a gaussian-smoothed step, per voxel
def g_of(mask):
    # signed distance (mm, + outside) of a voxel set, boundary half-way between voxels, lightly smoothed
    m = mask.astype(bool)
    dout = ndimage.distance_transform_edt(~m); din = ndimage.distance_transform_edt(m)
    g = np.where(m, -(din-0.5), dout-0.5).astype(np.float32)*h
    return ndimage.gaussian_filter(g, SIG)
OUTER={'front_panel','back_panel','top_panel','shoulder_block_L','shoulder_block_R','hatch_L','hatch_R'}
os.makedirs(OUT,exist_ok=True); tot=0
for k in d.files:
    if k in ('lo','h'): continue
    m=d[k]; idx=np.argwhere(m); a=np.maximum(idx.min(0)-5,0); b=np.minimum(idx.max(0)+6,m.shape)
    sl=tuple(slice(a[i],b[i]) for i in range(3)); P=m[sl]
    if k in OUTER:
        Ec=E[sl]; s=SO[sl]; sv=g_of(Ec); s=np.where(np.isnan(s),sv,s)
        f=np.maximum(s,g_of(P|~Ec))
        f[~ndimage.binary_dilation(P,iterations=4)]=h*3     # keep only this part
    else:
        f=g_of(P)
    f=np.pad(f,1,constant_values=h*3)
    v,fc,_,_=measure.marching_cubes(f,0.0,spacing=(h,)*3,step_size=int(sys.argv[1]) if len(sys.argv)>1 else 2)
    v=v-h+lo+a*h
    T=v[fc]; vol=np.einsum('ij,ij->i',T[:,0],np.cross(T[:,1],T[:,2])).sum()/6
    fn=os.path.join(OUT,f'{k}.stl'); write_stl(fn,v,fc,flip=vol<0); sz=os.path.getsize(fn); tot+=sz
    print(f'{k:18s} {len(fc):8d} tris {sz/1e6:6.1f} MB  vol {abs(vol)/1000:.1f} cm3 (voxels {m.sum()*h**3/1000:.1f})',flush=True)
print('total MB',tot/1e6)
