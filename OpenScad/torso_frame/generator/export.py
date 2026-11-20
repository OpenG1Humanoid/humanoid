import numpy as np, sys, os
from scipy import ndimage
from skimage import measure
from voxmesh import write_stl
sys.path.insert(0,os.path.join(os.path.dirname(os.path.abspath(__file__)),'..','..','pyparts'))
from solid import finalize
from prims import WORK, TF
d=np.load(os.path.join(WORK,'parts_0.5.npz')); lo=d['lo']; h=float(d['h'])
E=np.load(os.path.join(WORK,'E_0.5.npz'))['E']
SO=None   # exact outer-surface field, loaded only when an outer part is exported
OUT=os.path.join(os.path.dirname(os.path.dirname(TF)),'STL','torso_frame')
SIG=0.7; K=1/(np.sqrt(2*np.pi)*SIG)          # slope of a gaussian-smoothed step, per voxel
def g_of(mask):
    # signed distance (mm, + outside) of a voxel set, boundary half-way between voxels, lightly smoothed
    m = mask.astype(bool)
    dout = ndimage.distance_transform_edt(~m); din = ndimage.distance_transform_edt(m)
    g = np.where(m, -(din-0.5), dout-0.5).astype(np.float32)*h
    return ndimage.gaussian_filter(g, SIG)
OUTER={'front_panel','back_panel','top_panel','shoulder_block_L','shoulder_block_R','hatch_L','hatch_R'}
os.makedirs(OUT,exist_ok=True); tot=0
ONLY=sys.argv[2:]   # optional part names
for k in d.files:
    if k in ('lo','h') or (ONLY and k not in ONLY): continue
    m=d[k]; idx=np.argwhere(m); a=np.maximum(idx.min(0)-5,0); b=np.minimum(idx.max(0)+6,m.shape)
    sl=tuple(slice(a[i],b[i]) for i in range(3)); P=m[sl]
    if k in OUTER:
        if SO is None: SO=np.load(os.path.join(WORK,'outer_sdf.npy'))
        Ec=E[sl]; s=SO[sl]; sv=g_of(Ec); s=np.where(np.isnan(s),sv,s)
        f=np.maximum(s,g_of(P|~Ec))
        f[~ndimage.binary_dilation(P,iterations=4)]=h*3     # keep only this part
    else:
        f=g_of(P)
    fn=os.path.join(OUT,f'{k}.stl')
    r=finalize(f,lo+a*h-0.5*h,h,fn,debris_cm3=0.05,step=int(sys.argv[1]) if len(sys.argv)>1 else 2)   # one closed piece, no crumbs or air pockets
    sz=os.path.getsize(fn); tot+=sz
    print(f'{k:18s} {r["triangles"]:8d} tris {sz/1e6:6.1f} MB  vol {r["volume_cm3"]:.1f} cm3 (voxels {m.sum()*h**3/1000:.1f})  crumbs {r["crumbs_removed"]} voids {r["voids_filled"]} bridged {r["bridged"]}',flush=True)
print('total MB',tot/1e6)
