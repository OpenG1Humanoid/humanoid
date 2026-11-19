import sys, numpy as np, matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
from scipy import ndimage
import layout as L, os
from prims import WORK
d=np.load(os.path.join(WORK,'parts_0.5.npz')); lo=d['lo']; h=float(d['h'])
names=['front_panel','back_panel','top_panel','shoulder_block_L','shoulder_block_R','hatch_L','hatch_R','bottom_plate','compute_tray','driver_tray']
cols={'front_panel':(.82,.82,.85),'back_panel':(.7,.72,.78),'top_panel':(.9,.9,.9),'shoulder_block_L':(.95,.6,.2),'shoulder_block_R':(.95,.6,.2),
      'hatch_L':(.85,.35,.15),'hatch_R':(.85,.35,.15),'bottom_plate':(.25,.55,.85),'compute_tray':(.3,.75,.4),'driver_tray':(.75,.4,.75),'alu':(.55,.58,.62)}
def pool(m): # 0.5 -> 1 mm, any
    s=[(k//2)*2 for k in m.shape]; m=m[:s[0],:s[1],:s[2]]; return m.reshape(s[0]//2,2,s[1]//2,2,s[2]//2,2).any((1,3,5))
def labels(show):
    shp=pool(d['front_panel']).shape; lab=np.zeros(shp,np.uint8); pal=[(1,1,1)]
    hh=1.0; xs,ys,zs=[lo[i]+hh*np.arange(shp[i]) for i in range(3)]
    if 'alu' in show:
        a=np.zeros(shp,bool)
        for nm,b,ax in L.members():
            x0,x1,y0,y1,z0,z1=b; a[(xs>=x0)&(xs<=x1)][:,None,None] if False else None
            a|=((xs[:,None,None]>=x0)&(xs[:,None,None]<=x1)&(ys[None,:,None]>=y0)&(ys[None,:,None]<=y1)&(zs[None,None,:]>=z0)&(zs[None,None,:]<=z1))
        lab[a]=len(pal); pal.append(cols['alu'])
    for k in names:
        if k in show: lab[pool(d[k])]=len(pal); pal.append(cols[k])
    return lab,np.array(pal)
def view(lab,pal,az,el):
    a,e=np.radians(az),np.radians(el)
    Rz=np.array([[np.cos(a),-np.sin(a),0],[np.sin(a),np.cos(a),0],[0,0,1]]); Ry=np.array([[np.cos(e),0,np.sin(e)],[0,1,0],[-np.sin(e),0,np.cos(e)]])
    Rm=Rz@Ry; nn=np.array(lab.shape); m=int(np.linalg.norm(nn))+2
    off=(nn-1)/2-Rm@(np.array([m,m,m])/2)
    V=ndimage.affine_transform(lab,Rm,offset=off,output_shape=(m,m,m),order=0)
    occ=V>0; hit=occ.any(0); k=np.argmax(occ,0); lv=np.take_along_axis(V,k[None],0)[0]
    dd=ndimage.gaussian_filter(np.where(hit,k,k[hit].max()).astype(float),1.0)
    gy,gz=np.gradient(dd); nrm=np.stack([-np.ones_like(dd),gy,gz]); nrm/=np.linalg.norm(nrm,axis=0)
    Lg=np.array([-0.8,-0.3,0.55]); Lg/=np.linalg.norm(Lg); sh=0.35+0.65*np.clip((nrm*Lg[:,None,None]).sum(0),0,1)
    img=pal[lv]*sh[...,None]; img[~hit]=1; img=np.transpose(img,(1,0,2))[::-1,::-1]
    ys_,xs_=np.where(hit.T[::-1,::-1]); return img[ys_.min()-5:ys_.max()+5,xs_.min()-5:xs_.max()+5]
allp=names+['alu']
cfg=[(allp,180,10,'assembled, front'),(allp,30,15,'assembled, back'),
     ([n for n in allp if n not in('front_panel',)],210,15,'front panel off'),
     (['alu','bottom_plate','compute_tray','driver_tray','shoulder_block_L','shoulder_block_R','hatch_L','hatch_R'],215,25,'frame + shoulder blocks + trays'),
     (['alu','bottom_plate','compute_tray','driver_tray'],235,30,'frame, plate, trays'),
     (['top_panel','shoulder_block_L','shoulder_block_R','hatch_L','hatch_R','alu','compute_tray','driver_tray','bottom_plate'],200,-35,'front + back panels off, from below')]
fig,axs=plt.subplots(2,3,figsize=(24,16))
for ax,(show,az,el,t) in zip(axs.flat,cfg):
    lab,pal=labels(show); ax.imshow(np.clip(view(lab,pal,az,el),0,1)); ax.set_title(t,fontsize=16); ax.axis('off')
fig.tight_layout(); fig.savefig(sys.argv[1] if len(sys.argv)>1 else 'torso_preview.png',dpi=55)
