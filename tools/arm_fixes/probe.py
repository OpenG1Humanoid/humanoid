import numpy as np
def ray_hits(T, p, d):
    """all hit parameters t of ray p+t d with triangles (Moller-Trumbore), sorted."""
    p=np.asarray(p,float); d=np.asarray(d,float)
    a,b,c=T[:,0],T[:,1],T[:,2]; e1=b-a; e2=c-a
    pv=np.cross(d,e2); det=(e1*pv).sum(1); ok=np.abs(det)>1e-12
    inv=np.where(ok,1/np.where(ok,det,1),0); tv=p-a
    u=(tv*pv).sum(1)*inv; qv=np.cross(tv,e1); v=(qv*d).sum(1)*inv; t=(e2*qv).sum(1)*inv
    m=ok&(u>=0)&(v>=0)&(u+v<=1)
    return np.sort(t[m])
