import numpy as np, struct
def load(p):
    b=open(p,'rb').read()
    if b[:5]==b'solid' and b'facet' in b[:400]:
        v=[]; 
        for l in b.decode().splitlines():
            l=l.strip()
            if l.startswith('vertex'): v.append([float(x) for x in l.split()[1:]])
        return np.array(v).reshape(-1,3,3)
    n=struct.unpack('<I',b[80:84])[0]
    a=np.frombuffer(b[84:84+n*50],dtype=np.dtype([('n','<3f4'),('v','<9f4'),('a','<u2')]))
    return a['v'].reshape(-1,3,3).astype(np.float64)
def save(p,T):
    T=np.asarray(T,dtype=np.float32)
    n=np.cross(T[:,1]-T[:,0],T[:,2]-T[:,0]); l=np.linalg.norm(n,axis=1,keepdims=True); l[l==0]=1; n=n/l
    a=np.zeros(len(T),dtype=np.dtype([('n','<3f4'),('v','<9f4'),('a','<u2')]))
    a['n']=n; a['v']=T.reshape(-1,9)
    with open(p,'wb') as f:
        f.write(b'OpenG1'.ljust(80,b' ')); f.write(struct.pack('<I',len(T))); f.write(a.tobytes())
