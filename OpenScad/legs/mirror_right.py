"""Make the right leg STLs from the left ones (mirror in y, the G1's left/right symmetry plane).

    python mirror_right.py            # reads/writes ../../STL/legs/

Unitree's own right-leg meshes are the left ones mirrored in y (checked to < 0.1 mm), and the
right-leg URDF joints are the left ones with y negated, so each mirrored part sits in the right
link's own frame. The actuator parts themselves are not mirrored: the same actuator is used.
"""
import os, sys, struct
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
D = sys.argv[1] if len(sys.argv) > 1 else os.path.join(HERE, '..', '..', 'STL', 'legs')
PARTS = ['hip_pitch_link_integrated', 'hip_roll_link_integrated', 'hip_yaw_link_knee_integrated', 'knee_link_integrated']

def read(p):
    b = open(p, 'rb').read()
    if b[:5] == b'solid' and b'facet' in b[:300]:
        import re
        return np.array(re.findall(rb'vertex\s+(\S+)\s+(\S+)\s+(\S+)', b), float).reshape(-1, 3, 3)
    n = int.from_bytes(b[80:84], 'little')
    return np.frombuffer(b[84:84+50*n], dtype=np.dtype([('n', '<3f4'), ('v', '<9f4'), ('a', '<u2')]))['v'].reshape(-1, 3, 3).astype(float)

def write(p, T, header):
    N = np.cross(T[:, 1]-T[:, 0], T[:, 2]-T[:, 0]); N /= np.linalg.norm(N, axis=1, keepdims=True) + 1e-12
    rec = np.zeros(len(T), dtype=np.dtype([('n', '<3f4'), ('v', '<9f4'), ('a', '<u2')]))
    rec['n'] = N; rec['v'] = T.reshape(-1, 9)
    with open(p, 'wb') as f:
        f.write(header.encode()[:80].ljust(80, b' ')); f.write(struct.pack('<I', len(T))); f.write(rec.tobytes())

for k in PARTS:
    T = read(os.path.join(D, f'left_{k}.stl'))
    M = T.copy(); M[:, :, 1] *= -1; M = M[:, [0, 2, 1]]        # mirror y, flip winding to keep normals outward
    write(os.path.join(D, f'right_{k}.stl'), M, f'right_{k}: mirror of left_{k} in y (mirror_right.py)')
    vol = lambda X: np.einsum('ij,ij->i', X[:, 0], np.cross(X[:, 1], X[:, 2])).sum()/6
    print(f'right_{k}.stl  {len(M)} triangles  volume {vol(M)/1000:.1f} cm3 (left {vol(T)/1000:.1f})')
