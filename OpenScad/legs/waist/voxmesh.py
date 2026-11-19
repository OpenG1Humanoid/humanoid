"""Mesh <-> voxel helpers: winding-number scanline voxeliser, marching-cubes export."""
import numpy as np
from skimage import measure
from scipy import ndimage

def load_stl_mm(p):
    b = open(p, 'rb').read(); n = np.frombuffer(b[80:84], '<u4')[0]
    d = np.frombuffer(b[84:84 + n * 50], dtype=np.dtype([('n', '<f4', 3), ('v', '<f4', (3, 3)), ('a', '<u2')]))
    return d['v'].astype(float) * 1000.0

def voxelize(V, grid):
    """inside = winding number > 0, rays along +x through every (y,z) grid column"""
    nx, ny, nz = grid.n; res = grid.res
    D = np.zeros((nx + 1, ny, nz), np.int16)
    v0, v1, v2 = V[:, 0], V[:, 1], V[:, 2]
    nxs = np.cross(v1 - v0, v2 - v0)[:, 0]
    y0, z0 = grid.lo[1], grid.lo[2]
    for t in range(len(V)):
        if abs(nxs[t]) < 1e-12: continue
        a, b, c = V[t]
        ylo, yhi = min(a[1], b[1], c[1]), max(a[1], b[1], c[1])
        zlo, zhi = min(a[2], b[2], c[2]), max(a[2], b[2], c[2])
        j0 = max(0, int(np.ceil((ylo - y0) / res))); j1 = min(ny - 1, int(np.floor((yhi - y0) / res)))
        k0 = max(0, int(np.ceil((zlo - z0) / res))); k1 = min(nz - 1, int(np.floor((zhi - z0) / res)))
        if j1 < j0 or k1 < k0: continue
        J, K = np.meshgrid(np.arange(j0, j1 + 1), np.arange(k0, k1 + 1), indexing='ij')
        py = y0 + J * res; pz = z0 + K * res
        # barycentric in yz
        d = (b[1] - a[1]) * (c[2] - a[2]) - (c[1] - a[1]) * (b[2] - a[2])
        l1 = ((py - a[1]) * (c[2] - a[2]) - (c[1] - a[1]) * (pz - a[2])) / d
        l2 = ((b[1] - a[1]) * (pz - a[2]) - (py - a[1]) * (b[2] - a[2])) / d
        l0 = 1 - l1 - l2
        m = (l0 >= 0) & (l1 >= 0) & (l2 >= 0)
        if not m.any(): continue
        xs = l0[m] * a[0] + l1[m] * b[0] + l2[m] * c[0]
        ix = np.clip(np.ceil((xs - grid.lo[0]) / res).astype(int), 0, nx)
        s = 1 if nxs[t] < 0 else -1
        np.add.at(D, (ix, J[m], K[m]), s)
    W = np.cumsum(D, axis=0)[:nx]
    return W > 0

def to_mesh(vox, grid, smooth=0.6, step=1):
    f = ndimage.gaussian_filter(vox.astype(np.float32), smooth) if smooth else vox.astype(np.float32)
    f = np.pad(f, 1)
    verts, faces, _, _ = measure.marching_cubes(f, 0.5, spacing=(grid.res,) * 3, step_size=step)
    verts = verts - grid.res + grid.lo
    return verts, faces

def write_stl(path, verts, faces, flip=False):
    T = verts[faces]
    if flip: T = T[:, ::-1]
    n = np.cross(T[:, 1] - T[:, 0], T[:, 2] - T[:, 0]); n /= np.linalg.norm(n, axis=1)[:, None] + 1e-12
    rec = np.zeros(len(T), dtype=np.dtype([('n', '<f4', 3), ('v', '<f4', (3, 3)), ('a', '<u2')]))
    rec['n'] = n; rec['v'] = T
    with open(path, 'wb') as fh:
        fh.write(b'G1 arm link, modified for servo joints'.ljust(80, b' '))
        fh.write(np.uint32(len(T)).tobytes()); fh.write(rec.tobytes())
