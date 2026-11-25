"""Edit closed STL meshes with CSG: SDF voxelise -> analytic cuts/fills -> marching cubes -> QEM decimate."""
import ctypes, numpy as np, os, time
from skimage.measure import marching_cubes
from stlio import load, save

_lib = ctypes.CDLL(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'libgeom.so'))
_dp = ctypes.POINTER(ctypes.c_double); _ip = ctypes.POINTER(ctypes.c_int); _fp = ctypes.POINTER(ctypes.c_float)
_lib.sdf_grid.argtypes = [_dp, ctypes.c_int, _dp, ctypes.c_double, _ip, ctypes.c_double, _fp]
_lib.decimate.argtypes = [_dp, ctypes.c_int, _ip, ctypes.c_int, ctypes.c_double, ctypes.c_int, ctypes.c_double, _ip]
_lib.decimate.restype = ctypes.c_int


class Grid:
    def __init__(self, T, h, pad=3):
        V = T.reshape(-1, 3)
        lo = V.min(0) - pad * h; hi = V.max(0) + pad * h
        self.h = h
        self.n = np.ceil((hi - lo) / h).astype(np.int32) + 1
        self.o = lo + np.array([0.123e-3, 0.271e-3, 0.0379e-3])  # avoid exact alignment with faces
        self.band = 3.0 * h
        tr = np.ascontiguousarray(T.reshape(-1), dtype=np.float64)
        self.d = np.empty(tuple(self.n), dtype=np.float32)
        o = np.ascontiguousarray(self.o, dtype=np.float64); n = np.ascontiguousarray(self.n, dtype=np.int32)
        _lib.sdf_grid(tr.ctypes.data_as(_dp), len(T), o.ctypes.data_as(_dp), h, n.ctypes.data_as(_ip),
                      self.band, self.d.ctypes.data_as(_fp))

    def block(self, lo, hi):
        """index slices + local coordinates (world) for a world-space AABB."""
        i0 = np.clip(np.floor((np.asarray(lo) - self.o) / self.h).astype(int) - 1, 0, self.n - 1)
        i1 = np.clip(np.ceil((np.asarray(hi) - self.o) / self.h).astype(int) + 2, 0, self.n)
        sl = tuple(slice(a, b) for a, b in zip(i0, i1))
        ax = [self.o[k] + self.h * np.arange(i0[k], i1[k]) for k in range(3)]
        X, Y, Z = np.meshgrid(*ax, indexing='ij')
        return sl, np.stack([X, Y, Z], -1)

    def apply(self, prim, mode):
        lo, hi = prim.aabb()
        if mode == 'fill':  # grid must cover the fill
            pass
        sl, P = self.block(lo, hi)
        if P.size == 0: return
        s = prim.sdf(P).astype(np.float32)
        if mode == 'cut':
            self.d[sl] = np.maximum(self.d[sl], -s)
        else:
            self.d[sl] = np.minimum(self.d[sl], s)

    def mesh(self, max_err=0.02, min_faces=2000, min_cos=0.3):
        vs, fs, off = [], [], 0
        nx = self.d.shape[0]; step = 120
        for a in range(0, nx - 1, step):
            b = min(nx, a + step + 1)
            sub = self.d[a:b]
            if sub.min() > 0 or sub.max() < 0: continue
            v, f, _, _ = marching_cubes(sub, level=0.0, allow_degenerate=False)
            v = v.astype(np.float64); v[:, 0] += a
            vs.append(v); fs.append(f + off); off += len(v)
        v = np.concatenate(vs); f = np.concatenate(fs)
        v, f = weld(v, f, scale=1e4)
        v = v * self.h + self.o
        V = np.ascontiguousarray(v); F = np.ascontiguousarray(f, dtype=np.int32)
        nvo = ctypes.c_int(0)
        m = _lib.decimate(V.ctypes.data_as(_dp), len(V), F.ctypes.data_as(_ip), len(F), max_err, min_faces, min_cos,
                          ctypes.byref(nvo))
        V = V[:nvo.value].copy(); F = F[:m].copy()
        nc, lab = components(V, F)
        if nc > 1:  # drop specks (degenerate MC fragments)
            T = V[F]; fl = lab[F[:, 0]]
            vol = np.einsum('ij,ij->i', T[:, 0], np.cross(T[:, 1], T[:, 2])) / 6
            cv = np.bincount(fl, weights=vol, minlength=nc)
            keep = np.abs(cv) > 0.5
            F = F[keep[fl]]
            V, F = weld(V[F].reshape(-1, 3), np.arange(len(F) * 3).reshape(-1, 3))
        return V, F


def weld(v, f, scale=1e6):
    key = np.round(v * scale).astype(np.int64)
    _, idx, inv = np.unique(key, axis=0, return_index=True, return_inverse=True)
    inv = inv.reshape(-1)
    f = inv[f]
    good = (f[:, 0] != f[:, 1]) & (f[:, 1] != f[:, 2]) & (f[:, 0] != f[:, 2])
    return v[idx], f[good]


# ------------------------------------------------------------------ primitives (local frame -> world)
class Prim:
    """M: 4x4 local->world. sdf_local works in local coords."""
    def __init__(self, M=None):
        self.M = np.eye(4) if M is None else np.asarray(M, float)
        self.Mi = np.linalg.inv(self.M)

    def sdf(self, P):
        L = P @ self.Mi[:3, :3].T + self.Mi[:3, 3]
        return self.sdf_local(L)

    def aabb(self):
        lo, hi = self.local_box()
        c = np.array([[x, y, z] for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])])
        w = c @ self.M[:3, :3].T + self.M[:3, 3]
        return w.min(0) - 0.5, w.max(0) + 0.5


class Box(Prim):
    def __init__(self, lo, hi, M=None, r=0.0):
        super().__init__(M); self.lo = np.asarray(lo, float); self.hi = np.asarray(hi, float); self.r = r

    def local_box(self): return self.lo, self.hi

    def sdf_local(self, L):
        c = (self.lo + self.hi) / 2; e = (self.hi - self.lo) / 2 - self.r
        q = np.abs(L - c) - e
        return np.linalg.norm(np.maximum(q, 0), axis=-1) + np.minimum(q.max(-1), 0) - self.r


class Cyl(Prim):
    """cylinder along local z, centre (cx,cy), radius r, z0..z1; optional inner radius ri (tube)."""
    def __init__(self, cx, cy, r, z0, z1, M=None, ri=0.0):
        super().__init__(M); self.c = (cx, cy); self.r = r; self.z = (z0, z1); self.ri = ri

    def local_box(self):
        return (np.array([self.c[0] - self.r, self.c[1] - self.r, self.z[0]]),
                np.array([self.c[0] + self.r, self.c[1] + self.r, self.z[1]]))

    def sdf_local(self, L):
        rr = np.hypot(L[..., 0] - self.c[0], L[..., 1] - self.c[1])
        d2 = rr - self.r
        if self.ri > 0: d2 = np.maximum(d2, self.ri - rr)
        zc = (self.z[0] + self.z[1]) / 2; hz = (self.z[1] - self.z[0]) / 2
        dz = np.abs(L[..., 2] - zc) - hz
        return np.minimum(np.maximum(d2, dz), 0) + np.hypot(np.maximum(d2, 0), np.maximum(dz, 0))


def _uneven_capsule(px, py, r1, r2, h):
    """iq's sdUnevenCapsule: circle r1 at origin, circle r2 at (0,h)."""
    px = np.abs(px)
    b = (r1 - r2) / h; a = np.sqrt(max(1 - b * b, 0)); k = -px * b + py * a  # dot((-b,a),(px,py))
    c1 = np.hypot(px, py) - r1
    c2 = np.hypot(px, py - h) - r2
    c3 = px * a + py * b - r1
    return np.where(k < 0, c1, np.where(k > a * h, c2, c3))


class Horn(Prim):
    """Cross-horn pocket, extruded along local z from z0 to z1. Local origin = spline axis.
    hub_d: hub diameter; arms: list of (angle_deg, reach, w_root, w_tip)."""
    def __init__(self, hub_d, arms, z0, z1, M=None):
        super().__init__(M); self.hub = hub_d / 2; self.arms = arms; self.z = (z0, z1)
        self.R = max(max(a[1] for a in arms), self.hub)

    def local_box(self):
        return np.array([-self.R, -self.R, self.z[0]]), np.array([self.R, self.R, self.z[1]])

    def sdf2(self, x, y):
        d = np.hypot(x, y) - self.hub
        for ang, reach, w0, w1 in self.arms:
            t = np.radians(ang)
            # rotate so the arm points along +y
            u = x * np.cos(t - np.pi / 2) + y * np.sin(t - np.pi / 2)
            v = -x * np.sin(t - np.pi / 2) + y * np.cos(t - np.pi / 2)
            d = np.minimum(d, _uneven_capsule(u, v, w0 / 2, w1 / 2, reach - w1 / 2))
        return d

    def sdf_local(self, L):
        d2 = self.sdf2(L[..., 0], L[..., 1])
        zc = (self.z[0] + self.z[1]) / 2; hz = (self.z[1] - self.z[0]) / 2
        dz = np.abs(L[..., 2] - zc) - hz
        return np.minimum(np.maximum(d2, dz), 0) + np.hypot(np.maximum(d2, 0), np.maximum(dz, 0))


def _poly_sdf(px, py, pts):
    """exact signed distance to a simple polygon (iq), pts: (N,2)"""
    d = np.full(px.shape, np.inf); sgn = np.ones(px.shape)
    N = len(pts)
    for i in range(N):
        vi = pts[i]; vj = pts[i - 1]
        ex, ey = vj[0] - vi[0], vj[1] - vi[1]
        wx, wy = px - vi[0], py - vi[1]
        t = np.clip((wx * ex + wy * ey) / (ex * ex + ey * ey), 0, 1)
        bx, by = wx - ex * t, wy - ey * t
        d = np.minimum(d, bx * bx + by * by)
        c1 = py >= vi[1]; c2 = py < vj[1]; c3 = ex * wy > ey * wx
        flip = (c1 & c2 & c3) | (~c1 & ~c2 & ~c3)
        sgn = np.where(flip, -sgn, sgn)
    return sgn * np.sqrt(d)


class Poly(Prim):
    """2D polygon (local xy) extruded z0..z1"""
    def __init__(self, pts, z0, z1, M=None):
        super().__init__(M); self.pts = np.asarray(pts, float); self.z = (z0, z1)

    def local_box(self):
        return (np.r_[self.pts.min(0), self.z[0]], np.r_[self.pts.max(0), self.z[1]])

    def sdf_local(self, L):
        d2 = _poly_sdf(L[..., 0], L[..., 1], self.pts)
        zc = (self.z[0] + self.z[1]) / 2; hz = (self.z[1] - self.z[0]) / 2
        dz = np.abs(L[..., 2] - zc) - hz
        return np.minimum(np.maximum(d2, dz), 0) + np.hypot(np.maximum(d2, 0), np.maximum(dz, 0))


def sector_pts(cx, cy, r0, r1, a0, a1, n=48):
    """annular sector polygon, angles in degrees (a0->a1 counter-clockwise if a1>a0)"""
    a = np.radians(np.linspace(a0, a1, n))
    outer = np.c_[cx + r1 * np.cos(a), cy + r1 * np.sin(a)]
    inner = np.c_[cx + r0 * np.cos(a[::-1]), cy + r0 * np.sin(a[::-1])]
    return np.r_[outer, inner]


class Intersect(Prim):
    def __init__(self, *ps): self.ps = ps
    def sdf(self, P): return np.max([p.sdf(P) for p in self.ps], axis=0)
    def aabb(self):
        bs = [p.aabb() for p in self.ps]
        return np.max([b[0] for b in bs], 0), np.min([b[1] for b in bs], 0)


class Subtract(Prim):
    """a minus b"""
    def __init__(self, a, b): self.a, self.b = a, b
    def sdf(self, P): return np.maximum(self.a.sdf(P), -self.b.sdf(P))
    def aabb(self): return self.a.aabb()


# ------------------------------------------------------------------ checks
def components(V, F):
    import scipy.sparse as sp, scipy.sparse.csgraph as cg
    De = np.concatenate([F[:, [0, 1]], F[:, [1, 2]], F[:, [2, 0]]])
    n = len(V)
    A = sp.coo_matrix((np.ones(len(De)), (De[:, 0], De[:, 1])), shape=(n, n))
    return cg.connected_components(A, directed=False)


def check(V, F):
    E = np.sort(np.concatenate([F[:, [0, 1]], F[:, [1, 2]], F[:, [2, 0]]]), axis=1)
    _, cnt = np.unique(E, axis=0, return_counts=True)
    De = np.concatenate([F[:, [0, 1]], F[:, [1, 2]], F[:, [2, 0]]])
    _, dcnt = np.unique(De, axis=0, return_counts=True)
    watertight = bool((cnt == 2).all()); oriented = bool((dcnt == 1).all())
    # components
    import scipy.sparse as sp, scipy.sparse.csgraph as cg
    n = len(V)
    A = sp.coo_matrix((np.ones(len(De)), (De[:, 0], De[:, 1])), shape=(n, n))
    ncomp, _ = cg.connected_components(A, directed=False)
    T = V[F]
    vol = np.einsum('ij,ij->i', T[:, 0], np.cross(T[:, 1], T[:, 2])).sum() / 6
    chi = n - len(cnt) + len(F)
    return dict(tris=len(F), watertight=watertight, oriented=oriented, shells=int(ncomp), volume=round(vol, 1),
                genus=int((2 * ncomp - chi) // 2))


def run(T, ops, h, max_err=0.02, log=None):
    t0 = time.time()
    G = Grid(T, h)
    for mode, p in ops:
        G.apply(p, mode)
    V, F = G.mesh(max_err=max_err)
    info = check(V, F); info['t'] = round(time.time() - t0, 1)
    if log: print(log, info, flush=True)
    return V[F], info, G
