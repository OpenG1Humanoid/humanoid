"""Lug columns for the micro shoulder-yaw cartridge and the matching channels in the roll link.
The cartridge's own lug/arm outline (at lug height) is extruded down onto its bottom ring, and the roll link's
own lug pocket outline (which already fits the cartridge with clearance) is extruded down through the bore, so
the cartridge slides straight up into place and there is no gap under the lugs."""
import sys, os; sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np
from scipy import ndimage as ndi
from scipy.ndimage import map_coordinates
from meshedit import Prim
from sec import section
from matplotlib.path import Path

H2 = 0.05

def mask2d(T, z, lo, n):
    """solid mask of mesh T in the plane z (even-odd fill of the section loops)"""
    S = section(T, 2, z)                       # segments (k,2,2)
    xs = lo[0] + H2 * (np.arange(n[0]) + 0.5); ys = lo[1] + H2 * (np.arange(n[1]) + 0.5)
    X, Y = np.meshgrid(xs, ys, indexing='ij')
    # even-odd ray cast along +x
    cnt = np.zeros(X.shape, np.int32)
    a, b = S[:, 0], S[:, 1]
    for (x0, y0), (x1, y1) in zip(a, b):
        if y0 == y1: continue
        if y0 > y1: x0, y0, x1, y1 = x1, y1, x0, y0
        j0 = int(np.ceil((y0 - lo[1]) / H2 - 0.5)); j1 = int(np.ceil((y1 - lo[1]) / H2 - 0.5))
        if j1 <= j0: continue
        yy = ys[j0:j1]; xc = x0 + (yy - y0) * (x1 - x0) / (y1 - y0)
        ii = np.ceil((xc - lo[0]) / H2 - 0.5).astype(int)
        for j, i in zip(range(j0, j1), ii):
            if i < n[0]: cnt[max(i, 0):, j] += 1
    return cnt % 2 == 1

class Extrude(Prim):
    def __init__(self, mask, lo, z0, z1, M=None):
        super().__init__(M)
        self.lo = np.asarray(lo, float); self.z = (z0, z1)
        d = (ndi.distance_transform_edt(~mask) - ndi.distance_transform_edt(mask)) * H2
        self.d = d.astype(np.float32); self.n = mask.shape
    def local_box(self):
        return (np.r_[self.lo, self.z[0]], np.r_[self.lo + H2 * np.array(self.n), self.z[1]])
    def sdf_local(self, L):
        u = (L[..., 0] - self.lo[0]) / H2 - 0.5; v = (L[..., 1] - self.lo[1]) / H2 - 0.5
        d2 = map_coordinates(self.d, [u.ravel(), v.ravel()], order=1, mode='constant', cval=5.0).reshape(u.shape)
        zc = (self.z[0] + self.z[1]) / 2; hz = (self.z[1] - self.z[0]) / 2
        dz = np.abs(L[..., 2] - zc) - hz
        return np.minimum(np.maximum(d2, dz), 0) + np.hypot(np.maximum(d2, 0), np.maximum(dz, 0))

def cartridge_column(Tc, z_lug=34.0, z_body=22.0, z0=13.0, z1=31.2):
    """cartridge frame: lug/arm outline minus the housing outline, extruded z0..z1 (overlaps ring and lugs)"""
    lo = np.array([-26.0, -26.0]); n = (np.array([52.0, 52.0]) / H2).astype(int)
    lug = ndi.binary_fill_holes(mask2d(Tc, z_lug, lo, n))
    body = ndi.binary_fill_holes(mask2d(Tc, z_body, lo, n))
    m = ndi.binary_opening(lug & ~body, iterations=int(0.6 / H2))     # lug arms and lobes only, no slivers
    m = ndi.binary_dilation(m, iterations=int(0.4 / H2)) & lug          # overlap 0.4 mm into the housing wall
    return Extrude(m, lo, z0, z1), m, lo

def roll_channel(Tr, y0, z_pocket=-67.5, z0=-104.0, z1=-71.8, r_bore=23.0):
    """roll frame: the link's own empty lug pocket at z_pocket (inside the bore), extruded down z0..z1"""
    lo = np.array([-26.0, y0 - 26.0]); n = (np.array([52.0, 52.0]) / H2).astype(int)
    solid = mask2d(Tr, z_pocket, lo, n)
    xs = lo[0] + H2 * (np.arange(n[0]) + 0.5); ys = lo[1] + H2 * (np.arange(n[1]) + 0.5)
    X, Y = np.meshgrid(xs, ys, indexing='ij')
    m = ~solid & (X ** 2 + (Y - y0) ** 2 < r_bore ** 2)
    lab, k = ndi.label(m); c = lab[np.argmin(np.abs(xs)), np.argmin(np.abs(ys - y0))]
    m = lab == c                                     # the connected pocket around the yaw axis
    return Extrude(m, lo, z0, z1), m, lo
