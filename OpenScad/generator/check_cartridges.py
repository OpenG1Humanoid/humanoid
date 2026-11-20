"""Checks for the servo cartridges against the real link meshes (run from OpenScad/generator).

    python check_cartridges.py

For every cartridge joint in design.CART:
  * wall left between the bore (and its lugs / inserts) and the outside of the host link
  * clashes between the bore and the other joints' cuts in the same link (e.g. the shoulder roll bore
    vs the shoulder-pitch hex socket)
  * that the servo body, horn and hub sit inside the cartridge with its walls >= 1.5 mm
Writes cartridge_checks.json.
"""
import json, os, sys
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
import design as D
ROOT = os.path.normpath(os.path.join(HERE, '..', '..'))
sys.path.insert(0, os.path.join(ROOT, 'OpenScad', 'torso_frame', 'generator'))
from prims import prim_mask
from scipy.spatial import cKDTree


def load(p):
    b = open(p, 'rb').read(); n = int.from_bytes(b[80:84], 'little')
    if len(b) == 84 + 50 * n:
        return np.frombuffer(b[84:], dtype=np.dtype([('n', '<3f4'), ('v', '<9f4'), ('a', '<u2')]), count=n)['v'].reshape(-1, 3, 3).astype(float)
    import re
    return np.array(re.findall(rb'vertex\s+(\S+)\s+(\S+)\s+(\S+)', b), float).reshape(-1, 3, 3)


def link_mesh(link):
    a = os.path.join(ROOT, 'STL', 'arms')
    if link == 'torso_link':
        return None
    f = 'left_shoulder_roll_link_fixed.stl' if link == 'shoulder_roll_link' else f'left_{link}.STL'
    o = os.path.join(a, 'original', f)
    return load(o if os.path.exists(o) else os.path.join(a, f)) * 1000


class Inside:
    """point-in-mesh by ray parity along a skew direction (closed meshes)"""
    def __init__(self, T):
        self.a = T[:, 0]; self.e1 = T[:, 1] - T[:, 0]; self.e2 = T[:, 2] - T[:, 0]
        d = np.array([0.2873, 0.4127, 0.8645]); self.d = d / np.linalg.norm(d)
        self.p = np.cross(self.d, self.e2); det = np.einsum('ij,ij->i', self.e1, self.p)
        self.ok = np.abs(det) > 1e-12; self.inv = 1 / np.where(self.ok, det, 1)
        lo = T.min(1); hi = T.max(1); self.lo, self.hi = lo, hi
    def __call__(self, P):
        out = np.zeros(len(P), bool)
        for i, o in enumerate(P):
            s = o - self.a; u = np.einsum('ij,ij->i', s, self.p) * self.inv; q = np.cross(s, self.e1)
            v = (q @ self.d) * self.inv; t = np.einsum('ij,ij->i', self.e2, q) * self.inv
            out[i] = (self.ok & (u >= 0) & (v >= 0) & (u + v <= 1) & (t > 0)).sum() % 2 == 1
        return out


def to_link(M, P): return P @ M[:3, :3].T + M[:3, 3]


def main():
    rep = {}
    for tag, g in D.CART.items():
        if tag == 'pitchR' or g['link'] == 'torso_link':
            continue
        M = g['M']; Rc = g['Rc']; Rb = Rc + 0.2; T = link_mesh(g['link']); ins = Inside(T)
        adds = [o for o in D.OPS if o['link'] == g['link'] and o['kind'] == 'add']
        def solid(P):
            m = ins(P)
            for o in adds: m |= prim_mask(P, o['M'], o['prim'])
            return m
        z_end = g['zr'] + 3.0 if g['mode'] == 'front' else g['host_len']
        # wall: march outwards from the bore surface until the host ends
        walls = []
        for z in np.linspace(g['zf'] + 1, z_end - 1, 7):
            for a in np.arange(0, 360, 10):
                ar = np.radians(a); dirn = np.array([np.cos(ar), np.sin(ar), 0])
                r0 = Rb if (g['mode'] == 'front' or z >= g['ct'] - 0.3) else g['r_nose'] + 0.2
                if z >= z_end - 3.0 and g['mode'] == 'rear': r0 = Rb + 1.5
                if g['flat']:
                    u = np.cos(ar - np.radians(g['flat'][1]))
                    if u > 1e-6: r0 = min(r0, (g['flat'][0] + 0.2) / u)
                rs = r0 + np.arange(0.25, 30, 0.25)
                P = to_link(M, np.stack([rs * dirn[0], rs * dirn[1], np.full_like(rs, z)], 1))
                m = solid(P); k = np.argmin(m) if not m.all() else len(m)
                walls.append((float(rs[k - 1] - r0) if k > 0 else 0.0, round(float(z), 1), int(a)))
        walls.sort()
        thin = [w for w in walls if w[0] < 1.5]
        # inserts / lugs inside material
        bad_ins = []
        for o in D.OPS:
            if o['tag'] == tag and o['kind'] == 'cut' and o['prim'][0] == 'cylat':
                _, x, y, r, z0, z1 = o['prim']
                ang = np.linspace(0, 2 * np.pi, 16, endpoint=False)
                ring = np.stack([x + (r + 1.0) * np.cos(ang), y + (r + 1.0) * np.sin(ang), np.full(16, (z0 + z1) / 2)], 1)
                f = solid(to_link(M, ring)).mean()
                if f < 1: bad_ins.append((round(float(np.degrees(np.arctan2(y, x)))), round(f, 2)))
        # clashes with other tags' cuts in the same link (sample the bore volume)
        rr = np.sqrt(np.random.default_rng(0).random(40000)) * Rb; th = np.random.default_rng(1).random(40000) * 2 * np.pi
        zz = g['zf'] + np.random.default_rng(2).random(40000) * (z_end - g['zf'])
        Q = np.stack([rr * np.cos(th), rr * np.sin(th), zz], 1)
        if g['flat']:
            u = Q[:, 0] * np.cos(np.radians(g['flat'][1])) + Q[:, 1] * np.sin(np.radians(g['flat'][1])); Q = Q[u <= g['flat'][0] + 0.2]
        P = to_link(M, Q)
        clash = {}
        for o in D.OPS:
            if o['link'] == g['link'] and o['kind'] == 'cut' and not o['tag'].startswith(tag):
                n = int(prim_mask(P, o['M'], o['prim']).sum())
                if n: clash[o['tag']] = clash.get(o['tag'], 0) + n
        rep[tag] = dict(link=g['link'], cartridge_radius=round(Rc, 2), bore_radius=round(Rb, 2), length=round(z_end - g['zf'], 1),
                        min_wall_mm=walls[0][0], thin_spots=len(thin), thin_examples=thin[:4], inserts_not_in_solid=bad_ins,
                        clashes_with=clash)
        print(tag, json.dumps(rep[tag]))
    # inside the cartridge: servo + hub + bearing vs the cartridge walls
    for tag, g in D.CART.items():
        Rc = g['Rc']; corner = g['need']; B = g['B']
        rep.setdefault(tag, {})['servo_wall_mm'] = round(Rc - corner, 2)
        rep[tag]['bearing_wall_mm'] = round(Rc - B['od'] / 2, 2)
        rep[tag]['spigot_wall_mm'] = round(B['id'] / 2 - 0.05 - (g['hr'] + 0.3), 2)
    json.dump(rep, open(os.path.join(HERE, 'cartridge_checks.json'), 'w'), indent=1)
    print({t: (r.get('servo_wall_mm'), r.get('bearing_wall_mm'), r.get('spigot_wall_mm')) for t, r in rep.items()})


if __name__ == '__main__':
    main()
