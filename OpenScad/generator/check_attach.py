"""Every piece ADDED to a link (spigot rings, sleeves) must be well joined to the link, or it prints as
a floating layer. For each added piece this reports how much of its root lies inside solid material.

    python check_attach.py
"""
import json, os, sys
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
import design as D
ROOT = os.path.normpath(os.path.join(HERE, '..', '..'))
sys.path.insert(0, os.path.join(ROOT, 'OpenScad', 'torso_frame', 'generator'))
from prims import prim_mask
from check_cartridges import Inside, load
SH = json.load(open(os.path.join(HERE, 'shells.json')))


def link_tris(link):
    a = os.path.join(ROOT, 'STL', 'arms')
    if link == 'shoulder_yaw_link':
        return np.concatenate([load(os.path.join(a, n)) for n in SH['left_shoulder_yaw_link']]) * 1000
    f = 'left_shoulder_roll_link_fixed.stl' if link == 'shoulder_roll_link' else f'left_{link}.STL'
    o = os.path.join(a, 'original', f)
    return load(o if os.path.exists(o) else os.path.join(a, f)) * 1000


def main():
    rep = {}
    rng = np.random.default_rng(0)
    for link in D.LINKS:
        ops = [o for o in D.OPS if o['link'] == link]
        adds = [o for o in ops if o['kind'] in ('ring', 'add')]
        if not adds: continue
        ins = Inside(link_tris(link)); cuts = [o for o in ops if o['kind'] in ('cut', 'plug')]
        bosses = [o for o in ops if o['kind'] == 'addhull']      # added inside the link's outline
        for o in adds:
            p = o['prim']
            if p[0] == 'ann':                                  # sample the root band of the ring: the lowest 3 mm
                r0, r1, z0, z1 = p[1], p[2], min(p[3], p[4]), max(p[3], p[4])
                zb = (z0, min(z0 + 3.0, z1))
                r = np.sqrt(rng.uniform(r0 ** 2, r1 ** 2, 3000)); t = rng.uniform(0, 2 * np.pi, 3000); z = rng.uniform(*zb, 3000)
                Q = np.stack([r * np.cos(t), r * np.sin(t), z], 1)
            else:
                continue                                       # sleeves wrap the link; checked by check_cartridges.py
            P = Q @ o['M'][:3, :3].T + o['M'][:3, 3]
            solid = ins(P)
            for b in bosses: solid |= prim_mask(P, b['M'], b['prim'])
            for c in cuts: solid &= ~prim_mask(P, c['M'], c['prim'])
            f = float(solid.mean())
            rep.setdefault(link, []).append(dict(tag=o['tag'], kind=o['kind'], root_band_mm=[round(zb[0], 1), round(zb[1], 1)], root_in_solid=round(f, 2)))
            print(f'{link:20s} {o["tag"]:8s} ring r {r0:5.1f}-{r1:5.1f}: {f * 100:5.1f} % of its root is inside the link')
    json.dump(rep, open(os.path.join(HERE, 'attach_checks.json'), 'w'), indent=1)


if __name__ == "__main__":
    main()
