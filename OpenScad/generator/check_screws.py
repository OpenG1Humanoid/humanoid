"""Every radial screw (child link -> hub) must come out on the outside of the link so it can be
reached with a driver, and its counterbore must be open to the outside.

    python check_screws.py
"""
import json, os, sys
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
import design as D
from check_attach import link_tris
from check_cartridges import Inside
out = {}
for link in D.LINKS:
    rads = [o for o in D.OPS if o['link'] == link and o['kind'] == 'cut' and o['prim'][0] == 'rad' and o['prim'][3] > 0.5]
    if not rads: continue
    ins = Inside(link_tris(link))
    for o in rads:
        _, ang, z, r0, r1, d = o['prim']
        a = np.radians(ang); u = np.array([np.cos(a), np.sin(a), 0.0])
        rs = np.arange(r0, 90, 0.5)
        P = (np.outer(rs, u) + [0, 0, z]) @ o['M'][:3, :3].T + o['M'][:3, 3]
        m = ins(P)
        last = rs[np.where(m)[0].max()] if m.any() else None       # where the material ends along the screw line
        ok = last is not None and last < r1 - 1
        out.setdefault(link, []).append(dict(tag=o['tag'], angle=ang, material_ends_mm=None if last is None else float(last), bore_to_mm=r1, reaches_outside=bool(ok)))
        print(f'{link:20s} {o["tag"]:8s} screw at {ang:5.0f} deg: link material ends at r={last} mm, counterbore runs to r={r1} -> {"OK" if ok else "DOES NOT REACH THE OUTSIDE"}')
json.dump(out, open(os.path.join(HERE, 'screw_checks.json'), 'w'), indent=1)
