"""Split a mesh into separate closed shells, one STL each, for OpenSCAD to union.

Some of the original Unitree meshes are several solids in one file that touch or overlap a little
(the shoulder yaw link is the link plus a side cover plate). OpenSCAD's CGAL engine cannot read such
a file as one solid and silently drops it, but it unions separate closed solids without trouble.

    python split_shells.py            # (from OpenScad/generator) does the meshes in SPLIT below

Each shell is repaired (corners welded, duplicate triangles dropped, edge-kissing pieces separated)
and checked on OpenSCAD's import grid: closed, every edge used once in each direction.
Writes STL/arms/<side>_<link>_shell<k>.stl (metres, like the originals) and shells.json, which
emit.py reads to write the mesh() module.
"""
import json, os, struct, sys
import numpy as np
from scipy import sparse
from scipy.sparse.csgraph import connected_components
HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.normpath(os.path.join(HERE, '..', '..'))
sys.path.insert(0, ROOT)
import export_stls as X
SPLIT = {'shoulder_yaw_link': 'arms/{side}_shoulder_yaw_link.STL'}
GRID = 2.0 ** -20


def write(path, V, F):
    W = V[F].astype(np.float32)
    n = np.cross(W[:, 1] - W[:, 0], W[:, 2] - W[:, 0]); n /= np.maximum(np.linalg.norm(n, axis=1, keepdims=True), 1e-30)
    rec = np.zeros(len(W), dtype=np.dtype([('n', '<f4', 3), ('v', '<f4', (3, 3)), ('a', '<u2')])); rec['n'], rec['v'] = n, W
    with open(path, 'wb') as fh:
        fh.write(b'shell from split_shells.py'.ljust(80) + struct.pack('<I', len(W)) + rec.tobytes())


def repair(T):
    V = T.reshape(-1, 3).astype(np.float64)
    from scipy.spatial import cKDTree
    tol = 2e-6 * np.ptp(V, axis=0).max()
    parent = np.arange(len(V))
    def root(i):
        while parent[i] != i: parent[i] = parent[parent[i]]; i = parent[i]
        return i
    for i, j in cKDTree(V).query_pairs(tol, output_type='ndarray'):
        a, b = root(i), root(j)
        if a != b: parent[max(a, b)] = min(a, b)
    r = np.array([root(i) for i in range(len(V))]); F = r.reshape(-1, 3)
    F = F[(F[:, 0] != F[:, 1]) & (F[:, 1] != F[:, 2]) & (F[:, 0] != F[:, 2])]
    F = X._fill_holes(X._drop_duplicates(F))
    V, F, _ = X._split_nonmanifold(V, F, nudge=max(5 * tol, 2e-4 * np.ptp(V, axis=0).max()))
    return V, F


def shells(V, F):
    n = len(F)
    d = np.sort(np.concatenate([F[:, [0, 1]], F[:, [1, 2]], F[:, [2, 0]]]), 1)
    key = d[:, 0] * (len(V) + 1) + d[:, 1]; o = np.argsort(key); ks = key[o]; fi = np.tile(np.arange(n), 3)[o]
    s = np.where(ks[1:] == ks[:-1])[0]
    nc, lab = connected_components(sparse.coo_matrix((np.ones(len(s)), (fi[s], fi[s + 1])), shape=(n, n)), directed=False)
    return [F[lab == k] for k in np.argsort(-np.bincount(lab))]


def grid_check(V, F):
    W = V[F]; Q = np.round(W.reshape(-1, 3) / GRID).astype(np.int64)
    _, inv = np.unique(Q, axis=0, return_inverse=True); G = inv.ravel().reshape(-1, 3)
    G = G[(G[:, 0] != G[:, 1]) & (G[:, 1] != G[:, 2]) & (G[:, 0] != G[:, 2])]
    return X._open_edges(G) + X._nonmanifold_edges(G) + X._misoriented_edges(G)


def main():
    out = {}
    for link, pat in SPLIT.items():
        for side in ('right', 'left'):
            src = os.path.join(ROOT, 'STL', pat.format(side=side))
            orig = os.path.join(os.path.dirname(src), 'original', os.path.basename(src))
            T = X._read_stl(__import__('pathlib').Path(orig if os.path.exists(orig) else src))
            V, F = repair(T)
            parts = shells(V, F)
            other = 'right' if side == 'left' else 'left'
            if any(grid_check(V, S) for S in parts) and out.get(f'{other}_{link}'):
                # this side would not separate cleanly: use the other side's clean shells, mirrored in y
                # (Unitree's left and right arm meshes are mirror images)
                Vs, Fs = [], []
                for nm in out[f'{other}_{link}']:
                    t = X._read_stl(__import__('pathlib').Path(os.path.join(ROOT, 'STL', 'arms', nm)))
                    t = t.copy(); t[:, :, 1] *= -1; t = t[:, [0, 2, 1]]
                    q = np.round(t.reshape(-1, 3) / GRID * 1.0).astype(np.int64)
                    _, idx, inv = np.unique(q, axis=0, return_index=True, return_inverse=True)
                    Vs.append(t.reshape(-1, 3)[idx]); Fs.append(inv.ravel().reshape(-1, 3))
                print(f'  {side}_{link}: using the mirrored {other} shells')
                V = None; parts = list(zip(Vs, Fs))
            names = []
            for k, S in enumerate(parts):
                if V is None: VV, S = S
                else: VV = V
                vol = np.einsum('ij,ij->i', VV[S[:, 0]], np.cross(VV[S[:, 1]], VV[S[:, 2]])).sum() / 6
                bad = grid_check(VV, S)
                if vol < 0: S = S[:, [0, 2, 1]]; vol = -vol
                if vol * 1e6 < 0.01: continue
                nm = f'{side}_{link}_shell{k}.stl'
                write(os.path.join(ROOT, 'STL', 'arms', nm), VV, S); names.append(nm)
                print(f'  {nm:40s} {len(S):6d} triangles  {vol * 1e6:8.2f} cm3  defects on the OpenSCAD grid: {bad}')
            out[f'{side}_{link}'] = names
    json.dump(out, open(os.path.join(HERE, 'shells.json'), 'w'), indent=1)


def _unused():
    for _ in ():
            for k, S in enumerate(shells(V, F)):
                vol = np.einsum('ij,ij->i', V[S[:, 0]], np.cross(V[S[:, 1]], V[S[:, 2]])).sum() / 6
                bad = grid_check(V, S)
                if vol < 0: S = S[:, [0, 2, 1]]; vol = -vol           # inside-out shell: flip
                if vol * 1e6 < 0.01: continue                          # drop slivers
                nm = f'{side}_{link}_shell{k}.stl'
                write(os.path.join(ROOT, 'STL', 'arms', nm), V, S); names.append(nm)
                print(f'  {nm:40s} {len(S):6d} triangles  {vol * 1e6:8.2f} cm3  defects on the OpenSCAD grid: {bad}')
            out[f'{side}_{link}'] = names
    json.dump(out, open(os.path.join(HERE, 'shells.json'), 'w'), indent=1)


if __name__ == '__main__':
    main()
