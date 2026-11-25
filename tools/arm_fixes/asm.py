import re, numpy as np
import os
S=open(os.path.join(os.environ.get('ARMS_SRC', os.path.join(os.path.dirname(os.path.abspath(__file__)), 'original_arms')), 'arm_assembly.scad')).read()
def mods(side):
    out={}
    for m in re.finditer(r'module (%s_\w+_link)\(\)\{\n  multmatrix\((\[\[.*?\]\])\) rotate\(a=\w+, v=\[([^\]]*)\]\) \{(.*?)\n  \}\n\}'%side,S,re.S):
        name=m.group(1); M=np.array(eval(m.group(2))); v=np.array(eval('['+m.group(3)+']'),float)
        parts={}
        for mm in re.finditer(r'multmatrix\((\[\[.*?\]\])\) import\("([^"]+)"',m.group(4)):
            parts[mm.group(2).split('/')[-1][:-4]]=np.array(eval(mm.group(1)))
        child=re.findall(r'\n    (%s_\w+_link)\(\);'%side,m.group(4))
        out[name]=dict(M=M,v=v,parts=parts,child=child[0] if child else None)
    return out
