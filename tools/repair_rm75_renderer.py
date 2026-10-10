#!/usr/bin/env python3
"""Hash-locked repair of the existing renderer; keep all legacy asset guards."""
import hashlib
from pathlib import Path
import re

root=Path(__file__).resolve().parents[1]
p=root/'web/robot-workbench/robot-scene.source.js'
raw=p.read_bytes()
actual=hashlib.sha1(b'blob '+str(len(raw)).encode()+b'\0'+raw).hexdigest()
if actual!='ded70ec91b943ee72932763b6ddaa23b2fbbee6a':
    raise SystemExit('Renderer baseline changed; inspect before editing')
s=raw.decode()
old=r"    if (!/^\/meshes\/(rm75\/(base_link|link_[1-7])|mevius2\/(base_link|[FB][LR]_(scapula|thigh|calf)_link))\.STL$/.test(g.url)) return;"
new=r"    if (!/^\/meshes\/(rm75\/(base_link|link_[1-7])|mevius2\/(base_link|[FB][LR]_(scapula|thigh|calf)_link))\.STL$/.test(g.url) && !/^\/meshes\/rm75-cad\/[0-9a-f]{64}\.stl$/.test(g.url)) return;"
if s.count(old)!=1:
    raise SystemExit('Expected exact existing asset allowlist')
s=s.replace(old,new)
old="      const t=pose.link_transforms[side+'_tip']; path.visible=this.showTrails && !!t;"
new="      const t=pose.link_transforms[side+'_tip'] || pose.link_transforms[side+'_tool0']; path.visible=this.showTrails && !!t;"
if s.count(old)!=1:
    raise SystemExit('Expected exact existing TCP trail lookup')
s=s.replace(old,new)
# Reject schemes, traversal, suffixes, queries and unapproved resource names.
pattern=re.compile(r'/meshes/rm75-cad/[0-9a-f]{64}\.stl\Z')
valid='/meshes/rm75-cad/'+'a'*64+'.stl'
assert pattern.fullmatch(valid)
for bad in ('https://example.com'+valid,'file:///etc/passwd',valid+'?x=1',
            valid+'/../file.stl','/meshes/rm75-cad/../file.stl',valid.upper(),valid[:-1]):
    assert not pattern.fullmatch(bad),bad
p.write_text(s)
print('Renderer repair applied; only verified same-origin SHA256 mesh paths added.')
