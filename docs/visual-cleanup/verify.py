from pathlib import Path
import hashlib,json,numpy as np,mujoco as m
r=Path(__file__).parent;baseline=r/'baseline';candidate=r/'candidate';checks={};changed=[]
for p in baseline.rglob('*'):
 if not p.is_file() or '__pycache__' in str(p):continue
 q=candidate/p.relative_to(baseline)
 if p.read_bytes()!=q.read_bytes():changed.append(str(p.relative_to(baseline)))
checks['only_authorized_files_changed']=all((s.startswith('dyna_taku/assets/meshes/') and s.endswith('.obj')) or s in ['dyna_taku/taku.xml','dyna_taku/preview.png','dyna_taku/CHANGELOG.md'] for s in changed)
old_xml=(baseline/'dyna_taku/taku.xml').read_text();new_xml=(candidate/'dyna_taku/taku.xml').read_text()
checks['only_head_xml_change']=old_xml.replace('rgba="0.89 0.793583333333 0.6675 1"','rgba="0.93 0.88164 0.8184 1"')==new_xml
a=m.MjModel.from_xml_path(str(baseline/'dyna_taku/scene.xml'));b=m.MjModel.from_xml_path(str(candidate/'dyna_taku/scene.xml'))
for attr in ['body_mass','body_inertia','body_ipos','body_iquat','body_pos','body_quat','jnt_pos','jnt_axis','jnt_range','actuator_gainprm','actuator_biasprm','actuator_forcerange','actuator_ctrlrange','key_qpos','key_ctrl','mat_rgba','mat_specular','mat_shininess','geom_contype','geom_conaffinity','geom_friction','geom_solref','geom_solimp']:
 if attr=='mat_rgba':
  mask=np.arange(a.nmat)!=a.material('head_warm').id;checks['all_other_colors_unchanged']=bool(np.array_equal(a.mat_rgba[mask],b.mat_rgba[mask]));checks['head_color_expected']=bool(np.allclose(b.mat_rgba[b.material('head_warm').id],[0.93, 0.88164, 0.8184, 1]))
 else:checks[attr]=bool(np.array_equal(getattr(a,attr),getattr(b,attr)))
mask=(a.geom_contype!=0)|(a.geom_conaffinity!=0)
for attr in ['geom_pos','geom_quat','geom_size','geom_type']:checks['collision_'+attr]=bool(np.array_equal(getattr(a,attr)[mask],getattr(b,attr)[mask]))
def triangles(path):
 records={'v':[],'vn':[],'vt':[]};faces=[]
 for line in path.read_text().splitlines():
  parts=line.split()
  if not parts:continue
  if parts[0] in records:records[parts[0]].append(tuple(parts[1:]))
  elif parts[0]=='f':faces.append(parts[1:])
 result=[]
 for face in faces:
  result.append(tuple(tuple(records[k][int(i)-1] if i else () for k,i in zip(['v','vt','vn'],vert.split('/'))) for vert in face))
 return result
stats=[]
for name in changed:
 if not name.endswith(".obj"):continue
 old=triangles(baseline/name);new=triangles(candidate/name);checks['original_faces_'+Path(name).stem]=set(new).issubset(set(old));stats.append({'mesh':Path(name).stem,'before':len(old),'after':len(new),'removed':len(old)-len(new)})
da=m.MjData(a);db=m.MjData(b);m.mj_resetDataKeyframe(a,da,0);m.mj_resetDataKeyframe(b,db,0)
for i in range(1000):m.mj_step(a,da);m.mj_step(b,db)
checks['one_second_state_identical']=bool(np.array_equal(da.qpos,db.qpos) and np.array_equal(da.qvel,db.qvel));checks['finite_state']=bool(np.isfinite(db.qpos).all());checks['no_mujoco_warnings']=bool(not db.warning.number.any())
report={'checks':checks,'changed_meshes':stats,'removed_triangles':sum(x['removed'] for x in stats),'visual_bytes_before':sum((baseline/x).stat().st_size for x in changed),'visual_bytes_after':sum((candidate/x).stat().st_size for x in changed)};(r/'verification.json').write_text(json.dumps(report,indent=2));print(json.dumps({'passed':all(checks.values()),'failed':[k for k,v in checks.items() if not v],'changed_meshes':len(stats),'removed_triangles':report['removed_triangles']},indent=2));assert all(checks.values())
