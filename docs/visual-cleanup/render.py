import os,sys,json
os.environ['MUJOCO_GL']='egl'
from pathlib import Path
import mujoco as m,numpy as np
from PIL import Image,ImageDraw
r=Path(__file__).parent;version=sys.argv[1];repo=r/version;out=r/'renders'/version;out.mkdir(parents=True,exist_ok=True)
a=m.MjModel.from_xml_path(str(repo/'dyna_taku/scene.xml'));d=m.MjData(a);a.vis.global_.offwidth=1400;a.vis.global_.offheight=1400
a.vis.headlight.diffuse[:]=.38;a.vis.headlight.ambient[:]=.32;a.vis.headlight.specular[:]=.15;a.light_diffuse[:]=.52;a.light_castshadow[:]=False
for i in range(a.ntex):
 if a.tex_type[i]==2:
  start=a.tex_adr[i];count=a.tex_width[i]*a.tex_height[i]*a.tex_nchannel[i];a.tex_data[start:start+count].reshape(-1,a.tex_nchannel[i])[:,:3]=[195,203,210]
a.mat_rgba[a.material('floor').id,:3]=[.65,.68,.71];a.mat_reflectance[a.material('floor').id]=.03
opt=m.MjvOption();opt.geomgroup[3:]=0;opt.sitegroup[:]=0
views={'front':(155,-8,False),'back':(25,-8,False),'side':(85,-8,False),'bent_back':(25,-16,True)}
with m.Renderer(a,1400,1400) as renderer:
 for view,(az,el,bent) in views.items():
  m.mj_resetDataKeyframe(a,d,0)
  pose={'folding_low_joint':-.15,'folding_high_joint':.6,'waist_pitch_joint':-.45,'left_shoulder_pitch_joint':-.35,'right_shoulder_pitch_joint':-.35,'left_elbow_joint':-1.15,'right_elbow_joint':-1.15,'left_shoulder_roll_joint':.18,'right_shoulder_roll_joint':-.18}
  if bent:pose.update(folding_low_joint=-.5,folding_high_joint=1.2,waist_pitch_joint=-.7,right_shoulder_roll_joint=-.7,right_elbow_joint=-1.9,right_wrist_pitch_joint=.6)
  for name,value in pose.items():d.qpos[a.joint(name).qposadr[0]]=value
  m.mj_forward(a,d);cam=m.MjvCamera();cam.lookat[:]=[0,0,.83];cam.distance=2.8;cam.azimuth=az;cam.elevation=el
  for mode in ['solid','transparent']:
   opt.flags[m.mjtVisFlag.mjVIS_TRANSPARENT]=(mode=='transparent');renderer.update_scene(d,cam,scene_option=opt);Image.fromarray(renderer.render()).save(out/f'{view}_{mode}.png')
print(out)
