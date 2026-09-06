import json, math, subprocess
from pathlib import Path
import cv2, numpy as np, yaml
from scipy.spatial.transform import Rotation
from cleanup_perception.depth_geometry import DepthSample
from cleanup_perception.jaw_geometry import jaw_center_sample
root=Path('/home/user/turtlebot3_ws/cleanup_debug/hover_diagonal_forward30mm_20260906_171829')
r=json.loads((root/'result.json').read_text());a=np.load(Path(r['image']).with_suffix('.npz'))
k=a['camera_k'].reshape(3,3);ki=np.linalg.inv(k)
c=yaml.safe_load(Path('/home/user/turtlebot3_ws/src/cleanup_perception/config/grasp_camera_20260906.yaml').read_text())['scan_perception_node']['ros__parameters']
q=r['start_joints'];rw=Rotation.from_euler('z',q[0]).as_matrix()@Rotation.from_euler('y',sum(q[1:])).as_matrix();rotation=rw@np.array(c['grasp_camera_rotation']).reshape(3,3)
orig=np.array(r['jaw_geometry']['original_link1'])
# Recover effective translation from the exact recorded original point.
# Orientation uses nearby pre-motion joints: offline approximate analysis only.
from cleanup_perception.depth_geometry import central_grasp_sample
s=central_grasp_sample(a['mask'],a['depth'],.1,2.5,30)
translation=orig-rotation@(ki@np.array([s.u,s.v,1])*s.distance)
d=a['depth']*.001;m=cv2.erode(a['mask'].astype(np.uint8),np.ones((5,5),np.uint8))>0
yy,xx=np.nonzero(m&(d>.1)&(d<.8));points=((np.c_[xx,yy,np.ones(len(xx))]@ki.T)*d[yy,xx,None])@rotation.T+translation
center=np.median(points[:,:2],axis=0);_,ax=np.linalg.eigh(np.cov(points[:,:2],rowvar=False));axis=ax[:,-1];axis*=1 if axis[0]>0 else -1
long=(points[:,:2]-center)@axis;rows=[]
for percentile in range(20,81,5):
 t=np.percentile(long,percentile);band=np.abs(long-t)<.003
 if band.sum()<60:continue
 midpoint=np.median(points[band],axis=0);idx=np.argmin(np.linalg.norm(points-midpoint,axis=1))
 seed=DepthSample(float(xx[idx]),float(yy[idx]),float(d[yy[idx],xx[idx]]),0.,int(band.sum()))
 try:sample,info=jaw_center_sample(a['mask'],a['depth'],k,rotation,translation,seed)
 except ValueError:continue
 xyz=np.array(info['jaw_center_link1']);xyz[0]+=.03
 yaw=math.atan2(xyz[1],xyz[0]-.012);along=np.array([math.cos(yaw),math.sin(yaw)])
 local=points[np.linalg.norm(points[:,:2]-np.array(info['jaw_center_link1'])[:2],axis=1)<.025]
 _,axes=np.linalg.eigh(np.cov(local[:,:2],rowvar=False));tangent=axes[:,-1]
 angle=math.degrees(math.acos(np.clip(abs(tangent@along),0,1)))
 raw=subprocess.run(['/tmp/evaluate_grasp_candidate'],input=' '.join(map(str,[*xyz,-.101]))+'\n',text=True,capture_output=True,check=True).stdout.split()
 rows.append(dict(body_percentile=percentile,pixel=[sample.u,sample.v],target_link1=xyz.tolist(),visible_width_mm=1000*info['transverse_width_m'],local_axis_mismatch_deg=angle,feasible_pitch_count=int(raw[0]),first_pitch_deg=float(raw[1])))
result={'note':'Offline approximate orientation from pre-motion joints; floor=-0.101m. Feasibility checks IK and floor only; excludes object/arm collisions, true contact width, force closure, uncertainty. No hardware commands.','candidates':rows}
(root/'candidate_feasibility.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
