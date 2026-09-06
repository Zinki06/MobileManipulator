"""One authorized park/capture/grasp trial; stationary base, no transport or retries."""
from argparse import Namespace
from pathlib import Path
import json
import shutil
import subprocess
import time
import numpy as np
import rclpy
from rclpy.signals import SignalHandlerOptions
from control_msgs.action import FollowJointTrajectory
from std_msgs.msg import String
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectoryPoint
from cleanup_perception.candidate_hover_trial import HoverTrial

rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
t = HoverTrial(Namespace(output='/home/user/turtlebot3_ws/cleanup_debug', execute_hover=True,
    camera_calibration='/home/user/turtlebot3_ws/src/cleanup_perception/config/grasp_camera_20260906.yaml',
    forward_offset=.03, floor_height=0.))
t.report['trial_kind'] = 'single_authorized_candidate_grasp'
events=t.node.create_publisher(String, '/cleanup/events', 20)
base=None

def check(path):
    t._spin(.05)
    start=t._joints()
    if np.max(np.abs(t._base()-base))>.005: raise RuntimeError('Base moved')
    numbers=[t.report['geometry']['floor_link1'],len(path)]+start+[v for q in path for v in q]
    raw=subprocess.run(['/tmp/check_candidate_motion'],input=' '.join(map(str,numbers)),
        text=True,capture_output=True,check=True,timeout=2)
    result=json.loads(raw.stdout)
    if not result['safe']: raise RuntimeError('Finger floor/path clearance rejected: '+raw.stdout)
    return result

def move(path,seconds,label):
    t.report['phase']=label
    t.report.setdefault('motion_checks',[]).append(dict(phase=label,**check(path)))
    goal=FollowJointTrajectory.Goal()
    goal.trajectory.joint_names=['joint1','joint2','joint3','joint4']
    for i,q in enumerate(path):
        p=JointTrajectoryPoint(positions=q)
        nanos=round((i+1)*seconds/len(path)*1e9)
        p.time_from_start.sec=nanos//1000000000;p.time_from_start.nanosec=nanos%1000000000
        goal.trajectory.points.append(p)
    t.complete=False
    t.handle=t._wait(t.arm.send_goal_async(goal),3)
    if not t.handle.accepted:raise RuntimeError(label+' rejected')
    future=t.handle.get_result_async();deadline=time.monotonic()+seconds+4
    while not future.done():
        if time.monotonic()>deadline:raise RuntimeError(label+' timed out')
        t._spin(.10)
        check([])
    result=future.result()
    if result.status!=4 or result.result.error_code!=0:raise RuntimeError(label+' controller failed')
    t.complete=True
    t._spin(.25)
    t.report[label+'_actual_joints']=t._joints()
    t.report[label+'_clearance']=check([])
    print(label+'_COMPLETE',flush=True)

def holding():
    deadline=time.monotonic()+2.0;records=[]
    while time.monotonic()<deadline:
        t._spin(.05)
        m=t.sensors['joints'];t._fresh(m)
        stamp=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
        q=dict(zip(m.name,m.position))['gripper_left_joint']
        if not -.0085<q<.0175:records=[]
        elif not records or stamp!=records[-1][0]:
            if records and abs(q-records[-1][1])>.0005:records=[]
            records.append([stamp,q])
        if len(records)>=4 and records[-1][0]-records[0][0]>=.4:
            return True,records
    return False,records

try:
    t._spin(2.)
    base=t._base()
    events.publish(String(data='TARGET_VIEW_START|authorized single candidate grasp'))
    t._call(Trigger,'/park_arm',Trigger.Request(),6)
    t.report['park_complete']=True
    t._spin(.8)
    print('PARK_COMPLETE',flush=True)
    t.run()
    # The existing hover is completed before this script permits descent.
    selected=t.report['selected'];plan=selected['plan']
    move(plan['descent'],4.0,'DESCENT')
    t.report['phase']='CLOSE'
    t._call(Trigger,'/close_gripper',Trigger.Request(),5)
    t.options.execute_hover=False  # Joint feedback remains fresh; closed fingers are now expected.
    t.report['close_complete']=True
    held,records=holding();t.report['hold_after_close']=held;t.report['close_hold_samples']=records
    if not held:
        t.report['outcome']='GRASP_UNVERIFIED_NO_LIFT'
        print('GRASP_UNVERIFIED_NO_LIFT',flush=True)
    else:
        # Retrace only three validated descent segments, lifting nominally about 2cm.
        lift=list(reversed(plan['descent'][-4:-1]))
        move(lift,2.0,'SHORT_LIFT')
        held,records=holding();t.report['hold_after_lift']=held;t.report['lift_hold_samples']=records
        t.report['outcome']='ENCODER_HOLD_AFTER_SHORT_LIFT' if held else 'HOLD_LOST_AFTER_LIFT'
        print(t.report['outcome'],flush=True)
    t.report['final_joints']=t._joints()
    t.report['final_gripper']=dict(zip(t.sensors['joints'].name,t.sensors['joints'].position))['gripper_left_joint']
    t.report['final_base_delta']=(t._base()-base).tolist()
    # Save current view without disturbing the held pose.
    t._record_hover_evidence()
except BaseException as error:
    t.report['error']=str(error)
    print('TRIAL_STOPPED',repr(error),flush=True)
    raise
finally:
    shutil.copy2(__file__,t.root/'pick_trial_script.py')
    shutil.copy2('/tmp/check_candidate_motion.cpp',t.root/'check_candidate_motion.cpp')
    print('RESULT',t.root,flush=True)
    t.close()
    if rclpy.ok():rclpy.shutdown()
