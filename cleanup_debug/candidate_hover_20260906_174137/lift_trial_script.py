"""Continue the validated upward path only; never open, regrasp, or move the base."""
from pathlib import Path
source=Path('/tmp/candidate_pick_once.py').read_text().split('\ntry:\n',1)[0]
exec(compile(source,'/tmp/candidate_pick_once_helpers.py','exec'))
t.options.execute_hover=False
t.report['trial_kind']='user_confirmed_grasp_continue_lift'
t.report['source_trial']='candidate_hover_20260906_173957'
previous=json.loads(Path('/home/user/turtlebot3_ws/cleanup_debug/candidate_hover_20260906_173957/result.json').read_text())
t.report['geometry']=previous['geometry']
try:
    deadline=time.monotonic()+10
    while not all(k in t.sensors for k in ('odom','joints','rgb','depth','info')):
        if time.monotonic()>deadline:raise RuntimeError('Sensor discovery timeout')
        t._spin(.1)
    t._spin(.3)
    base=t._base()
    actual=t._joints()
    if np.max(np.abs(np.array(actual)-previous['final_joints']))>.04:
        raise RuntimeError('Arm no longer at recorded short-lift pose')
    held,samples=holding()
    t.report['hold_before_lift']=held
    if not held:raise RuntimeError('Holding feedback lost before lift')
    plan=previous['selected']['plan']
    path=list(reversed(plan['descent'][:6]))+[plan['joints']]
    move(path,3.5,'CONTINUE_LIFT')
    held,samples=holding()
    t.report['hold_after_lift']=held;t.report['hold_samples']=samples
    t.report['final_joints']=t._joints()
    t.report['final_gripper']=dict(zip(t.sensors['joints'].name,t.sensors['joints'].position))['gripper_left_joint']
    t.report['base_delta']=(t._base()-base).tolist()
    t._record_hover_evidence()
    print('LIFT_DONE_HOLDING',held,flush=True)
except BaseException as error:
    t.report['error']=str(error)
    raise
finally:
    shutil.copy2(__file__,t.root/'lift_trial_script.py')
    shutil.copy2('/tmp/candidate_pick_once.py',t.root/'pick_helpers_source.py')
    print('RESULT',t.root,flush=True)
    t.close()
    if rclpy.ok():rclpy.shutdown()
