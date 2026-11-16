"""Native MuJoCo contact-model starter. This is setup, not a validated benchmark."""
import argparse,json,math,platform,queue,time
from pathlib import Path
import mujoco
import numpy as np

HERE=Path(__file__).resolve().parent;OUT=HERE/'output';CONFIG=json.loads((HERE.parent/'config.json').read_text());K=np.array(CONFIG['lqr_gain'])

class Robot:
    def __init__(self):
        self.model=mujoco.MjModel.from_xml_path(str(HERE/'balancing_robot.xml'));self.data=mujoco.MjData(self.model)
        self.base=self.model.body('base').id;self.torso=self.model.body('torso').id
        self.reset()
    def reset(self):
        mujoco.mj_resetData(self.model,self.data);theta=math.radians(5);self.data.qpos[3:7]=[math.cos(theta/2),0,math.sin(theta/2),0];self.ticks=0;self.fallen=False;self.peak=5;self.pulse_end=0;self.force=0;self.max_contacts=0;mujoco.mj_forward(self.model,self.data)
    def state(self):
        d=self.data;R=d.xmat[self.base].reshape(3,3);pitch=math.atan2(R[0,2],R[2,2]);omega=(R@d.qvel[3:6])[1]
        return np.array([d.qpos[0],d.qvel[0],pitch,omega])
    def push(self,force):self.force=force;self.pulse_end=self.ticks+40
    def step(self):
        if self.fallen:return
        m,d=self.model,self.data;s=self.state();torque=float(np.clip(-K@s,-.7,.7));d.ctrl[:]=torque/2
        force=self.force if self.ticks<self.pulse_end else 0
        for _ in range(5):
            d.xfrc_applied[:]=0;d.xfrc_applied[self.base,0]=-.08*d.qvel[0];d.xfrc_applied[self.torso,0]=force;d.xfrc_applied[self.torso,4]=-.002*self.state()[3]
            mujoco.mj_step(m,d);self.max_contacts=max(self.max_contacts,d.ncon)
        # Refresh derived body poses at the endpoint before telemetry and feedback.
        mujoco.mj_forward(m,d);self.ticks+=1;s=self.state();R=d.xmat[self.base].reshape(3,3);roll=math.atan2(R[2,1],R[2,2]);self.peak=max(self.peak,abs(math.degrees(s[2])))
        self.fallen=abs(s[2])>=math.pi/4 or abs(roll)>=math.pi/4
        if not np.all(np.isfinite(d.qpos)) or not np.all(np.isfinite(d.qvel)):raise RuntimeError('Non-finite simulation state')
        if self.fallen:d.ctrl[:]=0;d.xfrc_applied[:]=0

def camera(robot):
    cam=mujoco.MjvCamera();cam.azimuth=135;cam.elevation=-18;cam.distance=1.05;cam.lookat[:]=[robot.data.qpos[0],0,.18];return cam

def render(robot,path):
    from PIL import Image
    with mujoco.Renderer(robot.model,height=900,width=1280) as renderer:
        renderer.update_scene(robot.data,camera(robot));Image.fromarray(renderer.render()).save(path)

def smoke():
    OUT.mkdir(exist_ok=True);r=Robot();assert r.model.nq==9 and r.model.nv==8 and r.model.nu==2
    assert abs(sum(r.model.body_mass)-1.55)<1e-12
    render(r,OUT/'preview.png')
    for i in range(2000):
        if i==400:r.push(3)
        r.step()
        if r.fallen:break
    render(r,OUT/'after_smoke.png')
    warnings={str(mujoco.mjtWarning(i)):int(w.number) for i,w in enumerate(r.data.warning) if w.number}
    assert not warnings,warnings
    result={'mujoco_version':mujoco.__version__,'python':platform.python_version(),'platform':platform.platform(),'model':'balancing_robot.xml','nq':r.model.nq,'nv':r.model.nv,'actuators':r.model.nu,'total_mass_kg':float(sum(r.model.body_mass)),'physics_step_s':r.model.opt.timestep,'control_step_s':.005,'simulated_seconds':r.data.time,'fallen':r.fallen,'peak_pitch_deg':r.peak,'final_state_x_v_pitch_rate':r.state().tolist(),'max_contact_count':r.max_contacts,'warnings':warnings,'rendering':'Two 1280x900 PNGs successfully rendered','scope':'Installation/contact-model smoke test only; not calibrated validation against the planar model.'}
    assert r.max_contacts>0
    (OUT/'setup_check.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))

def viewer():
    import mujoco.viewer
    OUT.mkdir(exist_ok=True);r=Robot();events=queue.SimpleQueue();paused=True;count=0
    print('SPACE: run/pause | J: -X push | L: +X push | R: reset | close window: exit',flush=True)
    with mujoco.viewer.launch_passive(r.model,r.data,key_callback=events.put,show_left_ui=False,show_right_ui=False) as view:
        with view.lock():view.cam.azimuth=135;view.cam.elevation=-18;view.cam.distance=1.05;view.cam.lookat[:]=[0,0,.18]
        view.sync();(OUT/'viewer_status.json').write_text(json.dumps({'running':view.is_running(),'paused':True,'version':mujoco.__version__,'started_unix':time.time()},indent=2)+'\n')
        while view.is_running():
            start=time.perf_counter()
            with view.lock():
                while not events.empty():
                    key=events.get()
                    if key==32 and not r.fallen:paused=not paused
                    elif key in (82,114):r.reset();paused=True
                    elif key in (74,106):r.push(-3)
                    elif key in (76,108):r.push(3)
                if not paused and not r.fallen:r.step()
                if r.fallen:paused=True
                view.cam.lookat[0]=r.data.qpos[0]
            count+=1
            if paused or count%3==0:
                status='FALL LIMIT - press R' if r.fallen else ('PAUSED' if paused else 'RUNNING')
                active_force=r.force if r.ticks<r.pulse_end else 0
                view.set_texts((None,None,
                    'Balancing robot | '+status+'\nSpace: run/pause    R: reset\nJ: push -X    L: push +X (3 N, 0.2 s)\nMouse: orbit / zoom\nNative contact starter - validation pending',
                    f'Time: {r.data.time:.2f} s\nPitch: {math.degrees(r.state()[2]):+.2f} deg\nPush: {active_force:+.1f} N\nPosition: {r.data.qpos[0]:+.3f} m'))
                view.sync()
            time.sleep(max(0,(.016 if paused else .005)-(time.perf_counter()-start)))
    (OUT/'viewer_status.json').write_text(json.dumps({'running':False,'ended_unix':time.time()},indent=2)+'\n')

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--viewer',action='store_true');parser.add_argument('--smoke',action='store_true');args=parser.parse_args()
    if args.viewer:viewer()
    else:smoke()
