"""Render recorded MuJoCo states. This replays one completed run; it never steps physics."""
import argparse
import csv
import json
from pathlib import Path
import subprocess

import mujoco as mj
import numpy as np
from PIL import Image,ImageDraw,ImageFont

HERE=Path(__file__).resolve().parent;NAV=HERE.parents[2]

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--run',type=Path,required=True);p.add_argument('--baseline',type=Path,required=True)
    p.add_argument('--out',type=Path,required=True);a=p.parse_args();a.out.mkdir(parents=True,exist_ok=False)
    result=json.loads((a.run/'result.json').read_text())
    states=np.load(a.run/'states.npz');times=states['t']
    with (a.run/'motion.csv').open() as f:rows=list(csv.DictReader(f))
    rt=np.array([float(r['t']) for r in rows])
    model=mj.MjModel.from_xml_path(str(a.run/'assets/robot.xml'));data=mj.MjData(model)
    renderer=mj.Renderer(model,height=500,width=1000);camera=mj.MjvCamera()
    camera.azimuth=0;camera.elevation=-28;camera.distance=2.3
    opt=mj.MjvOption();opt.geomgroup[3]=0;opt.geomgroup[5]=1
    font=ImageFont.truetype('C:/Windows/Fonts/consola.ttf',21)
    small=ImageFont.truetype('C:/Windows/Fonts/consola.ttf',18)
    ffmpeg=NAV/'sim/build/review_videos_20261021/tools/ffmpeg-win-x86_64-v7.1.exe'
    command=[str(ffmpeg),'-y','-f','rawvideo','-vcodec','rawvideo','-pix_fmt','rgb24',
             '-s','1000x720','-r','25','-i','-','-an','-c:v','libx264','-preset','veryfast',
             '-crf','23','-pix_fmt','yuv420p',str(a.out/'ultrasonic_mujoco.mp4')]
    with (a.out/'render.log').open('w') as log:
        proc=subprocess.Popen(command,stdin=subprocess.PIPE,stdout=log,stderr=log,
                              creationflags=subprocess.CREATE_NO_WINDOW)
        for t,q in zip(times,states['qpos']):
            data.qpos[:]=q;mj.mj_forward(model,data)
            camera.lookat[:]=[0,(data.xpos[model.body('base').id,1]+2.4)/2,.20]
            renderer.update_scene(data,camera,scene_option=opt)
            image=Image.new('RGB',(1000,720),(17,24,35));image.paste(Image.fromarray(renderer.render()),(0,56))
            d=ImageDraw.Draw(image);r=rows[max(0,np.searchsorted(rt,t,side='right')-1)]
            d.text((14,9),'MuJoCo 3.2.7 | Original4500 | SG90 fixed forward',font=font,fill='white')
            d.text((14,570),f't={t:5.2f}s   Remote request: {float(r["remote_request_mps"]):+.3f} m/s (UNCHANGED)',font=font,fill='#ffdb7d')
            d.text((14,601),f'Applied target: {float(r["guarded_target_mps"]):+.3f} m/s   Actual: {float(r["actual_forward_mps"]):+.3f} m/s',font=font,fill='#9ce1b1')
            d.text((14,632),f'Sonar: {float(r["sonar_m"]):.3f}m ({r["sonar_hit"]})   Body gap: {float(r["body_clearance_m"]):.3f}m',font=font,fill='white')
            d.text((14,663),'Mode: '+('ON' if r['mode_enabled']=='1' else 'OFF')+'   Stop latch: '+r['stop_latched']+'   Encoder parking hold: ON',font=small,fill='#9fc9ee')
            d.text((14,692),'One idealized physical scene; sonar mounting/echoes are assumptions, not measured hardware.',font=small,fill='#abb3c2')
            proc.stdin.write(image.tobytes())
            if abs(t-15.)<.021:image.save(a.out/'mujoco_stop_frame.png')
        proc.stdin.close();code=proc.wait();renderer.close()
        if code:raise RuntimeError('Read render.log')
    import matplotlib
    matplotlib.use('Agg');import matplotlib.pyplot as plt
    fig,axes=plt.subplots(3,1,figsize=(10,8),sharex=True)
    for folder,label in [(a.baseline,'Zero target only'),(a.run,'Encoder parking hold')]:
        with (folder/'motion.csv').open() as f:r=list(csv.DictReader(f))
        tt=[float(v['t']) for v in r]
        axes[0].plot(tt,[float(v['actual_forward_mps']) for v in r],label=label)
        axes[1].plot(tt,[float(v['body_clearance_m']) for v in r],label=label)
        axes[2].plot(tt,[float(v['tilt_deg']) for v in r],label=label)
    axes[0].plot(rt,[float(r['remote_request_mps']) for r in rows],'k--',alpha=.5,label='Remote request +0.3m/s')
    axes[0].set_ylabel('Actual speed (m/s)');axes[1].set_ylabel('Body gap (m)');axes[2].set_ylabel('Tilt (deg)');axes[2].set_xlabel('Simulation time (s)')
    for ax in axes:ax.grid(alpha=.3)
    axes[0].legend();fig.suptitle('Same20s MuJoCo scene, original4500: no change to remote speed')
    fig.tight_layout();fig.savefig(a.out/'mujoco_stop_comparison.png',dpi=150);plt.close(fig)
    (a.out/'render.json').write_text(json.dumps(dict(run=str(a.run),baseline=str(a.baseline),
        replay_only=True,physics_steps_during_render=0,frames=len(times),fps=25,
        remote_request_mps=.3,video=str(a.out/'ultrasonic_mujoco.mp4')),indent=2)+'\n')
    print(json.dumps(dict(output=str(a.out),frames=len(times),replay_only=True)))

if __name__=='__main__':main()
