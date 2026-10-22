"""Render each complete recorded trial; never re-run policy for a video."""
import argparse
import json
from pathlib import Path
import subprocess
import numpy as np
import mujoco as mj
from PIL import Image,ImageDraw,ImageFont
from terrain_scene import TerrainSpec,TerrainScene,S

ROOT=Path(__file__).resolve().parents[2]
FFMPEG=ROOT/'sim/build/review_videos_20261021/tools/ffmpeg-win-x86_64-v7.1.exe'
FONT=ImageFont.truetype('C:/Windows/Fonts/consola.ttf',16)
SMALL=ImageFont.truetype('C:/Windows/Fonts/consola.ttf',13)


def panel(arr,kind):
    if kind=='height':
        a=np.clip((arr.astype(float)+160)/400,0,1)
        rgb=np.stack([70+160*a,70+120*a,180-100*a],axis=-1).astype(np.uint8)
    elif kind=='raw':
        rgb=np.where(arr[...,None],np.array([235,60,50]),np.array([55,145,85])).astype(np.uint8)
    elif kind=='age':
        a=np.clip(arr.astype(float)/60,0,1);rgb=np.stack([50+190*a,180-130*a,70+0*a],axis=-1).astype(np.uint8)
    else:
        rgb=np.full((*arr.shape,3),[55,145,85],dtype=np.uint8)
        rgb[arr==253]=[235,170,40];rgb[arr==254]=[235,60,50];rgb[arr==255]=[55,58,67]
    return rgb


def render(path,fps=5):
    path=Path(path);result=path/'result.json';nav=not result.exists()
    r=json.loads((path/'metrics.json' if nav else result).read_text(encoding='utf-8'))
    modeldir=path/'physics_backend' if nav else path
    model=mj.MjModel.from_xml_path(str(modeldir/'assets/robot.xml'));data=mj.MjData(model)
    mapfile=path/'maps_v2/map_frames.npz' if (path/'maps_v2/map_frames.npz').exists() else path/'map_frames.npz'
    state=np.load(modeldir/'full_state.npz');maps=np.load(mapfile) if mapfile.exists() else None
    spec=r.get('scene',{}).get('spec') if not nav else json.loads((path/'terrain_model.json').read_text())['spec']
    spec=TerrainSpec(**spec);sc=TerrainScene(spec)
    report=r.get('physics_backend',r);end=float(state['t'][-1]);fail=report.get('first_failure_s')
    image=Image.new('RGB',(1280,720),(18,22,30));cam=mj.MjvCamera()
    cam.azimuth=155;cam.elevation=-21;cam.distance=1.0
    # CAD +Y is forward; look at robot with real terrain in frame, no hidden geom.
    opt=mj.MjvOption();opt.geomgroup[3:]=0;opt.geomgroup[0:3]=1
    opt.flags[mj.mjtVisFlag.mjVIS_CONTACTPOINT]=True
    renderer=mj.Renderer(model,height=450,width=720)
    target=path/('video_map_v2.mp4' if 'maps_v2' in str(mapfile) else 'video.mp4')
    proc=subprocess.Popen([str(FFMPEG),'-loglevel','error','-y','-f','rawvideo','-vcodec','rawvideo','-pix_fmt','rgb24','-s','1280x720','-r',str(fps),'-i','-','-an','-vcodec','libx264','-preset','veryfast','-crf','24','-pix_fmt','yuv420p',str(target)],stdin=subprocess.PIPE)
    events=np.load(path/'nav_events.npz')['events'] if nav and (path/'nav_events.npz').exists() else None
    support=np.load(path/'support_frames.npz') if (path/'support_frames.npz').exists() else None
    try:
        for t in np.arange(0,end+1e-6,1/fps):
            i=min(len(state['t'])-1,int(np.searchsorted(state['t'],t)))
            data.qpos[:]=state['qpos'][i];data.qvel[:]=state['qvel'][i];mj.mj_forward(model,data)
            center=data.xpos[model.body('base').id];cam.lookat[:]=center+[0,.15,-.03]
            renderer.update_scene(data,cam,scene_option=opt);pic=Image.fromarray(renderer.render())
            image.paste((18,22,30),(0,0,1280,720));image.paste(pic,(0,70))
            d=ImageDraw.Draw(image);d.text((14,10),f'{path.parent.name}/{path.name}   t={t:05.1f}/{end:05.1f}s',font=FONT,fill='white')
            status='FAILED (physics continues)' if fail is not None and t>=fail else 'running'
            d.text((14,34),f'{spec.kind} value={spec.value:g} depth={spec.depth:g}  {spec.mechanism}/{spec.profile}/{spec.wheel_shape}',font=SMALL,fill='white')
            d.text((14,54),f'final: {r.get("analysis_outcome",r.get("outcome"))}  now: {status}',font=SMALL,fill=(255,110,95) if 'FAILED' in status else (130,225,150))
            c=S@(data.xpos[model.body('base').id]+data.xmat[model.body('base').id].reshape(3,3)@[-.08,-.006656542,0])
            R=S@data.xmat[model.body('base').id].reshape(3,3)@S.T
            ground=c-R@[0,0,.1305]
            d.text((16,530),f'wheel-reference x/y/z={ground[0]:+.3f}/{ground[1]:+.3f}/{ground[2]:+.3f}m',font=FONT,fill='white')
            names=[]
            for contact in data.contact:
                nn=[model.geom(g).name or '' for g in (contact.geom1,contact.geom2)]
                if any(n.startswith('terrain_') for n in nn):names.append('wheel' if any('Wheel' in n for n in nn) else 'NONWHEEL')
            d.text((16,554),f'contact candidates: wheels={names.count("wheel")} nonwheel={names.count("NONWHEEL")}',font=FONT,fill='white')
            d.text((16,580),'3D and map frames from this exact saved run; no resets.',font=SMALL,fill=(190,195,205))
            d.text((16,600),'green=free red=raw hazard yellow=inflated dark=unknown',font=SMALL,fill=(190,195,205))
            d.text((16,620),'map age: green=fresh orange/red=older; height colors mm',font=SMALL,fill=(190,195,205))
            if maps is not None and len(maps['t']) and t>=maps['t'][0]:
                k=max(0,min(len(maps['t'])-1,int(np.searchsorted(maps['t'],t,side='right')-1)))
                labels=[('H','height','height [-160,+240] mm'),('raw','raw','raw hazards (V only)'),('cost','cost','inflated cost + filled cells'),('age','age','age [0,60] frames')]
                for j,(key,kind,title) in enumerate(labels):
                    if j==3 and support is not None:
                        sk=max(0,min(len(support['t'])-1,int(np.searchsorted(support['t'],t,side='right')-1)))
                        bits=1<<np.arange(16,dtype=np.uint32)
                        known=(support['known'][sk][...,None]&bits)!=0;fresh=(support['fresh'][sk][...,None]&bits)!=0
                        C=np.where(known,np.where(fresh,0,253),255).astype(np.uint8).reshape(32,32,4,4).transpose(0,2,1,3).reshape(128,128)
                        rgb=panel(C,'cost');title='support4x zoom: fresh/remembered/?'
                    else:
                        rgb=panel(maps[key][k],kind)
                        if kind!='cost':rgb[~maps['V'][k]]=[55,58,67]
                    im=Image.fromarray(np.flipud(rgb));dd=ImageDraw.Draw(im)
                    # Map center marker; navigation maps can be in a rotated terrain frame.
                    dd.ellipse((62,62,66,66),fill='white')
                    im=im.resize((250,250),Image.Resampling.NEAREST);x=740+(j%2)*265;y=105+(j//2)*285
                    image.paste(im,(x,y));d.text((x,y-20),title,font=SMALL,fill='white')
                d.text((740,676),f'map t={maps["t"][k]:.3f}s frame age={t-maps["t"][k]:.3f}s',font=SMALL,fill='white')
            else:d.text((750,140),'No map captured yet',font=FONT,fill='white')
            # Independent terrain profile, never fed to navigation.
            xs=np.linspace(-.1,sc.goal_x+.15,250);zs=sc.height(xs,np.zeros_like(xs))
            pts=[(16+int(j*2.7),690-int(z*350)) for j,z in enumerate(zs)]
            d.line(pts,fill=(180,205,220),width=2)
            xx=16+int((ground[0]+.1)/(sc.goal_x+.25)*249*2.7)
            d.ellipse((xx-3,681,xx+3,687),fill=(255,100,90))
            proc.stdin.write(image.tobytes())
    finally:
        proc.stdin.close();code=proc.wait();renderer.close()
    if code:raise RuntimeError(f'ffmpeg failed {code}')
    print(target,flush=True)


def main():
    ap=argparse.ArgumentParser();ap.add_argument('paths',nargs='+',type=Path);ap.add_argument('--fps',type=int,default=5);a=ap.parse_args()
    for p in a.paths:
        trials=[p] if (p/'result.json').exists() or (p/'metrics.json').exists() else [x.parent for x in sorted(p.rglob('result.json'))]+[x.parent for x in sorted(p.rglob('metrics.json'))]
        for trial in trials:
            target=trial/('video_map_v2.mp4' if (trial/'maps_v2/map_frames.npz').exists() else 'video.mp4')
            if not target.exists():render(trial,a.fps)


if __name__=='__main__':main()
