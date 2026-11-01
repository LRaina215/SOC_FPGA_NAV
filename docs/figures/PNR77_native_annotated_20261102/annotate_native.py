"""Annotate native Gowin schematic PDFs without reconstructing their wiring.

All highlighted targets are matched to original yellow module rectangles using
the original instance labels. Only crop, uniform scale and annotation overlays
are applied to the existing vector PDF pages.
"""
from __future__ import annotations
import copy
import io
import json
import math
from pathlib import Path

import pdfplumber
import pypdfium2 as pdfium
from pypdf import PdfReader, PdfWriter, Transformation
from pypdf.generic import RectangleObject
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.pdfgen import canvas

OUT=Path(__file__).resolve().parent
SOURCE=OUT.parent/'gowin_PNR77_schematics_20261101'
W,H=900,600
pdfmetrics.registerFont(TTFont('CJK',r'C:\Windows\Fonts\simhei.ttf'))
INK='#213951'; MUTED='#5A6E80'; NAV='#225D8B'; TEAL='#19796E'; ORANGE='#AD641F'; RED='#A44343'


def color(c):
    if c=='white':return (1,1,1)
    return tuple(int(c[i:i+2],16)/255 for i in (1,3,5))


class Overlay:
    def __init__(self,c):self.c=c
    def text(self,x,y,s,size=12,fill=INK,center=False):
        self.c.setFont('CJK',size);self.c.setFillColorRGB(*color(fill))
        if center:self.c.drawCentredString(x,H-y-size*.82,s)
        else:self.c.drawString(x,H-y-size*.82,s)
    def rect(self,x,y,w,h,fill=None,stroke='#C9D4DD',r=6):
        self.c.saveState();self.c.setLineWidth(1)
        if fill:self.c.setFillColorRGB(*color(fill))
        self.c.setStrokeColorRGB(*color(stroke))
        self.c.roundRect(x,H-y-h,w,h,r,fill=bool(fill),stroke=True)
        self.c.restoreState()
    def arrow(self,pts,fill):
        self.c.saveState();self.c.setLineWidth(1.8);self.c.setStrokeColorRGB(*color(fill));self.c.setFillColorRGB(*color(fill))
        p=self.c.beginPath();p.moveTo(pts[0][0],H-pts[0][1])
        for x,y in pts[1:]:p.lineTo(x,H-y)
        self.c.drawPath(p)
        a,b=pts[-2],pts[-1];dx,dy=b[0]-a[0],b[1]-a[1];n=math.hypot(dx,dy);dx/=n;dy/=n
        tri=[b,(b[0]-6*dx+3*dy,b[1]-6*dy-3*dx),(b[0]-6*dx-3*dy,b[1]-6*dy+3*dx)]
        p=self.c.beginPath();p.moveTo(tri[0][0],H-tri[0][1])
        for x,y in tri[1:]:p.lineTo(x,H-y)
        p.close();self.c.drawPath(p,fill=1,stroke=0);self.c.restoreState()
    def badge(self,x,y,n,fill):
        self.c.setFillColorRGB(*color(fill));self.c.circle(x,H-y,8.5,fill=1,stroke=0)
        self.text(x,y-5.7,str(n),11.5,'white',True)
    def callout(self,x,y,w,h,n,title,lines,fill):
        self.rect(x,y,w,h,'white',fill)
        self.text(x+12,y+12,f'{n}  {title}',13.5,fill)
        yy=y+34
        for s in lines:self.text(x+12,yy,s,10.2,MUTED);yy+=14
    def header(self,n,title,sub):
        self.text(28,24,title,23)
        self.text(28,62,sub,10.8,MUTED)
        self.c.setStrokeColorRGB(*color('#C9D4DD'));self.c.line(28,H-85,872,H-85)
    def footer(self,n,source,note):
        self.c.setStrokeColorRGB(*color('#C9D4DD'));self.c.line(28,H-530,872,H-530)
        self.text(28,541,note,10.2)
        self.text(28,563,'原生来源：'+source,9.3,MUTED)
        self.text(800,580,f'PNR77 | {n}/3',9,MUTED)


def native_targets(filename,instances):
    p=SOURCE/filename
    with pdfplumber.open(p) as f:
        rects=[{k:r[k] for k in ('x0','y0','x1','y1')} for r in f.pages[0].rects if r.get('non_stroking_color')==(1,1,0)]
    texts={}
    def visit(t,cm,tm,font,size):
        s=t.strip()
        if s in instances:texts[s]=(cm[4],cm[5])
    PdfReader(p).pages[0].extract_text(visitor_text=visit)
    out={}
    for name in instances:
        assert name in texts,(filename,name)
        x,y=texts[name]
        r=min(rects,key=lambda r:abs(r['y0']-y)*10+abs((r['x0']+r['x1'])/2-x))
        assert abs(r['y0']-y)<1,(filename,name,r,texts[name])
        out[name]={'rect':r,'instance_label':list(texts[name]),'point':[(r['x0']+r['x1'])/2,(r['y0']+r['y1'])/2]}
    return out


def generate():
    OUT.mkdir(exist_ok=True,parents=True)
    specs=[
      {'file':'01_navigation_modules_native.pdf','crop':[0,550,595,842],'scale':.84,'x':185,'top':175,
       'instances':['u_pp','u_pt','u_pf','u_map','u_match','u_tr','u_sg']},
      {'file':'02_control_modules_native.pdf','crop':[0,0,595,842],'scale':.48,'x':300,'top':107,
       'instances':['sensor','wheel_left','wheel_right','model_thread','wheel_control','servo_thread','telemetry_thread']},
      {'file':'03_camera_lcd_modules_native.pdf','crop':[0,530,595,842],'scale':620/595,'x':245,'top':170,
       'instances':['capture','store','hud']},
    ]
    for s in specs:s['targets']=native_targets(s['file'],s['instances'])
    def point(s,name):
        x,y=s['targets'][name]['point'];return (s['x']+s['scale']*x,s['top']+s['scale']*(s['crop'][3]-y))
    buf=io.BytesIO();c=canvas.Canvas(buf,pagesize=(W,H),pageCompression=1)
    c.setTitle('PNR77 Gowin原生网表连线图 - 关键模块标注')
    d=Overlay(c);s=specs[0]
    d.header(1,'导航模块：Gowin 原生连线图','保留原始模块、引脚与连线；箭头只标出关键功能块，未重绘连接关系。')
    d.rect(183,173,504,250,r=4)
    calls=[
      ('u_pp',1,25,175,145,71,'深度投影',['u_pp','pc_project'],NAV),
      ('u_pt',2,25,280,145,71,'坐标变换',['u_pt','pc_transform'],NAV),
      ('u_pf',3,25,390,145,71,'点云过滤',['u_pf','pc_filter'],NAV),
      ('u_map',4,375,104,205,61,'高程 / 代价地图',['u_map / map_subsys'],TEAL),
      ('u_match',5,710,262,165,83,'SLF 结构匹配',['u_match / slf_match','结构点、结构图与似然场'],TEAL),
      ('u_tr',6,185,104,177,61,'轨迹推演评分',['u_tr / traj_eval'],ORANGE),
      ('u_sg',7,470,440,212,67,'安全限幅与看门狗',['u_sg / safety_guard'],RED),
    ]
    for name,n,x,y,w,h,title,lines,fill in calls:
        d.callout(x,y,w,h,n,title,lines,fill);t=point(s,name)
        if n in (1,2,3):start=(x+w,y+h/2);pts=[start,(178,start[1]),t]
        elif n==4:pts=[(x+w/2,y+h),(x+w/2,174),t]
        elif n==5:pts=[(x,y+h/2),t]
        elif n==6:pts=[(x+w/2,y+h),t]
        else:pts=[(x+w/2,y),t]
        d.arrow(pts,fill);d.badge(t[0]+12,t[1],n,fill)
    d.footer(1,s['file'],'视图：Post-Synthesis Netlist Viewer；本页是 u_nav 的模块连接过滤视图。')
    c.showPage()
    d=Overlay(c);s=specs[1]
    d.header(2,'原 4500 控制：Gowin 原生连线图','选取传感器、观测/策略模块和执行接口；普通控制网与自动编号不逐一标注。')
    d.rect(286,100,324,416,r=4)
    calls=[
      ('sensor',1,25,115,240,70,'BMI088 六轴采集',['sensor / bmi088'],NAV),
      ('wheel_left',2,25,218,240,82,'左右轮编码器与轮速',['wheel_left / wheel_right','motor520_speed'],NAV),
      ('model_thread',3,25,338,240,88,'观测与原 4500 策略',['model_thread / mem_policy','融合、历史观测、推理和动作映射'],TEAL),
      ('servo_thread',4,638,115,235,70,'四路腿部舵机接口',['servo_thread / servo_driver'],ORANGE),
      ('telemetry_thread',5,638,264,235,83,'控制遥测打包与发送',['telemetry_thread / uart_telemetry','P11-40 上行'],ORANGE),
      ('wheel_control',6,638,410,235,83,'双轮闭环与 PWM',['wheel_control','motor520_closed_loop'],ORANGE),
    ]
    for name,n,x,y,w,h,title,lines,fill in calls:
        d.callout(x,y,w,h,n,title,lines,fill);t=point(s,name)
        start=(x+w,y+h/2) if x<300 else (x,y+h/2)
        d.arrow([start,t],fill)
        bx,by={'sensor':(20,4),'wheel_left':(22,16),'servo_thread':(45,-9)}.get(name,(12,0))
        d.badge(t[0]+bx,t[1]+by,n,fill)
        if n==2:
            u=point(s,'wheel_right');d.arrow([(x+w,y+h/2+8),u],fill)
    d.footer(2,s['file'],'本页保留控制顶层原生连接；model_thread 内部子层级未在这份原图展开。')
    c.showPage()
    d=Overlay(c);s=specs[2]
    d.header(3,'摄像头 / LCD：Gowin 原生连线图','仅标注采集、帧缓存和 HUD 三个关键功能块；原始线网保持。')
    d.rect(238,166,634,335,r=4)
    d.callout(30,192,185,101,1,'双缓冲帧存储',['store / 片上 RAM','2 帧 × 19200 像素 × 16 bit','完整帧发布后按帧边界切换'],TEAL)
    d.callout(470,101,350,57,2,'OV5640 采集与缩采样',['capture：160×120 RGB565，全视场取样'],NAV)
    d.callout(30,362,185,95,3,'状态 HUD',['hud / g2_video_hud','IMU 与测距状态显示','与图像像素合成后送 LCD'],ORANGE)
    for name,n,pts,fill in [('store',1,[(215,242)],TEAL),('capture',2,[(635,158),(635,179)],NAV),('hud',3,[(215,407),(230,407)],ORANGE)]:
        t=point(s,name);pts.append(t);d.arrow(pts,fill);d.badge(t[0]+15,t[1],n,fill)
    d.footer(3,s['file'],'LCD 时序和像素合成位于父模块 g2_camera_lcd；本页保留子模块原始连接。')
    c.showPage();c.save();buf.seek(0)
    overlay=PdfReader(buf);w=PdfWriter()
    for i,s in enumerate(specs):
        original=copy.deepcopy(PdfReader(SOURCE/s['file']).pages[0])
        original.cropbox=RectangleObject(s['crop'])
        # Uniform scaling of original vector page, then annotation layer.
        p=w.add_blank_page(width=W,height=H)
        p.merge_transformed_page(original,Transformation((s['scale'],0,0,s['scale'],s['x'],H-s['top']-s['scale']*s['crop'][3])))
        p.merge_page(overlay.pages[i])
    w.add_metadata({'/Title':'PNR77 Gowin原生网表连线图 - 关键模块标注','/Subject':'Original Gowin vector netlists with arrows and Chinese labels. No schematic rewiring or block-diagram redraw.'})
    final=OUT/'PNR77_Gowin原生连线图_关键标注.pdf'
    with final.open('wb') as f:w.write(f)
    doc=pdfium.PdfDocument(str(final));names=['01_navigation_native','02_control_native','03_camera_native']
    for i,pg in enumerate(doc):pg.render(scale=3).to_pil().save(OUT/(names[i]+'.png'))
    (OUT/'provenance.json').write_text(json.dumps({'source_directory':str(SOURCE),'output':final.name,'method':'Original vector PDF + crop/scale + arrows and labels; no reconstruction of modules or wires','source_pdfs_modified':False,'schematic_exported_new':False,'ui_used_for_this_annotation':False,'eda_build_or_board_actions':False,'pages':specs},ensure_ascii=False,indent=2),encoding='utf-8')
    assert len(PdfReader(final).pages)==3
    print(str(final))


if __name__=='__main__':generate()
