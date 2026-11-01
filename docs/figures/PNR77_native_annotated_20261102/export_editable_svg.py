"""Export the accepted native-netlist figures as editable, layered SVGs.

Gowin's native PDF paths are converted by pdftocairo. Annotation operations are
captured from annotate_native.py into SVG text/shapes rather than outlined PDF
text. No EDA run or GUI interaction is involved.
"""
from __future__ import annotations
import copy
import importlib.util
import json
import math
import re
import subprocess
import zipfile
from pathlib import Path
from xml.etree import ElementTree as ET

OUT=Path(__file__).resolve().parent
SVGOUT=OUT/'editable_svg'
CONVERTER=Path(r'D:\latex\texlive\2026\bin\windows\pdftocairo.exe')
TEMPROOT=Path(r'F:\nav_integration_g2_20261023\Gowin_document_figures_20261101\svg_native_temp')
SVG='http://www.w3.org/2000/svg'
INKSCAPE='http://www.inkscape.org/namespaces/inkscape'
MIN_NATIVE_STROKE_PT=0.18
ET.register_namespace('',SVG)
ET.register_namespace('xlink','http://www.w3.org/1999/xlink')
ET.register_namespace('inkscape',INKSCAPE)


def e(tag,attrs=None,text=None):
    x=ET.Element('{'+SVG+'}'+tag,{k:str(v) for k,v in (attrs or {}).items()})
    if text is not None:x.text=text
    return x


def hexcolor(rgb):
    return '#'+''.join(f'{round(x*255):02x}' for x in rgb)


class Captured(Exception):pass


class PathRecorder:
    def __init__(self):self.parts=[]
    def moveTo(self,x,y):self.parts.append(f'M {x:g},{600-y:g}')
    def lineTo(self,x,y):self.parts.append(f'L {x:g},{600-y:g}')
    def close(self):self.parts.append('Z')


class SvgCanvas:
    def __init__(self,*args,**kw):
        self.pages=[];self.root=e('g');self.stack=[self.root]
        self.style={'font_size':12,'fill':'#000000','stroke':'#000000','width':1}
        self.saved=[];self.serial=0
    def add(self,item):self.stack[-1].append(item)
    def begin_group(self,label):
        self.serial+=1
        g=e('g',{'id':f'annotation_{len(self.pages)+1}_{self.serial}','data-label':label})
        self.add(g);self.stack.append(g)
    def end_group(self):self.stack.pop()
    def setTitle(self,*a):pass
    def setFont(self,f,size):self.style['font_size']=size
    def setFillColorRGB(self,*rgb):self.style['fill']=hexcolor(rgb)
    def setStrokeColorRGB(self,*rgb):self.style['stroke']=hexcolor(rgb)
    def setLineWidth(self,w):self.style['width']=w
    def saveState(self):self.saved.append(dict(self.style))
    def restoreState(self):self.style=self.saved.pop()
    def drawString(self,x,y,s):self._text(x,y,s,'start')
    def drawCentredString(self,x,y,s):self._text(x,y,s,'middle')
    def _text(self,x,y,s,anchor):
        self.add(e('text',{'x':f'{x:g}','y':f'{600-y:g}',
            'font-family':'SimHei,Microsoft YaHei,sans-serif',
            'font-size':self.style['font_size'],'text-anchor':anchor,
            'fill':self.style['fill'],'xml:space':'preserve'},s))
    def _paint(self,fill=False,stroke=True):
        return {'fill':self.style['fill'] if fill else 'none',
                'stroke':self.style['stroke'] if stroke else 'none',
                'stroke-width':self.style['width']}
    def roundRect(self,x,y,w,h,r,fill=False,stroke=True):
        self.add(e('rect',{'x':f'{x:g}','y':f'{600-y-h:g}','width':f'{w:g}',
                          'height':f'{h:g}','rx':f'{r:g}',**self._paint(fill,stroke)}))
    def line(self,x1,y1,x2,y2):
        self.add(e('line',{'x1':x1,'y1':600-y1,'x2':x2,'y2':600-y2,**self._paint(False,True)}))
    def circle(self,x,y,r,fill=False,stroke=True):
        self.add(e('circle',{'cx':x,'cy':600-y,'r':r,**self._paint(fill,stroke)}))
    def beginPath(self):return PathRecorder()
    def drawPath(self,p,fill=False,stroke=True):
        self.add(e('path',{'d':' '.join(p.parts),**self._paint(fill,stroke)}))
    def showPage(self):
        assert len(self.stack)==1
        self.pages.append(self.root);self.root=e('g');self.stack=[self.root]
    def save(self):raise Captured()


def capture_annotations():
    spec=importlib.util.spec_from_file_location('accepted_native_annotations',OUT/'annotate_native.py')
    m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
    old_canvas=m.canvas.Canvas;old_overlay=m.Overlay
    recorder=SvgCanvas()
    class Grouped(old_overlay):
        def callout(self,*a,**kw):
            recorder.begin_group('callout '+str(a[4]));super().callout(*a,**kw);recorder.end_group()
        def arrow(self,*a,**kw):
            recorder.begin_group('editable arrow');super().arrow(*a,**kw);recorder.end_group()
        def badge(self,*a,**kw):
            recorder.begin_group('badge '+str(a[2]));super().badge(*a,**kw);recorder.end_group()
        def header(self,*a,**kw):
            recorder.begin_group('title and subtitle');super().header(*a,**kw);recorder.end_group()
        def footer(self,*a,**kw):
            recorder.begin_group('caption and provenance');super().footer(*a,**kw);recorder.end_group()
    try:
        m.canvas.Canvas=lambda *a,**kw:recorder
        m.Overlay=Grouped
        try:m.generate()
        except Captured:pass
    finally:m.canvas.Canvas=old_canvas;m.Overlay=old_overlay
    assert len(recorder.pages)==3
    return recorder.pages


def keep_native_strokes_visible(native,outer_scale):
    """Match PDF hairline visibility without changing any path coordinates.

    Cairo emits strokes as small as 0.008 pt in these native exports. PDF
    viewers give these hairlines a minimum visible width; SVG renderers do not.
    Retain the original paths, colors, glyphs and positions, and apply a small
    final-page minimum only to stroked native objects.
    """
    assert all(not g.get('transform') for g in native.iter('{'+SVG+'}g'))
    changed=0
    for p in native.iter('{'+SVG+'}path'):
        style=dict(item.split(':',1) for item in p.get('style','').split(';') if ':' in item)
        if style.get('stroke','none')=='none' or 'stroke-width' not in style:continue
        transform=p.get('transform','')
        if transform:
            m=re.fullmatch(r'matrix\(([^)]+)\)',transform)
            assert m,transform
            a,b,c,d,_,_=map(float,re.split(r'[,\s]+',m.group(1).strip()))
            assert abs(a*a+b*b-c*c-d*d)<1e-8 and abs(a*c+b*d)<1e-8
            scale=math.sqrt(abs(a*d-b*c))*outer_scale
        else:scale=outer_scale
        width=float(style['stroke-width'])
        if width*scale<MIN_NATIVE_STROKE_PT:
            p.set('data-original-stroke-width',style['stroke-width'])
            style['stroke-width']=f'{MIN_NATIVE_STROKE_PT/scale:.9g}'
            p.set('style',';'.join(k+':'+v for k,v in style.items())+';')
            changed+=1
    return changed


def main():
    SVGOUT.mkdir(exist_ok=True)
    TEMPROOT.mkdir(parents=True,exist_ok=True)
    provenance=json.loads((OUT/'provenance.json').read_text(encoding='utf-8'))
    pages=capture_annotations()
    names=['01_navigation_editable.svg','02_control_editable.svg','03_camera_lcd_editable.svg']
    titles=['导航模块 - 原生连线与可编辑标注','原4500控制 - 原生连线与可编辑标注','摄像头LCD - 原生连线与可编辑标注']
    report=[]
    for i,(s,ann,name) in enumerate(zip(provenance['pages'],pages,names)):
        original=Path(provenance['source_directory'])/s['file']
        native_svg=TEMPROOT/f'native_{i+1}.svg'
        subprocess.run([str(CONVERTER),'-svg',str(original),str(native_svg)],check=True,capture_output=True)
        native=ET.parse(native_svg).getroot()
        adjusted_strokes=keep_native_strokes_visible(native,s['scale'])
        root=e('svg',{'width':'900pt','height':'600pt','viewBox':'0 0 900 600','version':'1.1'})
        root.append(e('title',text=titles[i]))
        root.append(e('desc',text='原生Gowin网表矢量路径，叠加可编辑中文文字、箭头与编号。原始细小网名字形保留为矢量路径。'))
        root.append(e('metadata',text=json.dumps({'source':str(original),'build':'PNR77 terrain11p2r0','editable_annotation_text':True,'native_labels':'outlined glyphs','raster_image_used':False,'minimum_native_display_stroke_pt':MIN_NATIVE_STROKE_PT,'native_path_coordinates_unchanged':True},ensure_ascii=False)))
        defs=e('defs')
        for child in native:
            if child.tag=='{'+SVG+'}defs':
                for d in child:defs.append(copy.deepcopy(d))
        clip=e('clipPath',{'id':'native_pdf_crop','clipPathUnits':'userSpaceOnUse'})
        crop=s['crop'];native_top=842-crop[3]
        clip.append(e('rect',{'x':crop[0],'y':native_top,'width':crop[2]-crop[0],'height':crop[3]-crop[1]}))
        defs.append(clip);root.append(defs)
        bg=e('g',{'id':'background',f'{{{INKSCAPE}}}groupmode':'layer',f'{{{INKSCAPE}}}label':'背景'})
        bg.append(e('rect',{'width':900,'height':600,'fill':'white'}));root.append(bg)
        layer=e('g',{'id':'native_netlist',f'{{{INKSCAPE}}}groupmode':'layer',f'{{{INKSCAPE}}}label':'Gowin原生网表 - 矢量模块与连线'})
        tr=e('g',{'transform':f'translate({s["x"]},{s["top"]-s["scale"]*native_top}) scale({s["scale"]})'})
        clipped=e('g',{'clip-path':'url(#native_pdf_crop)'})
        for child in native:
            if child.tag not in ['{'+SVG+'}defs','{'+SVG+'}title','{'+SVG+'}metadata']:
                clipped.append(copy.deepcopy(child))
        tr.append(clipped);layer.append(tr);root.append(layer)
        ann.set('id','editable_annotations')
        ann.set(f'{{{INKSCAPE}}}groupmode','layer')
        ann.set(f'{{{INKSCAPE}}}label','可编辑标注 - 文字、箭头、编号')
        root.append(ann)
        path=SVGOUT/name
        ET.ElementTree(root).write(path,encoding='utf-8',xml_declaration=True)
        parsed=ET.parse(path).getroot()
        images=parsed.findall('.//{'+SVG+'}image')
        texts=ann.findall('.//{'+SVG+'}text')
        paths=layer.findall('.//{'+SVG+'}path')
        assert not images and len(texts)>10 and len(paths)>100
        report.append({'file':name,'editable_annotation_text_objects':len(texts),'native_path_elements':len(paths),'raster_images':len(images),'display_strokes_adjusted':adjusted_strokes})
    (SVGOUT/'svg_editability.json').write_text(json.dumps({'files':report,'layers':['background','native_netlist','editable_annotations'],'notes_zh':['中文标注保持为text，可直接改字。','箭头、边框、编号与原生线网均为矢量对象；可解除分组或编辑节点。','Gowin原图中的细小信号名保留为字形路径，不能作为普通文本直接改字。','未嵌入PNG或PDF图片；修改注释不需要重新打开Gowin。','保留原生路径与位置；仅将极细描边的最终显示宽度设为至少0.18pt，避免SVG显示时丢失细线。']},ensure_ascii=False,indent=2),encoding='utf-8')
    zip_path=OUT/'PNR77_可编辑SVG.zip'
    with zipfile.ZipFile(zip_path,'w',zipfile.ZIP_DEFLATED) as z:
        for p in [*(SVGOUT/n for n in names),SVGOUT/'svg_editability.json']:
            z.write(p,p.name)
    print(json.dumps({'files':report,'zip':str(zip_path)},ensure_ascii=False,indent=2))


if __name__=='__main__':main()
