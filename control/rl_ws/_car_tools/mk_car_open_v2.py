import xml.etree.ElementTree as ET

CLOSED='/home/cyd/.local/share/Trash/files/car/urdf/car_closed.xml'
ALL1='/home/cyd/fpga/car/urdf/all1_isaacgym.urdf'
OPEN='/home/cyd/fpga/rl_ws2/Wheel-Legged-Gym-SF_small/resources/robots/car/urdf/car_open.urdf'
OUT='/home/cyd/fpga/rl_ws2/Wheel-Legged-Gym-SF_small/resources/robots/car/urdf/car_open_v2.urdf'
def v(s): return [float(x) for x in s.split()]

# ---- car_closed.xml：绝对位置 + 绝对质心 ----
C={}
def cwalk(b, base):
    nm=b.get('name'); pos=[a+c for a,c in zip(v(b.get('pos','0 0 0')), base)]
    ine=b.find('inertial')
    if ine is not None and ine.get('fullinertia') is not None:
        fi=[float(x) for x in ine.get('fullinertia').split()]   # ixx iyy izz ixy ixz iyz
        C[nm]=dict(m=float(ine.get('mass')), pos=pos,
                   com=[a+c for a,c in zip(v(ine.get('pos')), pos)],
                   I=dict(ixx=fi[0],iyy=fi[1],izz=fi[2],ixy=fi[3],ixz=fi[4],iyz=fi[5]))
    for ch in b.findall('body'): cwalk(ch, pos)
cr=ET.parse(CLOSED).getroot()
for b in cr.find('worldbody').findall('body'):
    if b.get('name')=='root': cwalk(b,[0,0,0])

# ---- all1 ----
A={}
for l in ET.parse(ALL1).getroot().findall('link'):
    ine=l.find('inertial')
    if ine is None: continue
    it=ine.find('inertia')
    A[l.get('name')]=dict(m=float(ine.find('mass').get('value')), com=v(ine.find('origin').get('xyz')),
        I=dict(ixx=float(it.get('ixx')),iyy=float(it.get('iyy')),izz=float(it.get('izz')),
               ixy=float(it.get('ixy')),ixz=float(it.get('ixz')),iyz=float(it.get('iyz'))))

def scale(b,k): return dict(m=b['m']*k, com=b['com'], I={a:b['I'][a]*k for a in b['I']})
def merge(b1,b2):
    m=b1['m']+b2['m']
    c=[(b1['m']*b1['com'][i]+b2['m']*b2['com'][i])/m for i in range(3)]
    acc={k:0.0 for k in ('ixx','iyy','izz','ixy','ixz','iyz')}
    for b in (b1,b2):
        d=[b['com'][i]-c[i] for i in range(3)]; d2=sum(x*x for x in d)
        for k in acc: acc[k]+=b['I'][k]
        acc['ixx']+=b['m']*(d2-d[0]**2); acc['iyy']+=b['m']*(d2-d[1]**2); acc['izz']+=b['m']*(d2-d[2]**2)
        acc['ixy']-=b['m']*d[0]*d[1];     acc['ixz']-=b['m']*d[0]*d[2];     acc['iyz']-=b['m']*d[1]*d[2]
    return dict(m=m,com=c,I=acc)

def build(kf_scale, kb_scale, front_mass, back_mass):
    """kf_scale/kb_scale: True=用 all1 缩放, False=用 car_closed 原值(自检)"""
    res={}
    for S in ('Right','Left'):
        fu,fl=C[f'{S}Front_link'],C[f'{S}FrontLower_link']
        bu,bl=C[f'{S}Back_link'], C[f'{S}BackLower_link']
        kf = (front_mass[S]/(fu['m']+fl['m'])) if kf_scale else 1.0
        kb = (back_mass[S]/(bu['m']+bl['m']))  if kb_scale else 1.0
        hip, knee = fu['pos'], fl['pos']   # cwalk 里存的都是绝对位置
        FU=dict(m=fu['m']*kf, com=[fu['com'][i]-hip[i]  for i in range(3)], I={k:fu['I'][k]*kf for k in fu['I']})
        FL=dict(m=fl['m']*kf, com=[fl['com'][i]-knee[i] for i in range(3)], I={k:fl['I'][k]*kf for k in fl['I']})
        BU=dict(m=bu['m']*kb, com=[bu['com'][i]-hip[i]  for i in range(3)], I={k:bu['I'][k]*kb for k in bu['I']})
        BL=dict(m=bl['m']*kb, com=[bl['com'][i]-knee[i] for i in range(3)], I={k:bl['I'][k]*kb for k in bl['I']})
        res[f'{S}Upper_link']=merge(FU,BU); res[f'{S}Lower_link']=merge(FL,BL)
    return res

# ===== 自检：k=1 应复现现有 car_open =====
old={l.get('name'):l.find('inertial') for l in ET.parse(OPEN).getroot().findall('link')}
chk=build(False,False,None,None)
print("自检（car_closed 原始质量 → 应等于现有 car_open）:")
ok=True
for nm in ('RightUpper_link','RightLower_link','LeftUpper_link','LeftLower_link'):
    o=old[nm]; om=float(o.find('mass').get('value')); oc=v(o.find('origin').get('xyz'))
    n=chk[nm]
    dm=abs(n['m']-om); dc=max(abs(n['com'][i]-oc[i]) for i in range(3))
    flag="✅" if (dm<1e-9 and dc<1e-9) else "❌"
    if flag=="❌": ok=False
    print(f"  {flag} {nm:18s} Δmass={dm:.2e} Δcom={dc:.2e}")
print("  自检通过" if ok else "  自检失败")

# ===== 正式生成 =====
front_mass={S:A[f'{S}Front_link']['m'] for S in ('Right','Left')}
back_mass ={S:A[f'{S}Back_link']['m']  for S in ('Right','Left')}
print(f"\n缩放系数:")
for S in ('Right','Left'):
    fu,fl=C[f'{S}Front_link'],C[f'{S}FrontLower_link']; bu,bl=C[f'{S}Back_link'],C[f'{S}BackLower_link']
    print(f"  {S:6s} 前链 ×{front_mass[S]/(fu['m']+fl['m']):.4f}   后链 ×{back_mass[S]/(bu['m']+bl['m']):.4f}")

res=build(True,True,front_mass,back_mass)
res['base']=A['base']
for S in ('Right','Left'): res[f'{S}Wheel_link']=A[f'{S}Wheel_link']

tot=sum(x['m'] for x in res.values())
print(f"\n新模型总质量 = {tot:.6f} kg    (all1 原始 = 1.571117, 旧 car_open = 0.675842)")
assert abs(tot-1.5711171)<1e-4

# 写 URDF
tree=ET.parse(OPEN); root=tree.getroot(); root.set('name','car_open_v2')
for c in list(root):
    if c.tag is ET.Comment: root.remove(c)
root.insert(0, ET.Comment('\n   car_open.urdf 的结构与运动学，质量/惯量取 all1_isaacgym.urdf 的完整数据。\n'
                          '   后腿按 car_closed.xml 比例缩放后合并进 Upper/Lower。自动生成，勿手改。\n  '))
for l in root.findall('link'):
    d=res[l.get('name')]; ine=l.find('inertial')
    ine.find('origin').set('xyz',' '.join(f'{x:.11g}' for x in d['com']))
    ine.find('mass').set('value',f"{d['m']:.11g}")
    for a,val in d['I'].items(): ine.find('inertia').set(a,f'{val:.11g}')
tree.write(OUT, encoding='utf-8', xml_declaration=True)

print(f"\n已写出: {OUT}")
print("\n逐 link（旧 → 新）:")
for lo,ln in zip(ET.parse(OPEN).getroot().findall('link'), ET.parse(OUT).getroot().findall('link')):
    mo=float(lo.find('inertial/mass').get('value')); mn=float(ln.find('inertial/mass').get('value'))
    print(f"  {ln.get('name'):18s} {mo:.6f} → {mn:.6f}  (×{mn/mo:.3f})")
def kin(r):
    return [(j.get('name'),j.get('type'),j.find('origin').get('xyz'),j.find('axis').get('xyz'),
             j.find('parent').get('link'),j.find('child').get('link'),
             j.find('limit').get('lower'),j.find('limit').get('upper')) for j in r.findall('joint')]
print("\n运动学完全一致:", kin(ET.parse(OPEN).getroot())==kin(ET.parse(OUT).getroot()))
