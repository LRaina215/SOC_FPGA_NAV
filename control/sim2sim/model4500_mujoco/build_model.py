"""Create MJCF directly from the exact training URDF, without changing assets."""
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco

ROOT = Path(__file__).resolve().parent
PROJECT = ROOT.parents[1]
URDF = PROJECT / 'rl_ws/Wheel-Legged-Gym-SF_small/resources/robots/chy/urdf/chy.urdf'
CONFIG = PROJECT / 'model/wheel_velocity_original/parking_config.json'


def build():
    cfg = json.loads(CONFIG.read_text())['env']
    src = ET.parse(URDF).getroot()
    model = ET.Element('mujoco', model='model4500_sim2sim')
    ET.SubElement(model, 'compiler', angle='radian', autolimits='true',
                  inertiafromgeom='false', balanceinertia='false')
    ET.SubElement(model, 'option', timestep=str(cfg['sim']['dt']), gravity='0 0 -9.81',
                  integrator='implicitfast', iterations='50', cone='elliptic')
    visual = ET.SubElement(model, 'visual')
    ET.SubElement(visual, 'global', offwidth='1280', offheight='900')
    ET.SubElement(visual, 'headlight', ambient='.4 .4 .4', diffuse='.7 .7 .7')
    default = ET.SubElement(model, 'default')
    ET.SubElement(default, 'joint', damping='0', armature='0', frictionloss='0')
    ET.SubElement(default, 'geom', friction='.6 .005 .0001', solref='.006 1',
                  solimp='.9 .95 .001', condim='3')
    asset = ET.SubElement(model, 'asset')
    ET.SubElement(asset, 'texture', name='grid', type='2d', builtin='checker',
                  rgb1='.18 .22 .26', rgb2='.28 .33 .38', width='256', height='256')
    ET.SubElement(asset, 'material', name='floor_mat', texture='grid', texrepeat='10 10',
                  texuniform='true', reflectance='.1')
    world = ET.SubElement(model, 'worldbody')
    ET.SubElement(world, 'light', pos='0 -1 3', dir='0 .2 -1', directional='true')
    ET.SubElement(world, 'geom', name='floor', type='plane', size='20 20 .1',
                  material='floor_mat', contype='0', conaffinity='1')
    links = {x.attrib['name']: x for x in src.findall('link')}
    joints = list(src.findall('joint'))
    actuators = ET.SubElement(model, 'actuator')
    loaded_meshes = {}

    def add_link(name, parent, joint=None):
        body = ET.SubElement(parent, 'body', name=name)
        if joint is None:
            body.set('pos', ' '.join(map(str, cfg['init_state']['pos'])))
            ET.SubElement(body, 'freejoint', name='root')
        else:
            origin = joint.find('origin')
            body.set('pos', origin.get('xyz', '0 0 0'))
            body.set('euler', origin.get('rpy', '0 0 0'))
            jname = joint.get('name')
            limit = joint.find('limit')
            ET.SubElement(body, 'joint', name=jname, type='hinge',
                          axis=joint.find('axis').get('xyz'),
                          range=limit.get('lower')+' '+limit.get('upper'))
            effort = limit.get('effort')
            wheel = 'Wheel' in jname
            kp = 0. if wheel else 60.
            kd = .025 if wheel else .06
            # Same saturated PD force law, expressed natively so implicitfast
            # sees the velocity derivative instead of integrating D explicitly.
            ET.SubElement(actuators, 'general', name=jname+'_motor', joint=jname,
                          gear='1', gaintype='fixed', gainprm=str(kd if wheel else kp),
                          biastype='affine', biasprm='0 %g %g' % (-kp, -kd),
                          forcelimited='true', forcerange='-'+effort+' '+effort)
        link = links[name]
        inertial = link.find('inertial')
        origin = inertial.find('origin')
        assert origin.get('rpy', '0 0 0') == '0 0 0'
        inertia = inertial.find('inertia')
        ET.SubElement(body, 'inertial', mass=inertial.find('mass').get('value'),
                      pos=origin.get('xyz'), fullinertia=' '.join(
                          inertia.get(k) for k in ['ixx', 'iyy', 'izz', 'ixy', 'ixz', 'iyz']))
        for kind in ['visual', 'collision']:
            for number, shape in enumerate(link.findall(kind)):
                geometry = list(shape.find('geometry'))[0]
                origin = shape.find('origin')
                attrs = dict(name='%s_%s%d' % (name, kind, number),
                             pos=origin.get('xyz', '0 0 0'),
                             euler=origin.get('rpy', '0 0 0'))
                if kind == 'visual':
                    attrs.update(contype='0', conaffinity='0', group='2',
                                 rgba='.68 .72 .77 1')
                else:
                    # Training uses self_collisions=1 (disabled). Only floor contacts.
                    attrs.update(contype='1', conaffinity='0', group='3',
                                 rgba='.25 .35 .45 .3')
                if geometry.tag == 'mesh':
                    path = (URDF.parent / geometry.get('filename')).resolve()
                    if path not in loaded_meshes:
                        mesh_name = 'mesh%d' % len(loaded_meshes)
                        ET.SubElement(asset, 'mesh', name=mesh_name, file=str(path))
                        loaded_meshes[path] = mesh_name
                    attrs.update(type='mesh', mesh=loaded_meshes[path])
                elif geometry.tag == 'cylinder':
                    # IsaacGym's asset.replace_cylinder_with_capsule is true.
                    attrs.update(type='capsule' if cfg['asset']['replace_cylinder_with_capsule']
                                 else 'cylinder', size=geometry.get('radius')+' '+
                                 str(float(geometry.get('length'))/2))
                else:
                    raise ValueError('Unsupported URDF geometry '+geometry.tag)
                ET.SubElement(body, 'geom', **attrs)
        for child in joints:
            if child.find('parent').get('link') == name:
                add_link(child.find('child').get('link'), body, child)

    add_link('base', world)
    path = ROOT / 'robot.xml'
    ET.indent(model)
    ET.ElementTree(model).write(path, encoding='unicode')
    compiled = mujoco.MjModel.from_xml_path(str(path))
    expected_mass = sum(float(link.find('inertial/mass').get('value')) for link in links.values())
    assert abs(compiled.body_mass.sum()-expected_mass) < 1e-9
    report = dict(urdf=str(URDF), urdf_sha256=hashlib.sha256(URDF.read_bytes()).hexdigest(),
                  total_mass_kg=float(compiled.body_mass.sum()), nq=compiled.nq, nv=compiled.nv,
                  joints=[compiled.joint(i).name for i in range(compiled.njnt)],
                  caveats=['MuJoCo contact solver differs from PhysX; no claim of contact equivalence.',
                           'Ground sliding friction .6; restitution .15 has no direct identical mapping.',
                           'Cylinder-to-capsule conversion follows the training asset option.',
                           'Native affine actuators preserve saturated PD gains; damping integrated implicitly.'])
    (ROOT / 'model_manifest.json').write_text(json.dumps(report, indent=2))
    print(json.dumps(report, indent=2))
    return path


if __name__ == '__main__':
    build()
