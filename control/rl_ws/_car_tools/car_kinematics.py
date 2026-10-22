"""CAD-derived planar five-bar / front-2R mapping (metres, radians).
This checks geometry only; it is not a claim of dynamic equivalence.
"""
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

SOURCE = Path('/home/cyd/fpga/car/urdf/car_closed.xml')


def vec(element, attr):
    return [float(x) for x in element.attrib[attr].split()]


class FiveBar:
    def __init__(self, path=SOURCE):
        root = ET.parse(path).getroot()
        self.sides = {}
        for side in ['Right', 'Left']:
            front = root.find('.//body[@name="'+side+'Front_link"]')
            back = root.find('.//body[@name="'+side+'Back_link"]')
            lower = front.find('body')
            rear_lower = back.find('body')
            a, b = vec(lower, 'pos'), vec(lower.find('body'), 'pos')
            c, d = vec(rear_lower, 'pos'), vec(rear_lower.find('site'), 'pos')
            self.sides[side] = dict(front=vec(front, 'pos'), back=vec(back, 'pos'),
                                   l1=math.hypot(a[1], a[2]), l2=math.hypot(b[1], b[2]),
                                   h0=math.atan2(a[2], a[1]),
                                   k0=math.atan2(b[2], b[1])-math.atan2(a[2], a[1]),
                                   rear_h0=math.atan2(c[2], c[1]),
                                   rear_k0=math.atan2(d[2], d[1])-math.atan2(c[2], c[1]),
                                   front_sign=vec(front.find('joint'), 'axis')[0],
                                   rear_sign=vec(back.find('joint'), 'axis')[0])

    @staticmethod
    def ik(y, z, l1, l2, branch):
        c = (y*y+z*z-l1*l1-l2*l2)/(2*l1*l2)
        if abs(c) > 1+1e-10:
            raise ValueError('wheel target outside linkage reach')
        knee = branch*math.acos(max(-1., min(1., c)))
        hip = math.atan2(z, y)-math.atan2(l2*math.sin(knee), l1+l2*math.cos(knee))
        return hip, knee

    def front_fk(self, side, qhip, qknee):
        s = self.sides[side]
        h, k = s['h0']+s['front_sign']*qhip, s['k0']+qknee
        return (s['front'][1]+s['l1']*math.cos(h)+s['l2']*math.cos(h+k),
                s['front'][2]+s['l1']*math.sin(h)+s['l2']*math.sin(h+k))

    def solve(self, side, y, z):
        s = self.sides[side]
        h, k = self.ik(y-s['front'][1], z-s['front'][2], s['l1'], s['l2'], -1)
        rh, rk = self.ik(y-s['back'][1], z-s['back'][2], s['l1'], s['l2'], 1)
        return dict(front=(h-s['h0'])/s['front_sign'], front_knee=k-s['k0'],
                    back=(rh-s['rear_h0'])/s['rear_sign'], back_knee=rk-s['rear_k0'])

    def rear_fk(self, side, qhip, qknee):
        s = self.sides[side]
        h, k = s['rear_h0']+s['rear_sign']*qhip, s['rear_k0']+qknee
        return (s['back'][1]+s['l1']*math.cos(h)+s['l2']*math.cos(h+k),
                s['back'][2]+s['l1']*math.sin(h)+s['l2']*math.sin(h+k))


def validate(output, minimum_mm=90):
    geometry = FiveBar()
    rows = []
    max_error = 0.0
    conditions = []
    for mm in range(minimum_mm, 191):
        h = mm/1000
        states = {}
        for side in geometry.sides:
            q = geometry.solve(side, -.03, .0325-h)
            fk = geometry.front_fk(side, q['front'], q['front_knee'])
            assert math.dist(fk, (-.03, .0325-h)) < 1e-10
            rear = geometry.rear_fk(side, q['back'], q['back_knee'])
            max_error = max(max_error, math.dist(fk, (-.03, .0325-h)),
                            math.dist(rear, (-.03, .0325-h)))
            assert max_error < 1e-10
            assert abs(q['front']) < 1.2 and abs(q['front_knee']) < 2.0
            assert abs(q['back']) < math.pi
            knee = geometry.sides[side]['k0']+q['front_knee']
            l1, l2 = geometry.sides[side]['l1'], geometry.sides[side]['l2']
            trace = l1*l1+2*l2*l2+2*l1*l2*math.cos(knee)
            det2 = (l1*l2*math.sin(knee))**2
            largest = (trace+math.sqrt(max(0., trace*trace-4*det2)))/2
            smallest = det2/largest
            conditions.append(math.sqrt(largest/smallest))
            states[side] = q
        rows.append(dict(height=h, joints=states))
    report = dict(height_travel_m=rows[-1]['height']-rows[0]['height'],
                  samples=len(rows)*2, fk_error_m_max=max_error,
                  jacobian_condition_min=min(conditions), jacobian_condition_max=max(conditions),
                  geometry=geometry.sides, endpoints=[rows[0], rows[-1]])
    Path(output).write_text(json.dumps(report, indent=2))
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    import sys
    validate(sys.argv[1] if len(sys.argv)>1 else 'kinematics_report.json',
             int(sys.argv[2]) if len(sys.argv)>2 else 90)
