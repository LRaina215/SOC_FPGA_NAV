"""Small single-factor stresses; ranges are assumptions, not measured bounds."""
from dataclasses import replace
from terrain_study import OUT,matrix,run_trial


def main():
    groups=[
        ('friction_low',dict(friction=.4),['up8','trench40d150']),
        ('friction_high',dict(friction=.8),['up8','trench40d150']),
        ('torque_half',dict(wheel_torque_scale=.5),['flat','up8','step20']),
        ('payload250g',dict(payload_kg=.25),['flat','up8']),
        ('policy_noise',dict(policy_noise=True),['flat','up8']),
        ('closed_cylinder',dict(mechanism='closed',wheel_shape='cad_cylinder'),['flat','up8','trench40d150']),
        ('oblique_ramp',dict(yaw_deg=30.),['up8']),
    ]
    for label,changes,cases in groups:
        for name in cases:
            out=OUT/label/name
            if (out/'result.json').exists():continue
            out.parent.mkdir(exist_ok=True)
            run_trial(replace(matrix()[name],**changes),out)


if __name__=='__main__':main()
