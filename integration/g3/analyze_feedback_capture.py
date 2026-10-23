"""Summarize actual B1 captures; these statistics do not calibrate the vehicle."""
import argparse, collections, json, math, statistics
from pathlib import Path
from read_feedback_probe import Decoder


def stats(values):
    return dict(count=len(values), mean=statistics.fmean(values),
                population_std=statistics.pstdev(values), min=min(values), max=max(values)) if values else None


def analyze(folder):
    folder=Path(folder); decoder=Decoder(); rows=decoder.feed((folder/'rx.bin').read_bytes())
    if not rows: raise ValueError('no complete B1 frames')
    differences=lambda key,mask: collections.Counter((b[key]-a[key])&mask for a,b in zip(rows,rows[1:]))
    valid=[r for r in rows if r['bmi_initialized_sample']]
    # Count each publication once even if telemetry repeats its snapshot.
    unique=[]; last=None
    for row in valid:
        if row['bmi_sample_sequence']!=last: unique.append(row);last=row['bmi_sample_sequence']
    axis={k:stats([r[k] for r in unique]) for k in ('ax_raw','ay_raw','az_raw','gx_raw','gy_raw','gz_raw')}
    scaled={}
    for k in axis:
        divisor=10920. if k.startswith('a') else 16.384
        scaled[k.replace('_raw','_g' if k.startswith('a') else '_dps')]=stats([r[k]/divisor for r in unique])
    result=dict(capture=str(folder.resolve()),frames=len(rows),crc_errors=decoder.bad_crc,
        first_timestamp_ms=rows[0]['timestamp_ms'],last_timestamp_ms=rows[-1]['timestamp_ms'],
        frame_step_histogram=dict(differences('sequence',65535)),timestamp_step_ms_histogram=dict(differences('timestamp_ms',0xffffffff)),
        bmi_publication_step_histogram=dict(differences('bmi_sample_sequence',0xffffffff)),
        valid_bmi_frames=len(valid),fresh_bmi_frames=sum(r['bmi_fresh_50ms'] for r in rows),unique_bmi_samples=len(unique),
        bmi_age_ms=stats([r['bmi_age_ms'] for r in valid]),
        error_frame_count=sum(bool(r['status']&2) for r in rows),
        raw_axes=axis,nominal_units=scaled,acceleration_norm_g=stats([math.sqrt(sum(r[k]**2 for k in ('ax_raw','ay_raw','az_raw')))/10920. for r in unique]),
        saturated_axis_values=sum(r[k] in (-32768,32767) for r in unique for k in axis),
        encoder={side:dict(position_first=rows[0][side+'_position'],position_last=rows[-1][side+'_position'],
                    position_range=[min(r[side+'_position'] for r in rows),max(r[side+'_position'] for r in rows)],
                    nonzero_delta_frames=sum(r[side+'_delta']!=0 for r in rows),
                    invalid_first=rows[0][side+'_invalid'],invalid_last=rows[-1][side+'_invalid']) for side in ('left','right')},
        first_frame=rows[0],last_frame=rows[-1],
        conversion_basis='existing project config: 10920 raw/g and 16.384 raw/(degree/s); no new calibration',
        scope='observed raw sample continuity and fluctuations; no independent stationary truth, temperature/axis calibration, fusion or physical-stop acceptance')
    if len(unique)>=400:
        # A diagnostic of window variation, not replacement bias coefficients.
        result['gyro_first_last_200_raw']={k:dict(first=stats([r[k] for r in unique[:200]]),last=stats([r[k] for r in unique[-200:]])) for k in ('gx_raw','gy_raw','gz_raw')}
    return result


def main():
    p=argparse.ArgumentParser();p.add_argument('capture',type=Path);p.add_argument('--out',type=Path,required=True);a=p.parse_args()
    result=analyze(a.capture);a.out.write_text(json.dumps(result,indent=2),encoding='utf-8')
    print(json.dumps({k:v for k,v in result.items() if k not in ('first_frame','last_frame','raw_axes','nominal_units','gyro_first_last_200_raw')}))


if __name__=='__main__':main()
