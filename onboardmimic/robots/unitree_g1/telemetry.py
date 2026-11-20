import math

# Hardware LowState order (zero-based), not the interleaved policy order.
MOTOR_LABELS=(
    '左髋俯仰','左髋横滚','左髋偏航','左膝','左踝俯仰','左踝横滚',
    '右髋俯仰','右髋横滚','右髋偏航','右膝','右踝俯仰','右踝横滚',
    '腰偏航','腰横滚','腰俯仰',
    '左肩俯仰','左肩横滚','左肩偏航','左肘','左腕横滚','左腕俯仰','左腕偏航',
    '右肩俯仰','右肩横滚','右肩偏航','右肘','右腕横滚','右腕俯仰','右腕偏航')

def feedback(latest, target, now):
    try:
        body_time, body = latest['rt/lowstate']
        imu_time, imu = latest['rt/secondary_imu']
        age = max(now - body_time, now - imu_time)
        if not 0 <= age <= .5:
            return {'fresh': False, 'age_ms': round(age * 1000)}
        motors = body['motor_state'][:29]
        q = [m['q'] for m in motors]
        dq = [m['dq'] for m in motors]
        quat = imu['quaternion']
        valid = (len(q) == 29 and len(quat) == 4 and
                 all(type(v) in (int, float) and math.isfinite(v) for v in q + dq + quat)
                 and .9 < sum(v*v for v in quat) < 1.1)
        if not valid:
            return {'fresh': True, 'valid': False}
        return dict(fresh=True, valid=True, age_ms=round(age*1000), q=q, dq=dq,
                    faults=[i for i, m in enumerate(motors) if m.get('motorstate', 0)],
                    fault_details=[dict(index=i,name=MOTOR_LABELS[i],code=m['motorstate'],
                        code_hex=hex(m['motorstate']) if type(m['motorstate']) is int else None,
                        temperature=m.get('temperature'))
                        for i,m in enumerate(motors) if m.get('motorstate',0)],
                    max_speed_rad_s=max(map(abs, dq)),
                    max_error_rad=max(abs(a-b) for a, b in zip(q, target)),
                    upright=1-2*(quat[1]**2+quat[2]**2)/sum(v*v for v in quat) > math.cos(.35))
    except (KeyError, TypeError, ValueError, OverflowError):
        return {'fresh': False, 'valid': False}


def safe(t):
    return bool(t.get('fresh') and t.get('valid') and not t.get('faults')
                and t.get('max_speed_rad_s', 99) <= 12.5)
