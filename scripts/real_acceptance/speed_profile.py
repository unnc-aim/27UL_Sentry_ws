"""Command shaping for staged acceptance; not a certified braking model."""
import math


def limited_velocity(vx,vy,limit,previous_speed,dt,remaining):
    values=(vx,vy,limit,previous_speed,dt,remaining)
    if not all(math.isfinite(v) for v in values) or not 0<limit<=1. or min(previous_speed,dt,remaining)<0:
        raise ValueError('Invalid speed profile input')
    norm=math.hypot(vx,vy)
    if norm==0:return 0.,0.
    # Decreases/zero are immediate; never ramp emergency stopping commands.
    # Smaller approach commands stalled the real steering chassis about 13 cm
    # from the goal. Use its previously verified 0.15 m/s travel command floor;
    # limit/ramp/upstream reductions still take precedence, including zero.
    cap=min(limit,previous_speed+.30*min(dt,.10),max(.15,.60*max(0.,remaining-.02)))
    scale=min(1.,cap/norm)
    return vx*scale,vy*scale
