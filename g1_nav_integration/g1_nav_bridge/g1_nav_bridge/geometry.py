"""SE(3) helpers, independent of ROS, quaternion order x,y,z,w."""
import math
import numpy as np


def transform(position, quaternion):
    p = np.asarray(position, dtype=float)
    q = np.asarray(quaternion, dtype=float)
    if p.shape != (3,) or q.shape != (4,) or not np.isfinite(p).all() or not np.isfinite(q).all():
        raise ValueError('Non-finite or malformed pose')
    norm = np.linalg.norm(q)
    if norm < 1e-8:
        raise ValueError('Zero quaternion')
    x, y, z, w = q / norm
    t = np.eye(4)
    t[:3, :3] = [[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                 [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                 [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]]
    t[:3, 3] = p
    return t


def quaternion(t):
    r = np.asarray(t)[:3, :3]
    # Symmetric eigensystem is stable at rotations near pi.
    k = np.array([[r[0,0]-r[1,1]-r[2,2], r[0,1]+r[1,0], r[0,2]+r[2,0], r[2,1]-r[1,2]],
                  [r[0,1]+r[1,0], r[1,1]-r[0,0]-r[2,2], r[1,2]+r[2,1], r[0,2]-r[2,0]],
                  [r[0,2]+r[2,0], r[1,2]+r[2,1], r[2,2]-r[0,0]-r[1,1], r[1,0]-r[0,1]],
                  [r[2,1]-r[1,2], r[0,2]-r[2,0], r[1,0]-r[0,1], np.trace(r)]]) / 3.0
    _, vectors = np.linalg.eigh(k)
    q = vectors[:, -1]
    return q if q[3] >= 0 else -q


def inverse(t):
    out = np.eye(4)
    out[:3, :3] = t[:3, :3].T
    out[:3, 3] = -out[:3, :3] @ t[:3, 3]
    return out


def planar(t):
    if np.linalg.norm(t[:2, 0]) < 1e-5:
        raise ValueError('Heading undefined near vertical body x-axis')
    yaw = math.atan2(t[1, 0], t[0, 0])
    out = transform([t[0, 3], t[1, 3], 0], [0, 0, math.sin(yaw/2), math.cos(yaw/2)])
    return out


def initial_tracking_pose(desired_base, current_tracking, tracking_to_base):
    current_base = current_tracking @ tracking_to_base
    delta = planar(desired_base) @ inverse(planar(current_base))
    return delta @ current_tracking


def body_twist(previous, current, dt):
    if not 0.001 <= dt <= 0.5:
        raise ValueError('Invalid velocity interval')
    linear = current[:3, :3].T @ ((current[:3, 3] - previous[:3, 3]) / dt)
    rel = inverse(current) @ previous
    q = quaternion(rel)
    sn = np.linalg.norm(q[:3])
    angular = -2.0 * q[:3] / dt if sn < 1e-9 else -q[:3] / sn * (2 * math.atan2(sn, q[3])) / dt
    return linear, angular


def rpy(t):
    r = t[:3, :3]
    pitch = math.atan2(-r[2, 0], math.hypot(r[0, 0], r[1, 0]))
    if abs(math.cos(pitch)) < 1e-7:
        return [math.atan2(-r[1, 2], r[1, 1]), pitch, 0.0]
    return [math.atan2(r[2, 1], r[2, 2]), pitch, math.atan2(r[1, 0], r[0, 0])]
