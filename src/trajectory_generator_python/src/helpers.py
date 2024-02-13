import math

def quat2yaw(q) -> float:
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                    1 - 2 * (q.y * q.y + q.z * q.z))
    return yaw

def saturate(val, low, high):
        return max(min(val, high), low)

def wrap(val):
    while val > math.pi:
        val -= 2.0 * math.pi
    while val < -math.pi:
        val += 2.0 * math.pi
    return val