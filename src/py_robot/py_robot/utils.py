def angle_pos_neg(angle) -> float:
    return (angle + 180.0) % 360.0 - 180.0

def angle_full_pos(angle) -> float:
    return angle % 360.0

def angle_dist(a, b) -> float:
    return abs(angle_pos_neg(a - b))

def clamp(value, low, high):
    return max(low, min(high, value))