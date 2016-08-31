def clamp(value, lb, ub):
    # Takes a value, a lower bound (lb) and an upper bound (ub) and returns the
    # value if it is between the lb and ub, the lb if the value is below the
    # lb, and the ub if the value is above the upper bound.
    result = max(min(value, ub), lb)
    return result

