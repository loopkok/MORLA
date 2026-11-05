"""Six-joint numeric boundary for local URDF kinematics."""
import numpy as np

def vector6(values):
    result=np.asarray(values,dtype=float)
    if result.shape!=(6,) or not np.isfinite(result).all():
        raise ValueError('FR3 expected six finite values')
    return result
