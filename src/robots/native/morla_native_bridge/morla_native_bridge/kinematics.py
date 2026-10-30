"""Device-owned kinematics factories; the binding engine selects capability only."""
def nero(instance):
    from nero_arm_teleop.ik_solver import IKSolver
    return IKSolver()


def astral(instance):
    from astral_arm_teleop.ik.factory import make_single_arm_ik
    return make_single_arm_ik(instance.get('parameters',{}).get('side','left'),solver_type='geometric')
