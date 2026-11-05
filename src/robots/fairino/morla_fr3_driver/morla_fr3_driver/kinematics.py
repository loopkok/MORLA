"""FR3 URDF provider; all FK/IK uses the extracted fairino3_v6 model."""
from .math_utils import vector6
from .urdf_ik import IKSolver


class FR3Kinematics(IKSolver):
    """Compatibility constructor for read-only FK checks and generic bindings."""
    def __init__(self, parameters=None):
        p=parameters or {}
        super().__init__(joint_lower=p.get('joint_lower'),joint_upper=p.get('joint_upper'),
                         joint_signs=p.get('urdf_joint_signs'),joint_offsets=p.get('urdf_joint_offsets'))


def create_kinematics(instance):
    return FR3Kinematics(instance.get('parameters'))
