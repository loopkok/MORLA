import numpy as np
import pytest
from morla_learning.openpi_bridge import ComposedInputs,ComposedOutputs
from morla_learning.contracts import LearningContractError


def test_openpi_independent_dims_and_explicit_capacity_inverse():
    transform=ComposedInputs({'base_0_rgb':'scene_top'},11,8)
    data={'state':np.arange(11),'actions':np.arange(24).reshape(3,8),
          'images':{'scene_top':np.zeros((3,16,24),np.uint8)}}
    out=transform(data)
    assert out['state'].shape==(11,) and out['actions'].shape==(3,8)
    assert out['image']['base_0_rgb'].shape==(16,24,3)
    padded=np.zeros((3,32)); padded[:,:8]=out['actions']
    assert np.array_equal(ComposedOutputs(8,32)({'actions':padded})['actions'],data['actions'])
    with pytest.raises(LearningContractError): transform({**data,'actions':np.zeros((3,7))})
    with pytest.raises(LearningContractError): transform({**data,'images':{}})
    with pytest.raises(LearningContractError): ComposedOutputs(8,32)({'actions':np.zeros((3,54))})
