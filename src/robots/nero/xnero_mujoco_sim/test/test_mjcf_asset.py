"""Central robot_description MJCF structure checks."""
import re
from pathlib import Path

import pytest

MJCF = Path(__file__).resolve().parents[5] / "src" / "robot_description" / "mjcf" / "xnero" / "xnero_dual.xml"


@pytest.fixture(scope="module")
def text():
    assert MJCF.is_file(), f"MJCF 不存在：{MJCF}（先跑 make_mjcf.py）"
    return MJCF.read_text(encoding="utf-8")


def test_generator_header(text):
    head = "\n".join(text.splitlines()[:8])
    assert "make_xnero_mjcf.py" in head, "missing centralized generator provenance"


def test_meshdir_relative(text):
    m = re.search(r'meshdir="([^"]+)"', text)
    assert m, "缺 meshdir"
    assert not m.group(1).startswith("/"), "meshdir 必须是相对路径（节点加载时才解析成绝对）"
    assert m.group(1) == "../../meshes"


def test_no_absolute_mesh_file(text):
    for f in re.findall(r'<mesh [^>]*file="([^"]+)"', text):
        assert not f.startswith("/"), f"mesh file 仍是绝对路径：{f}"


def test_hand_actuators_24(text):
    names = re.findall(r'<position name="(act_[^"]+)"', text)
    assert len(names) == 24, names[:3]
    assert "act_left_hand_index_joint1" in names
    assert "act_right_hand_pinky_joint2" in names


def test_actuator_ctrlrange_matches_urdf_limits(text):
    """ctrlrange 必须取自 URDF 限位，不能瞎写。"""
    m = re.search(r'<position name="act_left_hand_thumb_bend_joint"[^>]*ctrlrange="([^"]+)"', text)
    assert m and m.group(1).split() == ["0", "1.832"], m


def test_touch_sensors_10(text):
    names = re.findall(r'<touch name="(ft_[^"]+)"', text)
    assert len(names) == 10, names
    assert "ft_left_index" in names and "ft_right_thumb" in names


def test_pad_sites_10(text):
    assert len(re.findall(r'<site name="pad_', text)) == 10


def test_tactile_geoms_1200_with_contact_params(text):
    geoms = re.findall(r'<geom name="tac_[^>]*/>', text)
    assert len(geoms) == 1200
    assert all("solref=" in g for g in geoms), "触觉 geom 缺接触参数"


def test_floor_and_option(text):
    assert 'name="floor"' in text
    assert re.search(r'<option[^>]*timestep="0\.002"', text), "缺 option timestep"


def test_element_order_is_mjcf_canonical(text):
    """MJCF 要求 compiler → option → visual → asset → worldbody → actuator → sensor。"""
    order = ["<compiler", "<option", "<visual", "<asset", "<worldbody", "<actuator", "<sensor"]
    pos = [text.find(t) for t in order]
    assert all(p >= 0 for p in pos), dict(zip(order, pos))
    assert pos == sorted(pos), f"元素顺序不对：{dict(zip(order, pos))}"
