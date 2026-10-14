"""对抗性配置测试：换机器人配置（双臂 / 灵巧手 / 腰头 / 多相机 / 异 fps）后，
采集→对齐→校验→转换→openpi 训练管线全链路是否仍然正确。

每种配置都跑完整链条，并在 openpi 侧逐维验证：
  - delta/绝对划分与 schema 推导的期望掩码逐位一致（delta 是相对 chunk 起始 state）
  - AstralOutputs 截断维 == schema.state_dim
  - 相机槽位内容确实来自 camera_map 指定的相机（按 conftest 的相机颜色指纹验证）
  - 空 task episode 触发 default_prompt 兜底

负例：总维数 >32（pi05_base 上限）、camera_map 指向不存在的相机、缺 meta。

注意：HF_LEROBOT_HOME 必须在 import openpi（间接 import pinned lerobot）之前
设定——lerobot 0.1.0 在 import 时把它固化为模块常量。本文件用独立临时
HF_LEROBOT_HOME，完全不碰真实数据集。
"""

import json
import os
import sys
import tempfile

import numpy as np
import pytest

_ADV_HOME = tempfile.mkdtemp(prefix="adv_lerobot_home_")
os.environ["HF_LEROBOT_HOME"] = _ADV_HOME
# pinned lerobot 0.1.0 见到旧变量名会直接 raise，先清掉
os.environ.pop("LEROBOT_HOME", None)

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from astral_data_collect.pipeline.align_data import align_episode  # noqa: E402
from astral_data_collect.pipeline.convert_to_lerobot import convert_session  # noqa: E402
from astral_data_collect.schema import CollectSchema  # noqa: E402
from astral_data_collect.pipeline.validate_data import validate_session  # noqa: E402
from conftest import write_raw_episode  # noqa: E402

pytest.importorskip("openpi.training.config", reason="需在 openpi uv venv 中运行")

from openpi.models import pi0_config  # noqa: E402
from openpi.shared import normalize as openpi_normalize  # noqa: E402
from openpi.training import config as openpi_config  # noqa: E402
from openpi.training import data_loader as openpi_data_loader  # noqa: E402

HORIZON = 8  # 测试用小 horizon，语义验证不依赖 50
MODEL = pi0_config.Pi0Config(pi05=True, action_horizon=HORIZON, discrete_state_input=False)
DEFAULT_PROMPT = "perform the manipulation task"


# --------------------------------------------------------------------------
# 工具
# --------------------------------------------------------------------------

def _run_chain(name: str, schema: CollectSchema, tasks: list[str],
               *, fps: int = 30, duration_s: float = 2.5,  # 须 >= 校验最短时长
               nan_ep: int | None = None, nan_stream: str | None = None) -> str:
    """造 session → 对齐 → 校验(NaN 段隔离) → 转换，返回 repo_id。"""
    session = os.path.join(_ADV_HOME, f"_session_{name}")
    for i, task in enumerate(tasks):
        kw = {}
        if nan_ep == i:
            kw["nan_in"] = nan_stream
        write_raw_episode(
            os.path.join(session, f"episode{i:06d}"), schema=schema,
            duration_s=duration_s, fps=fps, task=task, **kw,
        )
        assert align_episode(os.path.join(session, f"episode{i:06d}")) is not None

    summary = validate_session(session, apply=True, log=lambda *a: None)
    if nan_ep is not None:
        assert summary["fail"] == 1 and summary["quarantined"] == 1, summary
    else:
        assert summary["fail"] == 0, summary

    repo_id = f"adv/{name}"
    out = os.path.join(_ADV_HOME, "adv", name)
    convert_session(session, out, codec="h264", log=lambda *a: None)
    return repo_id


def _make_dcfg(repo_id: str, camera_map: dict[str, str], assets: str):
    cfg = openpi_config.LeRobotAstralDataConfig(
        repo_id=repo_id,
        base_config=openpi_config.DataConfig(prompt_from_task=True),
        default_prompt=DEFAULT_PROMPT,
        camera_map=camera_map,
    )
    import pathlib
    return cfg.create(pathlib.Path(assets), MODEL)


def _check_openpi_semantics(repo_id: str, schema: CollectSchema,
                            camera_map: dict[str, str], expect_delta: list[bool],
                            tmp_path) -> None:
    """openpi 侧逐维语义验证（delta/绝对、截断维、相机接线、prompt）。"""
    dcfg = _make_dcfg(repo_id, camera_map, str(tmp_path))

    # 1) delta 掩码与截断维
    da = [t for t in dcfg.data_transforms.inputs
          if type(t).__name__ == "DeltaActions"][0]
    assert list(da.mask) == expect_delta
    assert dcfg.data_transforms.outputs[-1].action_dim == schema.state_dim

    # 2) 加载（跳过归一化，语义在原始量纲上验证）
    raw = openpi_data_loader.create_torch_dataset(dcfg, HORIZON, MODEL)
    ds = openpi_data_loader.transform_dataset(raw, dcfg, skip_norm_stats=True)
    it = ds[0]
    assert it["state"].shape == (MODEL.action_dim,)
    assert it["actions"].shape == (HORIZON, MODEL.action_dim)

    # 3) delta 语义：actions[k] = raw_action[k] - raw_state[0]（对 chunk 起始
    #    state 取差，不是逐步差分）；ee 维保持绝对（= next_state 原值）
    s = [np.asarray(raw[i]["observation.state"]) for i in range(3)]
    acts = np.asarray(it["actions"])
    for k in (0, 1):
        expect = s[k + 1].copy()  # next_state: action[t]=state[t+1]
        for d in range(schema.state_dim):
            if expect_delta[d]:
                expect[d] = s[k + 1][d] - s[0][d]
        assert np.allclose(acts[k, : schema.state_dim], expect, atol=1e-4), \
            f"k={k} 语义不符"

    # 4) 相机接线：槽位图像颜色指纹 == camera_map 指向相机的指纹
    #    conftest 第 ci 个相机颜色 (30+60ci, 100, 200-30ci)；图像在数据变换
    #    阶段保持 uint8（[-1,1] 归一化发生在模型内部）。BGR/RGB 两种通道序
    #    假设任一成立即可（通道0+2 组合对 ci 唯一）。
    schema_cams = schema.cameras
    imgs = {k: np.asarray(v) for k, v in it["image"].items()}
    masks = {k: bool(np.asarray(v)) for k, v in it["image_mask"].items()}
    for slot in ("base_0_rgb", "left_wrist_0_rgb", "right_wrist_0_rgb"):
        if slot in camera_map:
            assert masks[slot] is True, slot
            cam_key = camera_map[slot]  # "observation.images.<cam>"
            ci = schema_cams.index(cam_key.split(".")[-1])
            px = imgs[slot][112, 112].astype(np.float32)
            rgb = np.array([30 + 60 * ci, 100, 200 - 30 * ci], dtype=np.float32)
            ok_rgb = np.allclose(px, rgb, atol=45)
            ok_bgr = np.allclose(px, rgb[::-1], atol=45)
            assert ok_rgb or ok_bgr, f"{slot} 像素 {px} 不匹配相机 ci={ci}"
        else:
            assert masks[slot] is False, slot
            assert not imgs[slot].any()  # 未映射槽位 = 零填充

    # 5) 分词存在（prompt 兜底/透传至少其一生效）
    assert "tokenized_prompt" in it


# --------------------------------------------------------------------------
# 配置 1：双臂 + 双夹爪 + 三相机（16 维，三槽全映射）
# --------------------------------------------------------------------------

def test_dual_arm_grippers_3cam(tmp_path):
    schema = CollectSchema(
        arms=["left", "right"], end_effector_left="gripper",
        end_effector_right="gripper",
        cameras=["d435i", "wrist_left", "wrist_right"], dataset_fps=30,
    )
    camera_map = {
        "base_0_rgb": "observation.images.d435i",
        "left_wrist_0_rgb": "observation.images.wrist_left",
        "right_wrist_0_rgb": "observation.images.wrist_right",
    }
    repo_id = _run_chain("dual_gripper", schema, ["pick the cube", "place it"])
    with open(os.path.join(_ADV_HOME, "adv", "dual_gripper",
                           "meta", "info.json")) as f:
        feats = json.load(f)["features"]
    assert feats["observation.state"]["shape"] == [16]
    assert feats["observation.state"]["names"] == schema.state_names()
    expect_delta = [True] * 14 + [False, False]
    _check_openpi_semantics(repo_id, schema, camera_map, expect_delta, tmp_path)


# --------------------------------------------------------------------------
# 配置 2：左臂 + wuji 灵巧手 + 腰 + 头（31 维，贴近 32 上限；含空 task 段）
# --------------------------------------------------------------------------

def test_wuji_waist_head_near_limit(tmp_path):
    schema = CollectSchema(
        arms=["left"], end_effector_left="wuji", end_effector_right="none",
        include_waist=True, include_head=True,
        cameras=["d435i", "wrist_left"], dataset_fps=30,
    )
    assert schema.state_dim == 31  # 7 + 20 + 2 + 2
    camera_map = {
        "base_0_rgb": "observation.images.d435i",
        "left_wrist_0_rgb": "observation.images.wrist_left",
    }
    # ep0 空 task（兜底），ep1 正常 task
    repo_id = _run_chain("wuji_waist_head", schema, ["", "grasp the block"])
    # 臂 7 delta + 手 20 绝对 + 腰/头 4 delta
    expect_delta = [True] * 7 + [False] * 20 + [True] * 4
    _check_openpi_semantics(repo_id, schema, camera_map, expect_delta, tmp_path)

    # 空 task 兜底 vs 正常 task：两段的分词结果应当不同
    dcfg = _make_dcfg(repo_id, camera_map, str(tmp_path))
    raw = openpi_data_loader.create_torch_dataset(dcfg, HORIZON, MODEL)
    ds = openpi_data_loader.transform_dataset(raw, dcfg, skip_norm_stats=True)
    # ep1 起点 = ep0 实际帧数（从 parquet 读，对齐后帧数可能多于 fps×时长）
    import pyarrow.parquet as pq
    ep0_len = pq.ParquetFile(os.path.join(
        _ADV_HOME, "adv", "wuji_waist_head", "data", "chunk-000",
        "episode_000000.parquet")).metadata.num_rows
    tok_empty = np.asarray(ds[0]["tokenized_prompt"])
    tok_task = np.asarray(ds[ep0_len]["tokenized_prompt"])
    assert tok_empty.any() and tok_task.any()
    assert not np.array_equal(tok_empty, tok_task)


# --------------------------------------------------------------------------
# 配置 3：双臂 + 腰头 + 15fps + 非对称相机映射（右腕映射，左腕零填充）
# --------------------------------------------------------------------------

def test_dual_arm_waist_head_15fps_asym_cams(tmp_path):
    schema = CollectSchema(
        arms=["left", "right"], end_effector_left="gripper",
        end_effector_right="gripper", include_waist=True, include_head=True,
        cameras=["d435i", "wrist_left", "wrist_right"], dataset_fps=15,
    )
    assert schema.state_dim == 20  # 14 + 2 + 4
    camera_map = {
        "base_0_rgb": "observation.images.d435i",
        "right_wrist_0_rgb": "observation.images.wrist_right",
    }
    repo_id = _run_chain("dual_15fps", schema, ["task a", "task b"], fps=15)
    expect_delta = [True] * 14 + [False, False] + [True] * 4
    _check_openpi_semantics(repo_id, schema, camera_map, expect_delta, tmp_path)


# --------------------------------------------------------------------------
# 配置 4：NaN 段被校验隔离后，openpi 只看到健康段
# --------------------------------------------------------------------------

def test_nan_episode_quarantined(tmp_path):
    schema = CollectSchema(
        arms=["left", "right"], end_effector_left="gripper",
        end_effector_right="gripper",
        cameras=["d435i", "wrist_left"], dataset_fps=30,
    )
    camera_map = {
        "base_0_rgb": "observation.images.d435i",
        "left_wrist_0_rgb": "observation.images.wrist_left",
    }
    repo_id = _run_chain("nan_q", schema, ["good a", "bad", "good b"],
                         nan_ep=1, nan_stream="right_arm_state")
    with open(os.path.join(_ADV_HOME, "adv", "nan_q", "meta", "info.json")) as f:
        info = json.load(f)
    assert info["total_episodes"] == 2  # 坏段已隔离
    expect_delta = [True] * 14 + [False, False]
    _check_openpi_semantics(repo_id, schema, camera_map, expect_delta, tmp_path)


# --------------------------------------------------------------------------
# 配置 5：norm stats 全量计算 + 归一化链路（用 31 维 wuji 配置，最严苛）
# --------------------------------------------------------------------------

class _RemoveStrings:
    def __call__(self, x: dict) -> dict:
        return {k: v for k, v in x.items()
                if not np.issubdtype(np.asarray(v).dtype, np.str_)}


def test_norm_stats_roundtrip_31dim(tmp_path):
    schema = CollectSchema(
        arms=["left"], end_effector_left="wuji", end_effector_right="none",
        include_waist=True, include_head=True,
        cameras=["d435i", "wrist_left"], dataset_fps=30,
    )
    camera_map = {
        "base_0_rgb": "observation.images.d435i",
        "left_wrist_0_rgb": "observation.images.wrist_left",
    }
    repo_id = _run_chain("wuji_stats", schema, ["task x", "task y"])
    assets = str(tmp_path / "assets")

    # 复刻 scripts/compute_norm_stats.py 的核心：repack+data transforms 后统计
    dcfg = _make_dcfg(repo_id, camera_map, assets)
    raw = openpi_data_loader.create_torch_dataset(dcfg, HORIZON, MODEL)
    tds = openpi_data_loader.TransformedDataset(
        raw, [*dcfg.repack_transforms.inputs, *dcfg.data_transforms.inputs,
              _RemoveStrings()])
    stats = {k: openpi_normalize.RunningStats() for k in ("state", "actions")}
    for i in range(len(tds)):
        item = tds[i]
        for k in stats:
            stats[k].update(np.asarray(item[k])[None])
    openpi_normalize.save(os.path.join(assets, repo_id),
                          {k: s.get_statistics() for k, s in stats.items()})

    # 重create（加载 norm stats）→ 完整模型变换后必须有限且形状正确
    dcfg2 = _make_dcfg(repo_id, camera_map, assets)
    assert dcfg2.norm_stats is not None
    ds = openpi_data_loader.transform_dataset(
        openpi_data_loader.create_torch_dataset(dcfg2, HORIZON, MODEL), dcfg2)
    it = ds[0]
    assert np.all(np.isfinite(np.asarray(it["state"])))
    assert np.all(np.isfinite(np.asarray(it["actions"])))
    assert it["actions"].shape == (HORIZON, MODEL.action_dim)
    # 归一化后（quantile）大部分值应落在 [-1, 1] 附近，放 3.0 容差防极端值
    assert np.abs(np.asarray(it["actions"])).max() < 3.0


# --------------------------------------------------------------------------
# 负例 1：总维数 > 32（双臂 + 左 wuji + 右夹爪 = 35）——转换可过，openpi 必须拒
# --------------------------------------------------------------------------

def test_over_32_dims_rejected(tmp_path):
    schema = CollectSchema(
        arms=["left", "right"], end_effector_left="wuji",
        end_effector_right="gripper",
        cameras=["d435i", "wrist_left"], dataset_fps=30,
    )
    assert schema.state_dim == 35  # 7 + 7 + 20 + 1
    repo_id = _run_chain("over35", schema, ["task"])
    # 转换链本身对任意维度都能转（schema 驱动）……
    with open(os.path.join(_ADV_HOME, "adv", "over35", "meta", "info.json")) as f:
        assert json.load(f)["features"]["observation.state"]["shape"] == [35]
    # ……但 openpi 侧必须明确拒绝而不是静默错训
    with pytest.raises(ValueError, match="exceeds model action_dim"):
        _make_dcfg(repo_id, {"base_0_rgb": "observation.images.d435i"},
                   str(tmp_path))


# --------------------------------------------------------------------------
# 负例 2：camera_map 指向数据集里不存在的相机
# --------------------------------------------------------------------------

def test_camera_map_unknown_key_rejected(tmp_path):
    schema = CollectSchema(
        arms=["left"], end_effector_left="gripper", end_effector_right="none",
        cameras=["d435i", "wrist_left"], dataset_fps=30,
    )
    repo_id = _run_chain("camneg", schema, ["task"])
    with pytest.raises(ValueError, match="not present in meta"):
        _make_dcfg(repo_id, {"base_0_rgb": "observation.images.nonexistent"},
                   str(tmp_path))


# --------------------------------------------------------------------------
# 负例 3：数据集 meta 缺失（未转换/路径错）→ 清晰报错而非诡异崩溃
# --------------------------------------------------------------------------

def test_missing_meta_clear_error(tmp_path):
    with pytest.raises(FileNotFoundError, match="Astral dataset meta not found"):
        _make_dcfg("adv/definitely_not_here",
                   {"base_0_rgb": "observation.images.d435i"}, str(tmp_path))
