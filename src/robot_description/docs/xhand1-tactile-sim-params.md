# XHAND1 触觉阵列 · 仿真参数与接法

> 面向 MuJoCo 与 Isaac Lab 2.x。触觉测点已内嵌进
> [`urdf/xhand_nero_description.urdf`](../urdf/nero/xhand_nero_description.urdf)，
> 由 [`scripts/generate_tactile_points.py`](../scripts/generate_xhand_tactile_points.py) 生成。

## 0. 一句话

每个指尖 120 个三轴测点 → URDF 里 **1200 个 link**（`<side>_hand_<finger>_tac_p###`
+ 同名 fixed joint + **命名** collision 球），MuJoCo 侧逐 geom 读接触力、Isaac 侧逐 prim
挂 ContactSensor，就得到 10 片 120 点的"触觉图"。

---

## 1. 数据来源与核对结论

| 来源 | 内容 |
|---|---|
| `points_t16_transformed+(1).json` | T16，左右手四指共用（120 点） |
| `points_t30_left_hand_transformed.json` | T30，左手拇指（120 点） |
| `points_t30_right_hand_transformed.json` | T30，右手拇指（120 点） |
| `XHAND1触觉传感器说明文档+(1).pdf` | 模组规格、材料、坐标变换、FAQ |

厂商给的 transformed 文件**已经是 URDF 坐标系**（原点 = 各指 link2 原点，即关节轴中心），
单位 **mm**，无需再变换。落地前做过这些核对：

- **点位贴合**：120/120 个点都在对应 link2 网格表面上（点到面距离中位数 0.32~0.37 mm，
  最大 1.20 mm，四指 × 左右手 + 拇指 × 左右共 10 处全部通过）；
- **点间距**：由 JSON 复算的最近邻统计与 PDF 5.1 节**逐位吻合**（下表）；
- **坐标系**：URDF 里 index 指尖 joint3 的 `origin z=0.0422482924089424`，与 PDF FAQ 引用的
  数值完全一致 → 这份 URDF 与厂商测点数据同源；
- **左右手**：四指 link2 网格左右手几何相同，故共用一份 T16；拇指左右镜像，
  所以左手 T30 的变换是 `y'=-y-16.42, z'=z+8.63`（**注意与 PDF 只写的右手公式不同，
  以文件为准** —— PDF 的矩阵只适用右手）。

## 2. 模组规格（PDF 2 节）

| 参数 | T30（拇指） | T16（其余四指） |
|---|---|---|
| 测点数 | 120（三轴阵列） | 120（三轴阵列） |
| 结构尺寸 | 30.3 × 35.3 × 15.1 mm | 16.7 × 27.0 × 13.0 mm |
| 通信 | 8 线 SPI @ 5 MHz | 8 线 SPI @ 5 MHz |
| 灵敏度 | 10 LSB/N | 10 LSB/N |
| 供电 / 电流 | 3.3 V / ≤300 mA | 3.3 V / ≤300 mA |
| 工作温度 | 0 ~ 50 °C | 0 ~ 50 °C |
| 寿命 | ≥300 万次 | ≥300 万次 |

## 3. 几何参数（已实测）

| 指标 | T30 | T16 |
|---|---|---|
| 平均最近邻间距 | 2.714 mm | 2.203 mm |
| 最小最近邻间距 | 1.720 mm | 1.396 mm |
| 最大最近邻间距 | 3.436 mm | 2.585 mm |

**探针球半径怎么选**：半径必须小于最小间距的一半，否则球会互相重叠成一片。
T16 是瓶颈 → 上限 **0.698 mm**。生成器默认 `--radius 0.5`（mm），安全且接触稳定。

> 点间距也是触觉图"分辨率"的物理上限：两个相距 <1.396 mm 的接触点，
> 这套阵列分不开（PDF 5.1 节的说法）。

坐标单位：JSON 是 **mm**，URDF / 仿真器要 **m**（生成器已除 1000）。

## 4. 材料参数（PDF 5.2 节）

触觉皮弹性外层：**Mooney-Rivlin** 超弹模型

| 参数 | 值 |
|---|---|
| C1 | −0.03215 MPa |
| C2 | 0.148075 MPa |
| C3 | 0.002175 MPa |
| 密度 | 1100 kg/m³ |

**折算成线弹性**（两个仿真器都更好吃这个）：Mooney-Rivlin 的初始剪切模量
`μ = 2(C1 + C2) = 0.2319 MPa`；橡胶类近似不可压（ν ≈ 0.5），`E ≈ 3μ ≈ 0.696 MPa`。

- 注意 **C1 是负值**，与常见（C1>0, C2≥0）相反 —— 按原值使用，别"顺手改符号"。
- 这个 E 是**小应变**等效值；真实硅胶在大变形下会硬化，用线弹性只是起步。

## 5. MuJoCo 接法

### 5.1 先过一遍预处理器（必须）

厂商 URDF 直接喂 MuJoCo 会**两处报错**，用
[`scripts/prep_xnero_sim_urdf.py`](../scripts/prep_xnero_sim_urdf.py) 解决（只改副本，不动原文件）：

```bash
python3 scripts/prep_xnero_sim_urdf.py --out build/sim \
    --no-collision '^(left|right)_hand_(thumb_rota|index_rota|mid|ring|pinky)_link2$'
```

它做三件事：

1. **`package://` → 文件系统路径**（MuJoCo 不认 package://，报
   `Error opening file 'package://robot_description/meshes/nero/...'`）；
2. **修复 34 个退化惯量**：厂商把 34 个 link 的六个惯量分量（含非对角）全写成同一个值
   `1e-11`，那是秩 1 矩阵、特征值 (3e-11, 0, 0)，MuJoCo 直接拒收整个模型
   （`inertia must have positive eigenvalues`）。修法是把非对角置 0、保留对角量级。
   **这是厂商 URDF 的既存问题，和触觉无关，但不定它 MuJoCo 一步都跑不动。**
3. **可选关碰撞**（见 5.3，触觉模式必开）。

### 5.2 逐点读接触力

```python
import mujoco, numpy as np
m = mujoco.MjModel.from_xml_path("build/sim/robot_sim.urdf")
d = mujoco.MjData(m)

gid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "tac_left_index_p100")
res = np.zeros(6)
for i in range(d.ncon):
    c = d.contact[i]
    g1, g2 = int(c.geom1), int(c.geom2)
    nm = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM,
                           g2 if g1 == gid else g1)
    if not (nm or "").startswith("tac_"):
        continue
    mujoco.mj_contactForce(m, d, i, res)   # res[0] = 法向力(N)，res[1:4] = 切向
    print(nm, "法向", abs(res[0]))
```

- **geom 名就是点的身份**：`tac_<side>_<finger>_p###` → `raw_force[###-1]`。
- 厂商原有的 `<collision>` 都**没写 name**，所以在 MuJoCo 里它们只是 `geom<N>`，
  认不出来 —— 这也是触觉 collision 特意全部命名的原因。
- 默认 `fusestatic=true`：1200 个点 body 会被**融合进对应指尖 body**，
  但 geom 全部保留且名字还在（实测：`ngeom=1280`，1200 个触觉 geom 全在，
  独立触觉 body 残留 0）。所以 DOF 数不变、不增加仿真负担。
- 想让点阵保持独立 body（例如做 per-point 传感器或柔顺控制）：在 MJCF 里
  `<compiler fusestatic="false"/>`。

### 5.2b 指尖合力（两种拿法，都实测过）

**逐点读的是"点"，指尖合力要自己汇总** —— MuJoCo 不会自动给。两条路都对：

**(a) 把 120 个点加起来**（推荐：精确，而且法向、切向都能分别给）

```python
fn = ft = 0.0
for i in range(d.ncon):
    ...                       # 同上筛出 tac_ 开头的 geom
    mujoco.mj_contactForce(m, d, i, res)
    fn += abs(res[0]); ft += np.linalg.norm(res[1:3])   # 法向 / 切向
```

**(b) MuJoCo `touch` 传感器**（省事，但只给**法向**标量）：在指尖 link2 上加一个
**罩住整片 pad 的 site** + `touch` 传感器：

```xml
<!-- MJCF 侧（或在 Python 里用 MjSpec 加） -->
<site name="pad_left_index" pos="0.00487 0 0.03077" size="0.01782 0 0"/>
<sensor><touch name="ft_left_index" site="pad_left_index"/></sensor>
```

每根手指的 site 位置/半径（= 120 点的质心 + 覆盖半径 + 2 mm 余量，直接从 JSON 算出）：

| 手指 | site pos (m, link2 系) | site size (半径 m) |
|---|---|---|
| left/right · index/mid/ring/pinky | `0.00487 0 0.03077` | `0.01782` |
| left · thumb | `0  -0.03508  0.00269` | `0.02384` |
| right · thumb | `0   0.03508  0.00347` | `0.02384` |

**实测结论**（左手食指，压入 0.2 / 0.5 / 0.8 / 1.5 / 3.0 mm 五档）：
`touch` 读数与逐点法向求和**逐档完全一致（差 0.0000）**。食指+中指同时压时，
两个 touch 各读各的（2.6025 / 10.8178 N），**不串味**。

**三个必须知道的语义**（MuJoCo 头文件原文：`mjSENS_TOUCH // scalar contact normal
forces summed over sensor zone`）：

1. `touch` 统计的是 **site 体积范围内**该 body 的接触法向力 —— **site 放错位置会读 0**。
   我把 site 放在球心、接触点在球底时，读数就是 0；site 一罩住接触点立刻给出正确值。
   所以 site 必须罩住整片 pad（上表）。
2. **只给法向标量**，不给切向、不给分点分布。要切向或分布就用 (a)。
3. `touch` 在纯运动学下（只 `mj_forward`、不 step）就有值，不需要跑动力学。
   ⚠ 反过来说：如果你 `mj_step` 让手臂动起来，两次读数之间姿态已经变了 ——
   对读数时要保证在同一姿态下取。

### 5.3 ⚠ 触觉模式必须关掉指尖 mesh 的碰撞

测点本来就**贴在指尖表面**，所以探针球与指尖 mesh 是共面的。实测（压左手食指 p100，
压入 0.8 mm）：

| 配置 | 总法向力 | 触觉阵列拿到 | 指尖 mesh 拿走 |
|---|---|---|---|
| 保留 mesh 碰撞 | 9.79 N | 5.93 N（**60.6%**，且分布失真） | 3.86 N（**39.4%，触觉完全看不见**） |
| 关掉 link2 的 mesh 碰撞 | 3.12 N | **3.12 N（100%）** | 0 |

保留 mesh 时，接触力有近四成被 mesh 吃掉、而且逐点分布是错的（p099=2.56 / p100=3.37
vs 正确值 p099=0.40 / p100=2.72）。**要让逐点力有意义，得让探针阵列成为接触面。**

正则要精确到 5 个**掌侧**指尖 link2 —— 别用 `.*link2$`，那会连带关掉 nero 臂的
`left_link2/right_link2` 和手背的 `*_rotaback_link2`（实测会误伤 22 个 link 而非 10 个）：

```
^(left|right)_hand_(thumb_rota|index_rota|mid|ring|pinky)_link2$
```

### 5.4 接触刚度调参

MuJoCo **不支持 Mooney-Rivlin**，触觉皮的柔顺性要靠接触参数近似。`solref` 第一项是
时间常数 τ，等效接触刚度 `k ≈ m/τ²`（m 为接触有效质量），反解 `τ = √(m/k)`。

用 §4 的 E 估 k：`k = E·A/t`（A = 接触斑面积，t = 硅胶层厚）。取 A = π(1 mm)²、
手指 link2 质量 m ≈ 0.016 kg：

| 硅胶层厚 t | 接触刚度 k | 建议 solref |
|---|---|---|
| 1 mm | ≈ 2.2 kN/m | `0.0027 1` |
| 2 mm | ≈ 1.1 kN/m | `0.0038 1` |
| 3 mm | ≈ 0.73 kN/m | `0.0047 1` |

MuJoCo 默认 `solref="0.02 1"` 要软得多（约 40 N/m）。**建议做法**：从默认值起步，
用已知重量的物体压上去，把仿真穿透深度调到与真实传感器一致，再回推 k。
τ 不要小于 2×仿真步长（步长 1e-3 → τ ≥ 0.002）。

### 5.5 自检

```bash
python3 scripts/test_xhand_tactile_mujoco.py            # 压一下指尖，打印逐点力（PASS/FAIL）
python3 scripts/test_xhand_tactile_mujoco.py --sweep    # 扫压入深度
python3 scripts/test_xhand_tactile_mujoco.py --with-mesh  # 对照组：看 mesh 抢力
```

## 6. Isaac Lab 2.x 接法

### 6.1 导入时别合并 fixed joint

```python
from isaaclab.sim.converters import UrdfConverterCfg
cfg = UrdfConverterCfg(
    asset_path=".../urdf/nero/xhand_nero_description.urdf",
    merge_fixed_joints=False,   # ← 关键：True 会把这 1200 个点 prim 合并掉
    fix_base=True,
    joint_drive=UrdfConverterCfg.JointDriveCfg(target_type="position", gains=...),
)
```

> Isaac 认 `package://`，但若走文件系统路径导入，直接用 `prep_xnero_sim_urdf.py --keep-mesh`
> 产出的副本即可（惯量修复对 PhysX 同样有益）。

### 6.2 逐点接触力 → 触觉图

```python
from isaaclab.sensors import ContactSensorCfg
tactile = ContactSensorCfg(
    prim_path="{ENV_REGEX_NS}/Robot/.*_tac_p.*",   # 1200 个点 prim
    history_length=1, track_air_time=False,
    update_period=0.0,                              # 每个物理步都更
)
# 读数 net_forces_w: (N, 1200, 3) → 按 <side>/<finger> 分组 reshape
forces = sensor.data.net_forces_w          # (N, 1200, 3)
tactile_img = forces.norm(dim=-1).view(N, 10, 120)   # 10 片 120 点触觉图
```

prim 命名与 MuJoCo geom 名一致（`tac_<side>_<finger>_p###`），所以
`tactile_img[..., k]` 就对应 `raw_force[k]`。

> **性能提示**：1200 个带碰撞的 rigid body 是真实开销。只做特定手/指的训练时，
> 把 `prim_path` 收窄到需要的指；纯推物不测触觉时，`merge_fixed_joints=True` 导入即可
> 退回轻量（点 prim 消失，URDF 不用改）。
>
> Isaac Lab 2.x 另有 tactile 传感器框架（`isaaclab.sensors` 下的 TactileSensor /
> VisuoTactileSensor 系列）；它是消费同一批 prim 的接触/相机数据，你的版本里若有，
> 可以直接把上面的 prim_path 喂给它，省掉手写 reshape。接口以你本地版本为准。

### 6.3 触觉皮材料

PhysX 变形体材质吃的是线弹性参数，用 §4 折算值起步：

| 属性 | 值 |
|---|---|
| youngsModulus | 0.696 MPa（= 6(C1+C2)，ν≈0.5 折算；见 §4） |
| poissonsRatio | 0.4995（近不可压） |
| density | 1100 kg/m³ |

Mooney-Rivlin 的 C1/C2/C3 能否直接用取决于你的 Isaac Sim 版本的变形体超弹选项
（PhysX 5.x 的 FEM deformable 提供超弹模型，具体枚举以本地版本为准）；
用线弹性 E 起步、按接触深度校准，是最稳的路径。

> 好消息：PDF 5.3 节称 **Isaac Sim 5.2 将官方支持 XHAND1 触觉传感器**，
> 到那时可以优先用官方资产，本文档的几何/材料参数仍可作为对照。

## 7. 已知限制与坑

1. **`raw_force[k]` ↔ `point k+1` 的序号对应厂商文档没有明示** —— 本文按自然顺序
   假设，首次上实机请用**单点按压**验证一次（点哪个点，看哪个索引动）。
2. **别靠近强磁**：传感器是霍尔原理，强磁会干扰甚至永久损坏（PDF 1.2 节）。
3. **量化噪声**：灵敏度 10 LSB/N → 1 LSB = 0.1 N。做 sim-to-real 时给读数加
   0.1 N 量化，比加高斯噪声更接近真实传感器行为。
4. **MuJoCo `mjMINVAL = 1e-15`**：显式写的惯量若 ≤1e-15 会被拒收。默认生成器
   **不写 `<inertial>`**，让仿真器按球体几何推导（MuJoCo 默认密度 1000 →
   每球 5.24e-7 kg，每指约 +0.063 g）。要显式控制就 `--density 1100`。
5. **工作温度 0~50 °C**：触觉读数带温度通道（每指 20 个原始温度点 + 1 个计算温度），
   仿真里通常忽略，但 sim-to-real 时注意温漂。
7. **指尖末端的几排测点会被指尖 mesh 遮挡**（实测）：URDF 里 `*_rota_tip` 与
   `*_rotaback_link2` 是 `*_rota_link2` 的 fixed 子链，MuJoCo 导入时（`fusestatic`）
   会把它们的碰撞 mesh 融进**同一个 body**。而 T16 的 120 点是**列优先网格**（每列
   12 点沿手指方向），第 12 行（p060/p072/... ）的 z≈42.08 mm 正好贴着指尖关节
   （z=42.25 mm）—— 用 4 mm 的球去压会先顶到那片 mesh，触觉点读不到力。
   要整片 pad 都干净：把 `make_mjcf.py` 的 `PAD_LINK2_RE` 扩到含 `_tip`（代价是
   指尖末端不再参与碰撞）。用小球（≤1 mm）压 pad 中段则不受影响。
8. **仿真触觉的坐标系与真机不同**：仿真给的是**指尖 link2 坐标系**下的三分量
   （120 点同一坐标系，可直接求和/看方向）；真机传感器给的是**传感器自身三轴**的
   分量。幅值可比，分量方向需按安装姿态换算。
9. 触觉点**只在指尖掌侧**（link2 段，z ∈ 18~42 mm）；`*_tip` 段没有测点，
   它仍是纯几何碰撞体。
