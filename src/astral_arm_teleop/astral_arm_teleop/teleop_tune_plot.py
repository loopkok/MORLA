#!/usr/bin/env python3
"""Live teleop tuner: overlay hand vs filter vs commanded EE (PID-style).

Subscribe to ``/teleop/{side}/tune/xyz`` published by
``astral_arm_teleop_node`` (also ``ee_vr`` / ``ee_filt`` / ``ee_cmd`` for
PlotJuggler). Positions are millimetres relative to the zero you set
(first sample, or press ``z`` while holding still).

How to read
-----------
* Hand (VR) and EE overlap on a 1–2 cm move → 跟手.
* EE wiggles while Hand is flat → 控制/IK 抖.
* Hand itself wiggles → Quest 跟踪噪声（降平滑会更跟、更抖）.
* EE smooth but lags Hand → ``pos_smoothing`` 太大.
* Hand amplitude > EE amplitude → ``motion_scale`` 已含在 VR 映射里；
  若仍偏小，是 IK/限速没跟上，不是 scale 没乘.

Keys: ``z`` re-zero, ``s`` save CSV of the visible buffer, ``q`` quit.
Bottom sliders: feel params (both solvers) + URDF LM weights.
Radio: hot-switch ``analytic_dh`` / ``urdf_numerical``.
"""

from __future__ import annotations

import csv
import threading
import time
from collections import deque
from datetime import datetime
from pathlib import Path

import numpy as np
import rclpy
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from scipy.spatial.transform import Rotation
from std_msgs.msg import Float64MultiArray

_HOT = (
    "pos_smoothing",
    "rot_smoothing",
    "motion_scale",
    "max_joint_vel",
    "workspace_radius",
    "ik_w_pos",
    "ik_w_ori",
    "ik_w_reg",
    "ik_q4_max",
    "ik_w_limit",
    "ik_max_iter",
    "ik_tol",
    "solver_type",
)
_DEFAULTS = {
    "pos_smoothing": 0.8,
    "rot_smoothing": 0.8,
    "motion_scale": 0.65,
    "max_joint_vel": 4.0,
    "workspace_radius": 0.55,
    "ik_w_pos": 1.0,
    "ik_w_ori": 0.3,
    "ik_w_reg": 1e-4,
    "ik_q4_max": -0.45,
    "ik_w_limit": 0.12,
    "ik_max_iter": 20.0,
    "ik_tol": 1e-8,
    "solver_type": "geometric",
}
_INT_PARAMS = {"ik_max_iter"}
_STR_PARAMS = {"solver_type"}


def _xcorr_lag_ms(ref: np.ndarray, sig: np.ndarray, dt: float) -> float | None:
    x = np.asarray(ref, dtype=float)
    y = np.asarray(sig, dtype=float)
    if x.size < 40 or dt <= 0:
        return None
    x = x - x.mean()
    y = y - y.mean()
    if x.std() < 1e-4 or y.std() < 1e-4:
        return None
    max_lag = max(1, int(0.25 / dt))
    c = np.correlate(y, x, mode="full")
    lags = np.arange(-len(x) + 1, len(x))
    mid = len(x) - 1
    lo = max(0, mid - max_lag)
    hi = min(len(c), mid + max_lag + 1)
    i = lo + int(np.argmax(c[lo:hi]))
    return float(lags[i] * dt * 1000.0)


class TeleopTunePlot(Node):
    def __init__(self) -> None:
        super().__init__("teleop_tune_plot")
        self.declare_parameter("arm_side", "right")
        self.declare_parameter("window_sec", 8.0)
        self.declare_parameter("metric_sec", 2.0)
        self.declare_parameter("save_dir", "/tmp")
        self.declare_parameter("apply_both_arms", True)
        self.declare_parameter("teleop_left", "astral_arm_teleop_left")
        self.declare_parameter("teleop_right", "astral_arm_teleop_right")

        self.side = str(self.get_parameter("arm_side").value).lower()
        if self.side not in ("left", "right"):
            raise ValueError("arm_side must be left|right")
        self.window_sec = float(self.get_parameter("window_sec").value)
        self.metric_sec = float(self.get_parameter("metric_sec").value)
        self.save_dir = Path(str(self.get_parameter("save_dir").value))
        self.sync_both = bool(self.get_parameter("apply_both_arms").value)
        self._left_name = str(self.get_parameter("teleop_left").value).strip("/")
        self._right_name = str(self.get_parameter("teleop_right").value).strip("/")
        self._set_cli = {}
        self._get_cli = {}
        self._pending: dict = {}
        self._sent: dict = {}
        self._param_status = "params: waiting"

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=50,
        )
        prefix = f"/teleop/{self.side}/tune"
        self.create_subscription(
            Float64MultiArray, f"{prefix}/xyz", self._on_xyz, qos
        )

        self._lock = threading.Lock()
        n = max(300, int(self.window_sec * 200))
        self._t: deque[float] = deque(maxlen=n)
        self._vr: deque[np.ndarray] = deque(maxlen=n)
        self._filt: deque[np.ndarray] = deque(maxlen=n)
        self._cmd: deque[np.ndarray] = deque(maxlen=n)
        self._vr_q: deque[np.ndarray] = deque(maxlen=n)
        self._filt_q: deque[np.ndarray] = deque(maxlen=n)
        self._cmd_q: deque[np.ndarray] = deque(maxlen=n)

        self._zero: np.ndarray | None = None
        self._zero_R: Rotation | None = None
        self._rezero = False
        self._save_req = False
        self._last_metric_print = 0.0
        self._metric_text = f"waiting for {prefix}/xyz ..."

        self.get_logger().info(
            f"teleop tune plot: {self.side}  subscribing {prefix}/xyz  "
            "keys: z=zero  s=save csv  q=quit"
        )

    def _on_xyz(self, msg: Float64MultiArray) -> None:
        d = np.asarray(msg.data, dtype=float)
        if d.size < 9:
            return
        vr_p, fi_p, cmd_p = d[0:3], d[3:6], d[6:9]
        if d.size >= 21:
            vr_q, fi_q, cmd_q = d[9:13], d[13:17], d[17:21]
        else:
            vr_q = fi_q = cmd_q = np.array([0.0, 0.0, 0.0, 1.0])
        t = time.monotonic()
        with self._lock:
            if self._rezero or self._zero is None:
                self._zero = cmd_p.copy()
                nq = float(np.linalg.norm(cmd_q))
                if nq > 0.5:
                    self._zero_R = Rotation.from_quat(cmd_q)
                self._rezero = False
                self._t.clear()
                self._vr.clear()
                self._filt.clear()
                self._cmd.clear()
                self._vr_q.clear()
                self._filt_q.clear()
                self._cmd_q.clear()
            self._t.append(t)
            self._vr.append(np.asarray(vr_p, dtype=float).copy())
            self._filt.append(np.asarray(fi_p, dtype=float).copy())
            self._cmd.append(np.asarray(cmd_p, dtype=float).copy())
            self._vr_q.append(np.asarray(vr_q, dtype=float).copy())
            self._filt_q.append(np.asarray(fi_q, dtype=float).copy())
            self._cmd_q.append(np.asarray(cmd_q, dtype=float).copy())

    def snapshot(self):
        with self._lock:
            if len(self._t) < 5 or self._zero is None:
                return None
            t = np.fromiter(self._t, dtype=float, count=len(self._t))
            vr = np.vstack(self._vr)
            filt = np.vstack(self._filt)
            cmd = np.vstack(self._cmd)
            vr_q = np.vstack(self._vr_q)
            filt_q = np.vstack(self._filt_q)
            cmd_q = np.vstack(self._cmd_q)
            zero = self._zero.copy()
            zero_R = self._zero_R
            save = self._save_req
            self._save_req = False
        t0 = t[-1] - self.window_sec
        m = t >= t0
        return {
            "t": t[m] - t[m][-1],
            "vr": (vr[m] - zero) * 1e3,
            "filt": (filt[m] - zero) * 1e3,
            "cmd": (cmd[m] - zero) * 1e3,
            "vr_q": vr_q[m],
            "filt_q": filt_q[m],
            "cmd_q": cmd_q[m],
            "zero_R": zero_R,
            "save": save,
        }

    def compute_metrics(self, snap: dict) -> dict:
        t = snap["t"]
        vr, filt, cmd = snap["vr"], snap["filt"], snap["cmd"]
        dt = float(np.median(np.diff(t))) if t.size > 2 else 0.007
        t_abs = t - t[0]
        m2 = t_abs >= (t_abs[-1] - self.metric_sec)
        if m2.sum() < 10:
            m2 = np.ones(t.size, dtype=bool)
        e = vr[m2] - cmd[m2]
        e_n = np.linalg.norm(e, axis=1)
        rms = float(np.sqrt(np.mean(e_n**2)))
        p95 = float(np.percentile(e_n, 95))
        e_filt = np.linalg.norm(filt[m2] - cmd[m2], axis=1)
        ik_rms = float(np.sqrt(np.mean(e_filt**2)))
        vr_std = float(np.linalg.norm(vr[m2].std(axis=0)))
        cmd_std = float(np.linalg.norm(cmd[m2].std(axis=0)))
        hold = vr_std < 2.0
        jitter = cmd_std if hold else float("nan")
        axis = int(np.argmax(vr[m2].std(axis=0)))
        lag = _xcorr_lag_ms(vr[m2, axis], cmd[m2, axis], max(dt, 1e-4))
        ori = None
        if snap["zero_R"] is not None:
            try:
                r0 = snap["zero_R"]
                ev = Rotation.from_quat(snap["vr_q"][m2])
                ec = Rotation.from_quat(snap["cmd_q"][m2])
                ang = (r0.inv() * ev).inv() * (r0.inv() * ec)
                ori = float(np.sqrt(np.mean(np.degrees(ang.magnitude()) ** 2)))
            except Exception:  # noqa: BLE001
                ori = None
        lag_s = "—" if lag is None else f"{lag:.0f} ms"
        jit_s = (
            f"{jitter:.2f} mm" if hold else f"—  手在动 {vr_std:.1f} mm"
        )
        ori_s = "—" if ori is None else f"{ori:.2f}°"
        now = time.monotonic()
        if now - self._last_metric_print > 2.0:
            self._last_metric_print = now
            extra = f"  HOLD jitter={jitter:.2f}mm" if hold else ""
            self.get_logger().info(
                f"[Tune][{self.side}] pos_rms={rms:.2f}mm p95={p95:.2f}mm "
                f"ik_rms={ik_rms:.2f}mm lag={lag_s} ori={ori_s}{extra}"
            )
        return {
            "rms": f"{rms:.2f} mm",
            "p95": f"{p95:.2f} mm",
            "ik": f"{ik_rms:.2f} mm",
            "jitter": jit_s,
            "lag": lag_s,
            "ori": ori_s,
            "status": self._param_status,
        }

    def save_csv(self, snap: dict) -> None:
        self.save_dir.mkdir(parents=True, exist_ok=True)
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        path = self.save_dir / f"teleop_tune_{self.side}_{ts}.csv"
        with path.open("w", newline="") as f:
            w = csv.writer(f)
            w.writerow(
                [
                    "t_rel_s",
                    "vr_x_mm",
                    "vr_y_mm",
                    "vr_z_mm",
                    "filt_x_mm",
                    "filt_y_mm",
                    "filt_z_mm",
                    "cmd_x_mm",
                    "cmd_y_mm",
                    "cmd_z_mm",
                    "err_x_mm",
                    "err_y_mm",
                    "err_z_mm",
                    "err_norm_mm",
                ]
            )
            err = snap["vr"] - snap["cmd"]
            for i in range(snap["t"].size):
                w.writerow(
                    [
                        f"{snap['t'][i]:.4f}",
                        *snap["vr"][i],
                        *snap["filt"][i],
                        *snap["cmd"][i],
                        *err[i],
                        np.linalg.norm(err[i]),
                    ]
                )
        self.get_logger().info(f"saved {path}")

    def _target_nodes(self) -> list[str]:
        if self.sync_both:
            return [self._left_name, self._right_name]
        return [self._left_name if self.side == "left" else self._right_name]

    def _client(self, kind: str, node_name: str):
        cache = self._set_cli if kind == "set" else self._get_cli
        if node_name not in cache:
            srv = SetParameters if kind == "set" else GetParameters
            cache[node_name] = self.create_client(srv, f"/{node_name}/{kind}_parameters")
        return cache[node_name]

    def fetch_params(self, timeout: float = 1.5) -> dict:
        out = dict(_DEFAULTS)
        target = self._left_name if self.side == "left" else self._right_name
        cli = self._client("get", target)
        if not cli.wait_for_service(timeout_sec=timeout):
            self.get_logger().warn(f"no {target}/get_parameters, using defaults")
            return out
        req = GetParameters.Request()
        req.names = list(_HOT)
        fut = cli.call_async(req)
        deadline = time.monotonic() + timeout
        while not fut.done() and time.monotonic() < deadline:
            time.sleep(0.02)
        if not fut.done() or fut.result() is None:
            return out
        for name, val in zip(_HOT, fut.result().values):
            if val.type == ParameterType.PARAMETER_DOUBLE:
                out[name] = float(val.double_value)
            elif val.type == ParameterType.PARAMETER_INTEGER:
                out[name] = float(val.integer_value)
            elif val.type == ParameterType.PARAMETER_STRING:
                out[name] = str(val.string_value)
        self._sent = dict(out)
        st = out.get("solver_type", "?")
        self._param_status = f"solver={st}"
        return out

    def queue_param(self, name: str, value) -> None:
        self._pending[name] = value

    def _param_msg(self, name: str, value) -> Parameter:
        p = Parameter()
        p.name = name
        if name in _STR_PARAMS:
            p.value = ParameterValue(
                type=ParameterType.PARAMETER_STRING, string_value=str(value)
            )
        elif name in _INT_PARAMS:
            p.value = ParameterValue(
                type=ParameterType.PARAMETER_INTEGER,
                integer_value=int(round(float(value))),
            )
        else:
            p.value = ParameterValue(
                type=ParameterType.PARAMETER_DOUBLE, double_value=float(value)
            )
        return p

    def flush_params(self) -> None:
        pending = self._pending
        if not pending:
            return

        def _changed(k, v) -> bool:
            prev = self._sent.get(k)
            if isinstance(v, str) or k in _STR_PARAMS:
                return prev != v
            if prev is None:
                return True
            try:
                return abs(float(v) - float(prev)) > 1e-12
            except (TypeError, ValueError):
                return prev != v

        to_send = {k: v for k, v in pending.items() if _changed(k, v)}
        self._pending = {}
        if not to_send:
            return
        params = [self._param_msg(k, v) for k, v in to_send.items()]
        ok_nodes = []
        for target in self._target_nodes():
            cli = self._client("set", target)
            if not cli.service_is_ready():
                if not cli.wait_for_service(timeout_sec=0.05):
                    continue
            req = SetParameters.Request()
            req.parameters = params
            cli.call_async(req)
            ok_nodes.append(target)
        self._sent.update(to_send)
        if "solver_type" in to_send:
            with self._lock:
                self._rezero = True
        who = "+".join(ok_nodes) if ok_nodes else "none"
        bits = []
        for k, v in to_send.items():
            if isinstance(v, str):
                bits.append(f"{k}={v}")
            elif k == "ik_tol" or k == "ik_w_reg":
                bits.append(f"{k}={float(v):.1e}")
            else:
                bits.append(f"{k}={float(v):.2f}")
        self._param_status = f"hot → {who}  " + "  ".join(bits)
        now = time.monotonic()
        if now - getattr(self, "_last_param_log", 0.0) > 0.8:
            self._last_param_log = now
            self.get_logger().info(self._param_status)


def _pick_cjk_font():
    from matplotlib import font_manager as fm

    candidates = [
        "/usr/share/fonts/truetype/wqy/wqy-microhei.ttc",
        "/usr/share/fonts/truetype/droid/DroidSansFallbackFull.ttf",
        "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
        "/usr/share/fonts/opentype/noto/NotoSansCJK-Black.ttc",
        "/usr/share/fonts/truetype/arphic/uming.ttc",
    ]
    for path in candidates:
        if not Path(path).is_file():
            continue
        try:
            fm.fontManager.addfont(path)
        except Exception:  # noqa: BLE001
            pass
        return fm.FontProperties(fname=path).get_name()
    return None


_COL = {
    "bg": "#F3F4F6",
    "card": "#FFFFFF",
    "ink": "#111827",
    "muted": "#6B7280",
    "line": "#E5E7EB",
    "hand": "#2563EB",
    "filt": "#B45309",
    "ee": "#047857",
    "err": "#DC2626",
    "accent": "#2563EB",
    "on": "#111827",
}


def _style_plot_ax(ax) -> None:
    ax.set_facecolor(_COL["card"])
    ax.tick_params(colors=_COL["muted"], labelsize=8, length=3)
    ax.yaxis.label.set_color(_COL["ink"])
    ax.xaxis.label.set_color(_COL["muted"])
    for name, spine in ax.spines.items():
        spine.set_color(_COL["line"])
        if name in ("top", "right"):
            spine.set_visible(False)
    ax.grid(True, color=_COL["line"], linewidth=0.8, alpha=1.0)
    ax.set_axisbelow(True)


def _style_card(ax) -> None:
    ax.set_facecolor(_COL["card"])
    ax.set_xticks([])
    ax.set_yticks([])
    for spine in ax.spines.values():
        spine.set_color(_COL["line"])
        spine.set_linewidth(1.0)


def _style_slider(sl, color: str) -> None:
    sl.poly.set_fc(color)
    try:
        sl.track.set_facecolor("#E5E7EB")
        sl.track.set_edgecolor(_COL["line"])
    except Exception:  # noqa: BLE001
        pass
    sl.valtext.set_color(_COL["ink"])
    sl.valtext.set_fontsize(8)
    sl.ax.set_facecolor(_COL["card"])
    for spine in sl.ax.spines.values():
        spine.set_visible(False)


def _run_plot(node: TeleopTunePlot) -> None:
    try:
        import matplotlib.pyplot as plt
        from matplotlib.animation import FuncAnimation
        from matplotlib.ticker import MultipleLocator
        from matplotlib.widgets import Button, Slider
    except ImportError as exc:
        raise SystemExit("need matplotlib: pip install matplotlib") from exc

    init = node.fetch_params()
    font_name = _pick_cjk_font()
    plt.rcParams.update(
        {
            "font.size": 10,
            "font.family": "sans-serif",
            "font.sans-serif": [font_name, "DejaVu Sans"]
            if font_name
            else ["DejaVu Sans"],
            "axes.unicode_minus": False,
            "figure.facecolor": _COL["bg"],
            "axes.facecolor": _COL["card"],
            "text.color": _COL["ink"],
        }
    )

    fig = plt.figure(figsize=(14.6, 9.2), dpi=110)
    fig.canvas.manager.set_window_title(f"Astral 跟手调参 · {node.side}")
    fig.patch.set_facecolor(_COL["bg"])

    gs = fig.add_gridspec(
        2,
        1,
        height_ratios=[2.55, 1.15],
        hspace=0.16,
        left=0.045,
        right=0.975,
        top=0.955,
        bottom=0.045,
    )
    gs_top = gs[0].subgridspec(1, 2, width_ratios=[3.35, 1.0], wspace=0.10)
    gs_plots = gs_top[0].subgridspec(4, 1, hspace=0.10)
    gs_bot = gs[1].subgridspec(1, 3, width_ratios=[1.2, 1.2, 0.72], wspace=0.10)

    axes = [fig.add_subplot(gs_plots[i]) for i in range(4)]
    for ax in axes[1:]:
        ax.sharex(axes[0])
    ylabels = ["X  mm", "Y  mm", "Z  mm", "误差  mm"]
    lines = []
    for i, (ax, ylab) in enumerate(zip(axes, ylabels)):
        _style_plot_ax(ax)
        ax.set_ylabel(ylab, fontsize=9)
        if i < 3:
            (l0,) = ax.plot([], [], color=_COL["hand"], lw=1.7, label="手  VR")
            (l1,) = ax.plot(
                [],
                [],
                color=_COL["filt"],
                lw=1.2,
                ls=(0, (4, 2)),
                label="滤波目标",
            )
            (l2,) = ax.plot([], [], color=_COL["ee"], lw=1.7, label="末端  EE")
            lines.append((l0, l1, l2))
        else:
            (le,) = ax.plot([], [], color=_COL["err"], lw=1.5, label="|手 − 末端|")
            lines.append((le,))
        ax.set_autoscalex_on(False)
        ax.set_xlim(-float(node.window_sec), 0.0)
        ax.xaxis.set_major_locator(MultipleLocator(1.0))
    axes[-1].set_xlabel(
        "时间（秒）    左 = 过去    右 0 = 现在",
        fontsize=8,
        color=_COL["muted"],
    )
    for ax in axes[:-1]:
        ax.tick_params(labelbottom=False)

    fig.legend(
        [lines[0][0], lines[0][1], lines[0][2], lines[3][0]],
        ["手  VR", "滤波目标", "末端  EE", "|手 − 末端|"],
        loc="upper left",
        bbox_to_anchor=(0.045, 0.995),
        ncol=4,
        frameon=False,
        fontsize=9,
        handlelength=1.8,
        columnspacing=1.4,
    )
    fig.text(
        0.62,
        0.975,
        f"{'左臂' if node.side == 'left' else '右臂'}    窗口 {node.window_sec:.0f}s",
        ha="right",
        va="center",
        fontsize=11,
        color=_COL["ink"],
        fontweight="bold",
    )

    ax_side = fig.add_subplot(gs_top[1])
    _style_card(ax_side)
    ax_side.text(
        0.08,
        0.955,
        "指标",
        fontsize=13,
        fontweight="bold",
        color=_COL["ink"],
        transform=ax_side.transAxes,
    )
    ax_side.text(
        0.08,
        0.905,
        f"最近 {node.metric_sec:.0f}s    正 lag = 末端落后",
        fontsize=8,
        color=_COL["muted"],
        transform=ax_side.transAxes,
    )
    metric_rows = [
        ("rms", "位置误差 RMS"),
        ("p95", "位置误差 p95"),
        ("ik", "IK 误差 RMS"),
        ("jitter", "静持抖动"),
        ("lag", "滞后 lag"),
        ("ori", "姿态误差"),
    ]
    metric_vals = {}
    y0 = 0.80
    for i, (key, label) in enumerate(metric_rows):
        y = y0 - i * 0.095
        ax_side.text(
            0.08,
            y,
            label,
            fontsize=8,
            color=_COL["muted"],
            transform=ax_side.transAxes,
            va="center",
        )
        metric_vals[key] = ax_side.text(
            0.92,
            y - 0.028,
            "—",
            fontsize=13,
            color=_COL["ink"],
            transform=ax_side.transAxes,
            ha="right",
            va="center",
            fontweight="bold",
        )
    status_txt = ax_side.text(
        0.08,
        0.08,
        "等待 /teleop/.../tune/xyz",
        fontsize=7.5,
        color=_COL["muted"],
        transform=ax_side.transAxes,
        va="bottom",
        wrap=True,
    )
    ax_side.text(
        0.08,
        0.025,
        "Z 清零    S 存 CSV    Q 退出",
        fontsize=8,
        color=_COL["muted"],
        transform=ax_side.transAxes,
        va="bottom",
    )

    def _section(parent_gs, title: str, hint: str, n_sliders: int):
        ratios = [0.65] + [1.0] * n_sliders
        inner = parent_gs.subgridspec(
            1 + n_sliders, 1, height_ratios=ratios, hspace=0.42
        )
        ax_h = fig.add_subplot(inner[0])
        ax_h.set_facecolor(_COL["bg"])
        ax_h.axis("off")
        ax_h.text(0.0, 0.72, title, fontsize=11, fontweight="bold", color=_COL["ink"])
        ax_h.text(0.0, 0.08, hint, fontsize=8, color=_COL["muted"])
        return inner

    gs_feel = _section(gs_bot[0], "手感", "DH 与 URDF 都生效", 5)
    gs_urdf = _section(gs_bot[1], "URDF 数值 IK", "DH 闭式会忽略这些", 7)
    gs_act = gs_bot[2].subgridspec(7, 1, height_ratios=[0.7, 1, 1, 1, 1, 1, 1], hspace=0.45)

    sliders = []

    def _add_slider(parent, row, name, title, vmin, vmax, step, fmt, color, transform=None):
        cell = parent[row].subgridspec(1, 2, width_ratios=[0.42, 0.58], wspace=0.08)
        ax_l = fig.add_subplot(cell[0, 0])
        ax_l.set_facecolor(_COL["bg"])
        ax_l.axis("off")
        ax_l.text(0.0, 0.5, title, va="center", ha="left", fontsize=9, color=_COL["ink"])
        ax_s = fig.add_subplot(cell[0, 1])
        val0 = float(init.get(name, vmin))
        if transform == "log10":
            val0 = float(np.clip(np.log10(max(val0, 10**vmin)), vmin, vmax))
        else:
            val0 = float(np.clip(val0, vmin, vmax))
        sl = Slider(ax_s, "", vmin, vmax, valinit=val0, valstep=step, valfmt=fmt)
        _style_slider(sl, color)
        if transform == "log10":
            sl.on_changed(lambda v, n=name: node.queue_param(n, 10.0 ** float(v)))
        else:
            sl.on_changed(lambda v, n=name: node.queue_param(n, v))
        sliders.append(sl)
        return sl

    _add_slider(gs_feel, 1, "pos_smoothing", "位置平滑", 0.0, 0.99, 0.01, "%.2f", _COL["hand"])
    _add_slider(gs_feel, 2, "rot_smoothing", "旋转平滑", 0.0, 0.99, 0.01, "%.2f", _COL["hand"])
    _add_slider(gs_feel, 3, "motion_scale", "行程比例", 0.10, 1.50, 0.01, "%.2f", _COL["ee"])
    _add_slider(gs_feel, 4, "max_joint_vel", "关节限速 rad/s", 0.50, 12.0, 0.05, "%.2f", _COL["ee"])
    _add_slider(gs_feel, 5, "workspace_radius", "工作球半径 m", 0.0, 0.70, 0.01, "%.2f", _COL["ee"])

    _add_slider(gs_urdf, 1, "ik_w_pos", "位置权重", 0.05, 3.0, 0.05, "%.2f", _COL["filt"])
    _add_slider(gs_urdf, 2, "ik_w_ori", "姿态权重", 0.0, 2.0, 0.05, "%.2f", _COL["filt"])
    _add_slider(gs_urdf, 3, "ik_max_iter", "最大迭代", 5.0, 50.0, 1.0, "%.0f", _COL["filt"])
    _add_slider(
        gs_urdf, 4, "ik_tol", "公差 log10", -10.0, -4.0, 0.5, "%.1f", _COL["filt"], "log10"
    )
    _add_slider(
        gs_urdf, 5, "ik_w_reg", "正则 log10", -6.0, -2.0, 0.5, "%.1f", _COL["filt"], "log10"
    )
    _add_slider(gs_urdf, 6, "ik_q4_max", "肘上限 q4", -1.00, 0.00, 0.01, "%.2f", _COL["filt"])
    _add_slider(gs_urdf, 7, "ik_w_limit", "限位软约束", 0.00, 0.50, 0.01, "%.2f", _COL["filt"])

    ax_act_h = fig.add_subplot(gs_act[0])
    ax_act_h.set_facecolor(_COL["bg"])
    ax_act_h.axis("off")
    ax_act_h.text(0.0, 0.72, "求解器", fontsize=11, fontweight="bold", color=_COL["ink"])
    ax_act_h.text(0.0, 0.08, "热切换会重定 VR 零点", fontsize=8, color=_COL["muted"])

    def _mk_btn(spec, label):
        ax = fig.add_subplot(spec)
        btn = Button(ax, label, color=_COL["card"], hovercolor="#E5E7EB")
        btn.label.set_fontsize(10)
        btn.label.set_color(_COL["ink"])
        for spine in ax.spines.values():
            spine.set_color(_COL["line"])
        return ax, btn

    ax_dh, btn_dh = _mk_btn(gs_act[1], "DH  闭式")
    ax_urdf, btn_urdf = _mk_btn(gs_act[2], "URDF  数值")
    ax_geo, btn_geo = _mk_btn(gs_act[3], "几何  臂角")
    ax_sync, btn_sync = _mk_btn(gs_act[4], "左右同步  开" if node.sync_both else "左右同步  关")
    ax_zero, btn_zero = _mk_btn(gs_act[5], "清零  Z")
    ax_save, btn_save = _mk_btn(gs_act[6], "保存 CSV  S")

    def _paint_btn(ax, btn, active: bool) -> None:
        # Button redraws from btn.color; ax.set_facecolor alone is overwritten.
        if active:
            btn.color = _COL["on"]
            btn.hovercolor = "#374151"
            ax.set_facecolor(_COL["on"])
            btn.label.set_color("#FFFFFF")
        else:
            btn.color = _COL["card"]
            btn.hovercolor = "#E5E7EB"
            ax.set_facecolor(_COL["card"])
            btn.label.set_color(_COL["ink"])

    def _paint_solver(mode: str) -> None:
        _paint_btn(ax_dh, btn_dh, mode == "dh")
        _paint_btn(ax_urdf, btn_urdf, mode == "urdf")
        _paint_btn(ax_geo, btn_geo, mode == "geo")
        fig.canvas.draw_idle()

    def _paint_sync() -> None:
        btn_sync.label.set_text("左右同步  开" if node.sync_both else "左右同步  关")
        _paint_btn(ax_sync, btn_sync, node.sync_both)
        fig.canvas.draw_idle()

    st0 = str(init.get("solver_type", "geometric")).lower()
    _paint_solver(
        "dh" if ("dh" in st0 or "analytic" in st0)
        else "urdf" if "urdf" in st0
        else "geo"
    )
    _paint_sync()

    def _on_dh(_event):
        node.queue_param("solver_type", "analytic_dh")
        _paint_solver("dh")

    def _on_urdf(_event):
        node.queue_param("solver_type", "urdf_numerical")
        _paint_solver("urdf")

    def _on_geo(_event):
        node.queue_param("solver_type", "geometric")
        _paint_solver("geo")

    def _on_sync(_event):
        node.sync_both = not node.sync_both
        _paint_sync()
        node.get_logger().info(f"sync both arms = {node.sync_both}")

    def _on_zero(_event):
        with node._lock:
            node._rezero = True
        node.get_logger().info("re-zero on next sample")

    def _on_save(_event):
        with node._lock:
            node._save_req = True

    btn_dh.on_clicked(_on_dh)
    btn_urdf.on_clicked(_on_urdf)
    btn_geo.on_clicked(_on_geo)
    btn_sync.on_clicked(_on_sync)
    btn_zero.on_clicked(_on_zero)
    btn_save.on_clicked(_on_save)

    def on_key(event):
        if event.key == "z":
            _on_zero(None)
        elif event.key == "s":
            _on_save(None)
        elif event.key in ("q", "escape"):
            plt.close(fig)

    fig.canvas.mpl_connect("key_press_event", on_key)

    def _set_metrics(m: dict | None) -> None:
        if m is None:
            return
        for key, txt in metric_vals.items():
            txt.set_text(m.get(key, "—"))
        status_txt.set_text(m.get("status", ""))

    def update(_):
        node.flush_params()
        snap = node.snapshot()
        if snap is None:
            _set_metrics(
                {
                    "rms": "—",
                    "p95": "—",
                    "ik": "—",
                    "jitter": "—",
                    "lag": "—",
                    "ori": "—",
                    "status": node._param_status or "等待数据",
                }
            )
            return []
        if snap["save"]:
            node.save_csv(snap)
        t = snap["t"]
        vr, filt, cmd = snap["vr"], snap["filt"], snap["cmd"]
        err = np.linalg.norm(vr - cmd, axis=1)
        for i in range(3):
            l0, l1, l2 = lines[i]
            l0.set_data(t, vr[:, i])
            l1.set_data(t, filt[:, i])
            l2.set_data(t, cmd[:, i])
            ymin = float(min(vr[:, i].min(), filt[:, i].min(), cmd[:, i].min()))
            ymax = float(max(vr[:, i].max(), filt[:, i].max(), cmd[:, i].max()))
            pad = max(2.0, 0.15 * (ymax - ymin + 1e-6))
            axes[i].set_xlim(-float(node.window_sec), 0.0)
            axes[i].set_ylim(ymin - pad, ymax + pad)
        lines[3][0].set_data(t, err)
        axes[3].set_xlim(-float(node.window_sec), 0.0)
        axes[3].set_ylim(0.0, max(2.0, float(err.max()) * 1.2))
        _set_metrics(node.compute_metrics(snap))
        return []

    _anim = FuncAnimation(fig, update, interval=50, blit=False, cache_frame_data=False)
    _keep = (sliders, btn_dh, btn_urdf, btn_sync, btn_zero, btn_save)
    plt.show()
    del _anim, _keep

def main(args=None) -> None:
    rclpy.init(args=args)
    node = TeleopTunePlot()
    spin = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin.start()
    try:
        _run_plot(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:  # noqa: BLE001
            pass


if __name__ == "__main__":
    main()
