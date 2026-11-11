"""把 data/ 里的压缩证据渲染成自包含的 report.html（不需要 trace）。

用法:
    /usr/bin/python3 scripts/report.py

只读 data/<run_id>/{summary.json,snapshots.csv,rates_per_second.csv}。
页面是纯静态 HTML + 内联 SVG，没有任何外部数据或脚本。
"""
import collections
import csv
import json
import os

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DATA = os.path.join(HERE, 'data')
OUT = os.path.join(HERE, 'report.html')

RUNS = [
    ('20261111T052027Z_mmk2_quest_sim_7ef3737a672a4e589b26db511a4dc0dd',
     'MMK2 · 双手', 'side: both', True),
    ('20261111T053601Z_mmk2_quest_sim_8f7f6c0e1c994fdda6739f1752b0a62d',
     'MMK2 · 单手', 'side: right', False),
    ('20261106T090825Z_fr3_quest_real_27073bfb455d45aaa69c338c6520a879',
     'FR3 · 单手', 'side: right', False),
    ('20261106T091451Z_fr3_quest_real_bdb268bc79d44e8f8c413e840b86d036',
     'FR3 · 单手', 'side: right', False),
]


def load(run):
    with open(os.path.join(DATA, run, 'summary.json')) as handle:
        return json.load(handle)


def rows(run, name):
    with open(os.path.join(DATA, run, name)) as handle:
        return list(csv.DictReader(handle))


def produced_per_second(run):
    """每个样本把 (sequence 增量 / 2) 记到它到达的那一秒 → 头显交付的位姿样本/秒。"""
    buckets = {'left': collections.defaultdict(float), 'right': collections.defaultdict(float)}
    previous = {}
    for row in rows(run, 'snapshots.csv'):
        side = row['side']
        sequence = row.get('sample_sequence')
        if not sequence:
            continue
        t, s = int(row['frame_ns']), int(sequence)
        if side in previous:
            buckets[side][t // 1_000_000_000] += (s - previous[side][1]) / 2.0
        previous[side] = (t, s)
    return buckets


def num(value, digits=1, dash='—'):
    if value is None:
        return dash
    try:
        return ('%.' + str(digits) + 'f') % float(value)
    except (TypeError, ValueError):
        return dash


def svg_timeseries(left, right, produced_l, produced_r):
    """每秒：虚线 = 头显交付，实线 = 引擎拿到。"""
    seconds = sorted(set(left) | set(right) | set(produced_l) | set(produced_r))
    if not seconds:
        return ''
    start = seconds[0]
    xs = [s - start for s in seconds]
    width, height = 880, 282
    left_pad, right_pad, top_pad, bottom_pad = 44, 16, 34, 46
    plot_w = width - left_pad - right_pad
    plot_h = height - top_pad - bottom_pad
    x_max = max(xs) or 1
    y_max = 120.0

    def px(second):
        return left_pad + plot_w * second / x_max

    def py(value):
        return top_pad + plot_h * (1 - min(value, y_max) / y_max)

    def line(series, colour, width_px, dash):
        points = []
        for second in seconds:
            value = series.get(second)
            if value is None:
                continue
            points.append('%.1f,%.1f' % (px(second - start), py(value)))
        if not points:
            return ''
        extra = ' stroke-dasharray="%s"' % dash if dash else ''
        return ('<polyline points="%s" fill="none" stroke="%s" stroke-width="%s" '
                'stroke-linejoin="round" stroke-linecap="round"%s/>'
                % (' '.join(points), colour, width_px, extra))

    grid = []
    for value in (0, 30, 60, 90, 120):
        y = py(value)
        grid.append('<line x1="%.1f" y1="%.1f" x2="%.1f" y2="%.1f" stroke="var(--rule)" stroke-width="1"/>'
                    % (left_pad, y, width - right_pad, y))
        grid.append('<text x="%.1f" y="%.1f" text-anchor="end" font-size="11" fill="var(--muted)">%d</text>'
                    % (left_pad - 8, y + 4, value))
    ticks = []
    step = 10 if x_max <= 120 else 20
    for second in range(0, int(x_max) + 1, step):
        ticks.append('<text x="%.1f" y="%.1f" text-anchor="middle" font-size="11" fill="var(--muted)">%d</text>'
                     % (px(second), height - bottom_pad + 18, second))
    return (
        '<svg viewBox="0 0 %d %d" role="img" aria-label="每秒头显交付与引擎摄取的位姿样本数">'
        '<text x="%.1f" y="%.1f" font-size="11" fill="var(--muted)">样本 / 秒</text>'
        '%s%s%s%s%s%s'
        '<text x="%.1f" y="%d" text-anchor="middle" font-size="11" fill="var(--muted)">运行内的秒</text>'
        '</svg>'
        % (width, height,
           left_pad - 36, 14,
           ''.join(grid),
           line(produced_l, 'var(--muted)', 1.4, '5 4'),
           line(produced_r, 'var(--muted)', 1.4, '5 4'),
           line(left, 'var(--accent)', 2.2, None),
           line(right, 'var(--warm)', 2.2, None),
           ''.join(ticks),
           width / 2.0, height - 6)
    )


def svg_bars(items, limit, caption):
    """items: [(label, value, colour)]；limit 是虚线参考值。"""
    width = 880
    row_h = 34
    height = len(items) * row_h + 34
    left_pad, right_pad = 150, 70
    plot_w = width - left_pad - right_pad
    scale = max([value for _, value, _ in items] + [limit]) * 1.08
    out = []
    for index, (label, value, colour) in enumerate(items):
        y = 12 + index * row_h
        bar = plot_w * value / scale
        out.append('<text x="%d" y="%.1f" text-anchor="end" font-size="12" fill="var(--ink)">%s</text>'
                   % (left_pad - 10, y + 13, label))
        out.append('<rect x="%d" y="%.1f" width="%.1f" height="18" rx="3" fill="%s"/>'
                   % (left_pad, y, bar, colour))
        out.append('<text x="%.1f" y="%.1f" font-size="12" fill="var(--muted)" '
                   'font-variant-numeric="tabular-nums">%s</text>'
                   % (left_pad + bar + 8, y + 13, caption(value)))
    x = left_pad + plot_w * limit / scale
    out.append('<line x1="%.1f" y1="4" x2="%.1f" y2="%d" stroke="var(--muted)" '
               'stroke-width="1" stroke-dasharray="4 4"/>' % (x, x, height - 18))
    out.append('<text x="%.1f" y="%d" text-anchor="middle" font-size="11" fill="var(--muted)">'
               '头显交付 90</text>' % (x, height - 4))
    return '<svg viewBox="0 0 %d %d" role="img" aria-label="指令率对比">%s</svg>' % (
        width, height, ''.join(out))


def table(headers, body, classes=''):
    head = ''.join('<th>%s</th>' % cell for cell in headers)
    lines = ''.join('<tr>%s</tr>' % ''.join(cell for cell in row) for row in body)
    return ('<div class="scroll"><table class="%s"><thead><tr>%s</tr></thead><tbody>%s</tbody></table></div>'
            % (classes, head, lines))


def cell(text, mono=True, strong=False):
    cls = 'num' if mono else ''
    if strong:
        cls = (cls + ' strong').strip()
    return '<td class="%s">%s</td>' % (cls, text)


def main():
    summaries = {run: load(run) for run, _, _, _ in RUNS}
    main_run = RUNS[0][0]
    fr3_run = RUNS[2][0]

    per_second = {int(row['second']): row for row in rows(main_run, 'rates_per_second.csv')}
    base = min(per_second)
    left_series = {second - base: int(row['left_samples']) for second, row in per_second.items()}
    right_series = {second - base: int(row['right_samples']) for second, row in per_second.items()}
    produced = produced_per_second(main_run)
    produced_l = {second - base: value for second, value in produced['left'].items()}
    produced_r = {second - base: value for second, value in produced['right'].items()}

    # 三段对照
    def phase(lo, hi):
        samples_l = [value for second, value in left_series.items() if lo <= second <= hi]
        samples_r = [value for second, value in right_series.items() if lo <= second <= hi]
        commands_l = [int(per_second[base + second]['left_commands']) for second in range(lo, hi + 1)]
        return (sum(samples_l) / len(samples_l) if samples_l else 0,
                sum(samples_r) / len(samples_r) if samples_r else 0,
                sum(commands_l) / len(commands_l) if commands_l else 0)

    idle_l, idle_r, idle_c = phase(1, 4)
    work_start, work_end = 7, 88
    work_l, work_r, work_c = phase(work_start, work_end)
    tail_l, tail_r, tail_c = phase(90, 91)

    main_summary = summaries[main_run]
    fr3_summary = summaries[fr3_run]
    left_port = main_summary['port']['left_pose']
    right_port = main_summary['port']['right_pose']
    fr3_port = fr3_summary['port']['right_pose']

    rate_rows = []
    for run, label, side, _ in RUNS:
        summary = summaries[run]
        for port in ('left_pose', 'right_pose'):
            data = summary['port'].get(port)
            if not data or 'pose_hz' not in data:
                continue
            command = num(data.get('rate_hz')) + ' Hz' if data.get('rate_hz') else '无指令'
            rate_rows.append([
                cell(label + (' · 左手' if port.startswith('left') else ' · 右手'), mono=False),
                cell(num(data['pose_hz']) + ' Hz'),
                cell(num(data['sequence_per_s']), strong=False),
                cell(num(data['seq_per_pose'], 2), strong=data['seq_per_pose'] > 3),
                cell(command),
            ])

    control_rows = []
    for label, data in (('MMK2 左', left_port), ('MMK2 右', right_port), ('FR3 右', fr3_port)):
        control_rows.append([
            cell(label, mono=False),
            cell(num(data.get('rate_hz')) + ' Hz'),
            cell('%s / %s' % (num(data.get('gap_p50_ms')), num(data.get('gap_p95_ms')))),
            cell('%s / %s' % (num(data.get('joint_jump_p50'), 4), num(data.get('joint_jump_p95'), 4))),
            cell('%s / %s' % (num(data.get('solve_p50_ms')), num(data.get('solve_p95_ms')))),
            cell('%s / %s' % (num(data.get('candidate_age_p50_ms')), num(data.get('candidate_age_p95_ms')))),
            cell('%s / %s' % (num(data.get('sigma_min_p50'), 3), num(data.get('damping_p50'), 3))),
        ])

    phase_rows = [
        [cell('0–4 s　未按 deadman', mono=False), cell('%s / %s' % (num(idle_l, 0), num(idle_r, 0))),
         cell(num(idle_c, 0)), cell('引擎空转，能全速收 261 帧/秒', mono=False)],
        [cell('%d–%d s　遥操中' % (work_start, work_end), mono=False),
         cell('%s / %s' % (num(work_l, 0), num(work_r, 0))), cell(num(work_c, 0)),
         cell('每拍同步解一次 IK，摄取与指令 1:1', mono=False)],
        [cell('90–91 s　松手', mono=False), cell('%s / %s' % (num(tail_l, 0), num(tail_r, 0))),
         cell(num(tail_c, 0)), cell('最后 1 秒指令为 0，摄取回升到 50', mono=False)],
    ]

    bars = svg_bars([
        ('MMK2 左手', left_port['rate_hz'], 'var(--accent)'),
        ('MMK2 右手', right_port['rate_hz'], 'var(--accent)'),
        ('FR3（第一次）', fr3_port['rate_hz'], 'var(--warm)'),
        ('FR3（第二次）', summaries[RUNS[3][0]]['port']['right_pose']['rate_hz'], 'var(--warm)'),
    ], 90.0, lambda value: '%.1f Hz' % value)

    style = STYLE
    body = PAGE.format(
        style=style,
        chart=svg_timeseries(left_series, right_series, produced_l, produced_r),
        bars=bars,
        rate_table=table(['运行', '引擎拿到', '接收端 sequence', 'seq/pose', '下发指令'], rate_rows),
        control_table=table(['', '指令率', '间隔 p50 / p95 ms', '每拍关节跳变 p50 / p95', '解算 ms',
                             'candidate age ms', 'σ_min / 阻尼 p50'], control_rows),
        phase_table=table(['时段', '引擎拿到 左 / 右（样本/秒）', '指令/秒', '含义'], phase_rows),
        delivered=num(main_summary['port']['left_pose']['sequence_per_s'] / 2, 0),
        consumed_l=num(left_port['pose_hz']),
        consumed_r=num(right_port['pose_hz']),
        lost_l=num(100 * (1 - 2 / left_port['seq_per_pose']), 0),
        lost_r=num(100 * (1 - 2 / right_port['seq_per_pose']), 0),
        solve=num(left_port['solve_p50_ms']),
        solve_fr3=num(fr3_port['solve_p50_ms']),
        cmd_l=num(left_port['rate_hz']),
        cmd_r=num(right_port['rate_hz']),
        cmd_fr3=num(fr3_port['rate_hz']),
        step_mmk=num(left_port['joint_jump_p50'], 4),
        step_fr3=num(fr3_port['joint_jump_p50'], 4),
        frames_work=num(main_summary['frame_rate_hz']),
    )
    with open(OUT, 'w') as handle:
        handle.write(body)
    print('%s -> %s' % (DATA, OUT))


STYLE = """
:root{
  --bg:#f4f6f7; --panel:#ffffff; --ink:#12171c; --muted:#57636e; --rule:#d9e0e5;
  --accent:#0b6f78; --warm:#b05730; --chip:#e8eff0; --shadow:0 1px 2px rgba(18,23,28,.06);
  --mono:"IBM Plex Mono",ui-monospace,SFMono-Regular,Menlo,Consolas,monospace;
  --sans:system-ui,-apple-system,"Segoe UI","PingFang SC","Hiragino Sans GB","Noto Sans CJK SC","Microsoft YaHei",sans-serif;
  color-scheme:light;
}
@media (prefers-color-scheme: dark){
  :root:not([data-theme="light"]){
    --bg:#0d1216; --panel:#141b21; --ink:#e8edf1; --muted:#96a3ae; --rule:#242e37;
    --accent:#56c8d2; --warm:#e08a5c; --chip:#1a232b; --shadow:none;
    color-scheme:dark;
  }
}
:root[data-theme="dark"]{
  --bg:#0d1216; --panel:#141b21; --ink:#e8edf1; --muted:#96a3ae; --rule:#242e37;
  --accent:#56c8d2; --warm:#e08a5c; --chip:#1a232b; --shadow:none;
  color-scheme:dark;
}
*{box-sizing:border-box}
body{margin:0;background:var(--bg);color:var(--ink);font-family:var(--sans);
  font-size:16px;line-height:1.65;-webkit-font-smoothing:antialiased}
.wrap{max-width:960px;margin:0 auto;
  padding-left:max(16px,env(safe-area-inset-left,0px));
  padding-right:max(16px,env(safe-area-inset-right,0px));
  padding-block:48px 80px}
.eyebrow{font-family:var(--mono);font-size:11px;letter-spacing:.14em;text-transform:uppercase;
  color:var(--muted);margin:0 0 14px}
h1{font-size:clamp(28px,5vw,42px);line-height:1.15;letter-spacing:-.015em;margin:0 0 16px;
  text-wrap:balance;font-weight:650}
.lead{font-size:17px;max-width:62ch;color:var(--ink);margin:0 0 8px}
.sub{font-size:15px;max-width:62ch;color:var(--muted);margin:0}
.tiles{display:grid;grid-template-columns:repeat(4,1fr);gap:12px;margin:32px 0 8px}
.tile{background:var(--panel);border:1px solid var(--rule);border-radius:8px;padding:14px 16px;box-shadow:var(--shadow)}
.tile .v{font-family:var(--mono);font-size:clamp(19px,2.5vw,24px);font-weight:600;
  font-variant-numeric:tabular-nums;letter-spacing:-.01em}
.tile .k{font-size:12px;color:var(--muted);margin-top:4px;line-height:1.45}
section{margin-top:44px}
h2{font-size:13px;font-family:var(--mono);letter-spacing:.12em;text-transform:uppercase;
  color:var(--muted);font-weight:600;margin:0 0 6px;padding-top:18px;border-top:1px solid var(--rule)}
h3{font-size:19px;margin:0 0 12px;font-weight:620;letter-spacing:-.01em}
p{margin:0 0 14px;max-width:68ch}
.fig{background:var(--panel);border:1px solid var(--rule);border-radius:8px;padding:18px 14px 12px;
  margin:18px 0 8px;box-shadow:var(--shadow)}
.fig svg{display:block;width:100%;height:auto}
.legend{display:flex;flex-wrap:wrap;gap:16px;font-size:12px;color:var(--muted);
  padding:0 6px 2px;font-family:var(--mono)}
.legend span{display:inline-flex;align-items:center;gap:7px}
.swatch{width:18px;height:0;border-top-width:2px;border-top-style:solid;display:inline-block}
.cap{font-size:13px;color:var(--muted);margin:8px 2px 0;max-width:70ch}
.scroll{overflow-x:auto;margin:16px 0 8px;border:1px solid var(--rule);border-radius:8px;background:var(--panel)}
table{border-collapse:collapse;width:100%;min-width:620px;font-size:14px}
th{text-align:left;font-weight:600;font-size:12px;letter-spacing:.04em;color:var(--muted);
  padding:10px 12px;border-bottom:1px solid var(--rule);white-space:nowrap;background:var(--chip)}
td{padding:9px 12px;border-bottom:1px solid var(--rule);vertical-align:top}
tr:last-child td{border-bottom:0}
td.num,.strong{font-family:var(--mono);font-variant-numeric:tabular-nums;white-space:nowrap;font-size:13px}
td.strong{font-weight:600;color:var(--warm)}
.notes{display:grid;grid-template-columns:1fr 1fr;gap:10px 28px;margin-top:6px}
.notes h3{font-size:15px;margin:0 0 6px}
.notes ul{margin:0;padding-left:18px;font-size:14px;color:var(--ink)}
.notes li{margin-bottom:6px}
code{font-family:var(--mono);font-size:.9em;background:var(--chip);padding:1px 5px;border-radius:4px}
pre{background:var(--panel);border:1px solid var(--rule);border-radius:8px;padding:14px 16px;
  overflow-x:auto;font-family:var(--mono);font-size:12.5px;line-height:1.7;margin:14px 0}
pre code{background:none;padding:0}
footer{margin-top:52px;padding-top:18px;border-top:1px solid var(--rule);color:var(--muted);font-size:13px}
@media (max-width:760px){
  .tiles{grid-template-columns:1fr 1fr}
  .notes{grid-template-columns:1fr}
  .wrap{padding-block:32px 56px}
}
@media (max-width:420px){ .tiles{grid-template-columns:1fr} }
@media (prefers-reduced-motion:reduce){ *{animation:none!important;transition:none!important} }
"""

PAGE = """<meta charset="utf-8">
<meta name="claude:viewport-fit-sides" content="cover">
<title>MMK2 遥操节奏核查</title>
<style>{style}</style>
<div class="wrap">
  <p class="eyebrow">MORLA · 2026-11-11 · 证据包</p>
  <h1>MMK2 为什么“一卡一卡”</h1>
  <p class="lead">头显一直在按每只手约 90 个位姿样本/秒交付。MORLA 的引擎每收到一个样本就同步解一次 IK，
  而 MMK2 解一次要 {solve} ms——于是每秒最多只能吃下约 78 个，其余的在到达引擎之前被丢掉。
  FR3 解一次只要 {solve_fr3} ms，所以同样的输入节奏它一条不丢。</p>
  <p class="sub">下面每个数字都能在 <code>data/</code> 里用 Excel 或 pandas 复算；原始 trace 不在本包里。</p>

  <div class="tiles">
    <div class="tile"><div class="v">{delivered} /s</div><div class="k">每只手交付的位姿样本<br>四轮运行完全相同</div></div>
    <div class="tile"><div class="v">{consumed_l} · {consumed_r}</div><div class="k">引擎实际吃到的样本/秒<br>报卡的那一轮（左 · 右）</div></div>
    <div class="tile"><div class="v">{lost_l}% · {lost_r}%</div><div class="k">在到达引擎前丢掉的份额<br>由 seq/pose 反推</div></div>
    <div class="tile"><div class="v">{solve} ms</div><div class="k">MMK2 单次 IK（FR3 {solve_fr3} ms）<br>决定了摄取上限 1/{solve} ms</div></div>
  </div>

  <section>
    <h2>证据一 · 节奏</h2>
    <h3>交付没有变，摄取塌了</h3>
    <p>虚线是接收端每秒交给下游的位姿样本数（由 <code>sequence</code> 增量除以 2 得到：一个位姿样本会带来
    一次 pose 更新和一次 landmarks 更新）。实线是引擎真正处理的样本数。三条线在开头 5 秒重合在 90；
    一按 deadman 开始解算，实线就掉到 30–40，虚线纹丝不动。</p>
    <div class="fig">
      <div class="legend">
        <span><i class="swatch" style="border-top-style:dashed;border-color:var(--muted)"></i>头显交付（左/右）</span>
        <span><i class="swatch" style="border-color:var(--accent)"></i>引擎拿到 · 左</span>
        <span><i class="swatch" style="border-color:var(--warm)"></i>引擎拿到 · 右</span>
      </div>
      {chart}
    </div>
    <p class="cap">同一轮运行内的三段对照，说明掉下来的是处理能力而不是输入：</p>
    {phase_table}
    <p class="cap">四轮运行横向对照。seq/pose 等于 2 表示接收端每发布一个位姿引擎就拿到一个；4.56 表示
    平均每 4.56 次接收端更新才有一个位姿到达引擎。</p>
    {rate_table}
  </section>

  <section>
    <h2>证据二 · 手感</h2>
    <h3>指令跟着摄取走，台阶因此变粗</h3>
    <p>引擎只在收到新样本时下发一次指令，所以指令率被摄取率封顶。平均速度两边接近
    （0.53 对 0.63 rad/s），但 MMK2 每拍走 {step_mmk} rad、FR3 每拍走 {step_fr3} rad，
    同样的动作被切成 5–6 倍粗的台阶——这就是“一格一格”的来源。</p>
    {control_table}
    <p class="cap">“每拍关节跳变”是相邻两条指令之间最大的关节差值，两台机器定义相同，可以直接比。</p>
    <div class="fig">
      {bars}
    </div>
    <p class="cap">虚线是头显的交付节奏。MMK2 的指令率不到它的一半，FR3 正好贴着它。</p>
  </section>

  <section>
    <h2>证据三 · 为什么是 MMK2</h2>
    <h3>绑定参数几乎一样，差在解算器</h3>
    <p>两边的 <code>position_scale</code>、<code>smoothing_alpha</code>、Quest 实例的
    <code>rate_hz</code> 都相同；差别是 IK：FR3 用内置解析解，MMK2 用
    <code>morla.mmk2.cartesian_delta</code> 的数值解，慢 {solve} ms。取值的出处见
    <code>results/config.txt</code>。</p>
    <p><code>smoothing_alpha = 0.6</code> 的时间常数是 2.5 个控制周期：90 Hz 下 28 ms，
    35 Hz 下 71 ms。同样的数字在 MMK2 上多滞后约 40 ms，这是次生效应，不是主因。</p>
    <p>另外，MMK2 的初值位形 <code>[0,-0.6,1,±1.6,∓0.6,0]</code> 落在奇异附近
    （σ_min 0.031，阻尼 0.025），部分拍会被回溯砍步；这是叠加因素。</p>
  </section>

  <section>
    <h2>边界</h2>
    <div class="notes">
      <div>
        <h3>已经排除</h3>
        <ul>
          <li>头显降频：接收端 sequence 四轮都是 180/s，与是否遥操无关。</li>
          <li>引擎门控：<code>control_gate</code> 只解释指令比样本少的部分，不解释样本本身的缺口。</li>
          <li>绑定参数更宽容：两边 position_scale / smoothing 相同。</li>
        </ul>
      </div>
      <div>
        <h3>还看不出来</h3>
        <ul>
          <li>sequence 是接收器自增的计数，不是头显时钟：它能证明“APK 交给接收器多少”，不能证明 APK 内部有没有丢采样。</li>
          <li>DDS 的 depth=1 不报丢弃数，具体丢了多少只能由 seq/pose 反推。</li>
          <li>本轮只覆盖仿真与上述几条运行，MMK2 真机不在其中。</li>
        </ul>
      </div>
    </div>
  </section>

  <section>
    <h2>复算</h2>
    <p>本页只读 <code>data/</code>，不需要 trace：</p>
    <pre><code>python3 scripts/report.py          # 重新生成本页
head data/&lt;run&gt;/rates_per_second.csv   # 逐秒的样本/指令数
head data/&lt;run&gt;/summary.json           # 汇总百分位</code></pre>
    <p>要从原始 trace 重新压缩数据，见 README 的“复现”一节。</p>
  </section>

  <footer>MORLA 遥操节奏核查 · 2026-11-11 · 证据文件：README.md、data/、results/</footer>
</div>
"""

if __name__ == '__main__':
    main()
