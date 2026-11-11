"""MMK2 求解器的成本画像：单次耗时、线程扩展、CPU 占用。

用法:
    OPENBLAS_NUM_THREADS=1 /usr/bin/python3 scripts/ik_bench.py   # 对照组
    /usr/bin/python3 scripts/ik_bench.py                           # 默认 BLAS 线程

需要仓库可导入（PYTHONPATH 见文末）。三组测量:
  solve      热身后单次 solve 的 p50（同一目标、同一初值）
  threads    1/2/4 个求解线程各自空转解算，看总吞吐是否随线程数增长
  cpu        同样负载下的 进程 CPU / 墙钟；>1 说明有核在空转

为什么看这三项：MMK2 的求解是纯 Python + numpy + scipy，6×6 的小矩阵调用
轮不到 BLAS 并行，但 OpenBLAS 仍会拉起线程自旋，白白占核。
"""
import os
import sys
import threading
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = HERE                      # scripts/ → 本目录 → investigations → docs → 仓库根
for _ in range(4):
    ROOT = os.path.dirname(ROOT)

sys.path.insert(0, os.path.join(ROOT, 'src/common/morla_core'))
sys.path.insert(0, os.path.join(ROOT, 'src/common/morla_teleop'))
sys.path.insert(0, os.path.join(ROOT, 'src/robots/airbot/morla_mmk2_teleop'))

from morla_mmk2_teleop.urdf_ik import ArmKinematics  # noqa: E402

START = np.array([0., -.6, 1., 1.6, -.6, 0.])


def make(side='left'):
    return ArmKinematics({'plugin_id': 'morla.airbot.mmk2.%s_arm' % side,
                          'parameters': {}}, max_step_rad=.04)


def cpu_seconds():
    t = os.times()
    return t.user + t.system


def single():
    solver = make()
    target = solver.fk(START)
    for _ in range(5):
        solver.solve(target, seed=START.copy())
    times = []
    for _ in range(20):
        began = time.perf_counter()
        solver.solve(target, seed=START.copy())
        times.append(time.perf_counter() - began)
    print('  solve（热身后，固定目标）p50 %.2f ms  p95 %.2f ms' % (
        1e3 * np.median(times), 1e3 * np.percentile(times, 95)))


def threads(count, seconds=3.0):
    solvers = [make('left' if i % 2 == 0 else 'right') for i in range(count)]
    target = solvers[0].fk(START)
    stop = threading.Event()
    done = [0] * count

    def work(i, solver):
        while not stop.is_set():
            solver.solve(target, seed=None)
            done[i] += 1

    workers = [threading.Thread(target=work, args=(i, s), daemon=True)
               for i, s in enumerate(solvers)]
    began, cpu0 = time.perf_counter(), cpu_seconds()
    for worker in workers:
        worker.start()
    time.sleep(seconds)
    stop.set()
    for worker in workers:
        worker.join()
    wall, used = time.perf_counter() - began, cpu_seconds() - cpu0
    print('  %d 线程：总 %6.1f 次/s   单线程 %s   CPU/墙钟 %.2f 核' % (
        count, sum(done) / wall, ' '.join('%.0f' % (n / wall) for n in done), used / wall))


if __name__ == '__main__':
    print('OPENBLAS_NUM_THREADS=%s  OMP_NUM_THREADS=%s' % (
        os.environ.get('OPENBLAS_NUM_THREADS', '未设置'), os.environ.get('OMP_NUM_THREADS', '未设置')))
    single()
    for count in (1, 2, 4):
        threads(count)
