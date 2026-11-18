"""Measure what the MMK2 sim camera render thread costs per second.

Reproduces the numbers in ``../README.md``: per-camera MuJoCo offscreen cost
(RGB + depth + JPEG q90), the full 4-camera render set, and how much a pure
Python thread loses while that render set runs in another thread (the render
thread shares the Web process with the teleop ingest thread).

Run from the MORLA checkout root:

    PYTHONPATH=src/common/morla_core:src/robots/airbot/mmk2_mujoco_sim:src/common/morla_runtime \
        /usr/bin/python3 docs/investigations/camera_preview_2026-11-18/scripts/bench_camera_render.py

``--cams`` picks which configured cameras to draw from; ``--interference 0``
skips the Python-throughput test. Nothing is written to disk.
"""
import argparse
import io
import json
import statistics
import threading
import time


def median_ms(values):
    return round(statistics.median(values) * 1e3, 3)


def python_units(seconds):
    """A pure-Python work unit plus its own CPU/wall fraction.

    The unit is interpreter-bound, with no I/O and no GIL release. The CPU
    fraction (``time.thread_time`` over wall time) says how much of the sample
    the thread actually got the interpreter: a value near 1.0 means nobody was
    holding the GIL against it.
    """
    count = 0
    began = time.perf_counter()
    cpu_began = time.thread_time()
    while time.perf_counter() - began < seconds:
        count += 1
        sum(range(20))
    wall = time.perf_counter() - began
    return count / wall, (time.thread_time() - cpu_began) / wall


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--cams', default='head,left_wrist,right_wrist,scene')
    parser.add_argument('--width', type=int, default=320)
    parser.add_argument('--height', type=int, default=240)
    parser.add_argument('--quality', type=int, default=90)
    parser.add_argument('--fps', type=float, default=15.0, help='render rate used for the per-second estimate')
    parser.add_argument('--repeat', type=int, default=15)
    parser.add_argument('--interference', type=float, default=2.0,
                        help='seconds per Python-throughput sample; 0 skips the test')
    args = parser.parse_args()

    from mmk2_mujoco_sim.scene import model_xml
    import mujoco
    import numpy as np
    from PIL import Image

    model = mujoco.MjModel.from_xml_string(model_xml())
    data = mujoco.MjData(model)
    names = [name.strip() for name in args.cams.split(',') if name.strip()]
    cameras = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, name) for name in names]
    if any(index < 0 for index in cameras):
        raise SystemExit('unknown camera in --cams: %s' % args.cams)
    mujoco.mj_forward(model, data)
    renderer = mujoco.Renderer(model, height=args.height, width=args.width)

    step, forward, rgb, depth, encode = [], [], [], [], []
    jpeg_bytes = depth_bytes = 0
    # Warm the GL context before timing.
    for camera in cameras:
        renderer.disable_depth_rendering()
        renderer.update_scene(data, camera=camera)
        renderer.render()
        renderer.enable_depth_rendering()
        renderer.update_scene(data, camera=camera)
        renderer.render()
    for _ in range(args.repeat):
        began = time.perf_counter()
        mujoco.mj_step(model, data)
        stepped = time.perf_counter()
        mujoco.mj_forward(model, data)
        forwarded = time.perf_counter()
        pixels = []
        for camera in cameras:
            renderer.disable_depth_rendering()
            renderer.update_scene(data, camera=camera)
            pixels.append(renderer.render())
        rendered = time.perf_counter()
        maps = []
        for camera in cameras:
            renderer.enable_depth_rendering()
            renderer.update_scene(data, camera=camera)
            maps.append(renderer.render())
        mapped = time.perf_counter()
        for image, depth_map in zip(pixels, maps):
            output = io.BytesIO()
            Image.fromarray(image).save(output, format='JPEG', quality=args.quality)
            depth_bytes = len(np.asarray(depth_map, dtype='<f4').tobytes())
            jpeg_bytes = len(output.getvalue())
        encoded = time.perf_counter()
        count = len(cameras)
        step.append(stepped - began)
        forward.append(forwarded - stepped)
        rgb.append((rendered - forwarded) / count)
        depth.append((mapped - rendered) / count)
        encode.append((encoded - mapped) / count)

    per_set_ms = (statistics.median(forward) + len(cameras) *
                  (statistics.median(rgb) + statistics.median(depth) + statistics.median(encode))) * 1e3
    result = {
        'cameras': names,
        'size': [args.width, args.height],
        'jpeg_quality': args.quality,
        'jpeg_bytes': jpeg_bytes,
        'depth_bytes': depth_bytes,
        'mj_step_ms': median_ms(step),
        'mj_forward_ms': median_ms(forward),
        'rgb_render_per_camera_ms': median_ms(rgb),
        'depth_render_per_camera_ms': median_ms(depth),
        'jpeg_and_depth_copy_per_camera_ms': median_ms(encode),
        'full_render_set_ms': round(per_set_ms, 2),
        'estimated_core_fraction_at_fps': round(per_set_ms / 1e3 * args.fps, 3),
    }

    if args.interference:
        stop = threading.Event()

        def render_loop():
            local = mujoco.MjData(model)
            period = 1.0 / args.fps
            while not stop.is_set():
                mujoco.mj_forward(model, local)
                for camera in cameras:
                    renderer.disable_depth_rendering()
                    renderer.update_scene(local, camera=camera)
                    image = renderer.render()
                    output = io.BytesIO()
                    Image.fromarray(image).save(output, format='JPEG', quality=args.quality)
                    renderer.enable_depth_rendering()
                    renderer.update_scene(local, camera=camera)
                    np.asarray(renderer.render(), dtype='<f4').tobytes()
                time.sleep(period)

        python_units(.5)  # warmup, discarded
        baseline = [python_units(args.interference), python_units(args.interference)]
        thread = threading.Thread(target=render_loop)
        thread.start()
        time.sleep(.3)
        under_load = [python_units(args.interference), python_units(args.interference)]
        stop.set()
        thread.join()
        result['python_units_per_s_baseline'] = [round(v[0]) for v in baseline]
        result['python_units_per_s_with_render'] = [round(v[0]) for v in under_load]
        result['python_cpu_fraction_baseline'] = [round(v[1], 3) for v in baseline]
        result['python_cpu_fraction_with_render'] = [round(v[1], 3) for v in under_load]
        result['python_throughput_ratio_with_render'] = round(
            statistics.median(v[0] for v in under_load) / statistics.median(v[0] for v in baseline), 3)

    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
