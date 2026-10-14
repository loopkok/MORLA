"""线缆图像编解码（JPEG 上行传输优化）。

节点侧 RemoteBackend 把 RGB 编码成 JPEG 字节，serve 侧解码还原——上行载荷
1.38MB(2×480×480×3) → ~0.2MB。编码/解码两端共用本模块，保证一致。

JPEG 有损（默认 quality 92）：训练数据本身是 collect tap 的 JPEG-90 解码产物，
节点 RGB 是"JPEG-90 解码后 letterbox"的像素，再编码一次引入的二次伪影很小、
模型对其鲁棒（可调 quality 权衡带宽 vs 保真）。cv2 最快；缺失时回退 PIL。
"""

from __future__ import annotations

import numpy as np

JPEG_MAGIC = b"\xff\xd8"


def encode_jpeg(rgb: np.ndarray, quality: int = 92) -> bytes:
    """RGB (H,W,3) uint8 → JPEG 字节。"""
    rgb = np.asarray(rgb, dtype=np.uint8)
    if rgb.ndim != 3 or rgb.shape[2] != 3:
        raise ValueError(f"encode_jpeg expects HxWx3 RGB, got {rgb.shape}")
    try:
        import cv2

        ok, buf = cv2.imencode(
            ".jpg", cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR),
            [cv2.IMWRITE_JPEG_QUALITY, int(quality)],
        )
        if not ok:
            raise ValueError("cv2.imencode failed")
        return buf.tobytes()
    except ImportError:
        from PIL import Image
        import io

        b = io.BytesIO()
        Image.fromarray(rgb).save(b, format="JPEG", quality=int(quality))
        return b.getvalue()


def decode_jpeg(data: bytes) -> np.ndarray:
    """JPEG 字节 → RGB (H,W,3) uint8。"""
    try:
        import cv2

        img = cv2.imdecode(np.frombuffer(data, dtype=np.uint8), cv2.IMREAD_COLOR)
        if img is None:
            raise ValueError("cv2.imdecode failed")
        return cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    except ImportError:
        from PIL import Image
        import io

        return np.asarray(Image.open(io.BytesIO(data)).convert("RGB"))


def normalize_request_images(request: dict) -> dict:
    """serve 端归一化：JPEG 上行载荷 → RGB 数组，并移除 image_format 标志。

    节点 jpeg_transport 时 camera 槽位是 JPEG 字节 + ``image_format="jpeg"``；
    这里逐槽解码回 RGB，pop 掉标志（避免污染 openpi 的 AstralInputs）。非 jpeg
    载荷（现状 RGB 数组）原样透传。
    """
    if request.get("image_format") != "jpeg":
        return request
    for key, val in list(request.items()):
        if key.startswith("observation/camera/") and isinstance(val, (bytes, bytearray)):
            request[key] = decode_jpeg(bytes(val))
    request.pop("image_format", None)
    return request
