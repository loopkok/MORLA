"""Small PCM frame contract used by the Unity client and the host receiver."""
import base64
import binascii


def decode_pcm(payload):
    fields = payload.split(":", 2)
    if len(fields) != 3 or fields[0] != "VoicePCM" or fields[1] != "16000":
        raise ValueError("expected VoicePCM:16000:<base64 mono s16le>")
    try:
        pcm = base64.b64decode(fields[2], validate=True)
    except (ValueError, binascii.Error) as exc:
        raise ValueError("invalid audio base64") from exc
    if len(pcm) != 1280:  # 640 samples = 40 ms; fixed size bounds decoding work.
        raise ValueError("expected exactly 640 signed 16-bit mono samples")
    return pcm
