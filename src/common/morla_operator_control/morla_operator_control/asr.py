"""Optional offline recognizer; no model download or microphone access on import."""
import json
import os
from pathlib import Path
import sys


def load_optional_dependencies():
    path = os.environ.get("MORLA_OPERATOR_SITE", "~/.local/share/morla/operator-python")
    if Path(path).expanduser().is_dir():
        sys.path.insert(0, str(Path(path).expanduser()))


class OfflineRecognizer:
    def __init__(self, model_path, sample_rate=16000):
        load_optional_dependencies()
        from vosk import Model, KaldiRecognizer
        model_path = Path(model_path).expanduser()
        if not (model_path / "am").is_dir():
            raise ValueError(f"Vosk model not found: {model_path}; run scripts/setup_operator_audio.sh")
        self.model = Model(str(model_path))
        self.recognizer = KaldiRecognizer(self.model, sample_rate)
        self.recognizer.SetWords(True)

    def reset(self):
        self.recognizer.Reset()

    def feed(self, pcm):
        if not self.recognizer.AcceptWaveform(pcm):
            return None
        result = json.loads(self.recognizer.Result())
        words = result.get("result", [])
        confidence = sum(float(word.get("conf", 0.0)) for word in words) / len(words) if words else 0.0
        return result.get("text", ""), confidence
