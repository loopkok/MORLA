#!/usr/bin/env bash
# Install optional ASR libraries/model into a MORLA-only directory. Opens no devices.
set -euo pipefail
OPERATOR_ROOT="${MORLA_OPERATOR_ROOT:-$HOME/.local/share/morla}"
mkdir -p "$OPERATOR_ROOT/operator-python" "$OPERATOR_ROOT/models"
python3 -m pip install --upgrade --target "$OPERATOR_ROOT/operator-python" 'vosk>=0.3.45,<0.4' 'sounddevice>=0.4.6,<0.6'
python3 - "$OPERATOR_ROOT/models" <<'PY'
from pathlib import Path
import hashlib, os, shutil, sys, tempfile, urllib.request, zipfile
root = Path(sys.argv[1])
name = 'vosk-model-small-cn-0.22'
if (root / name / 'am').is_dir():
    print('Model already installed:', root / name)
    raise SystemExit(0)
with tempfile.TemporaryDirectory(dir=root) as work:
    archive = Path(work) / 'model.zip'
    supplied = os.environ.get('MORLA_VOSK_ARCHIVE')
    if supplied:
        print('Using supplied model archive:', supplied)
        shutil.copyfile(supplied, archive)
    else:
        url = 'https://alphacephei.com/vosk/models/' + name + '.zip'
        print('Downloading model:', url)
        with urllib.request.urlopen(url, timeout=30) as response:
            total = int(response.headers.get('Content-Length') or 0)
            done = 0
            with archive.open('wb') as target:
                while True:
                    chunk = response.read(1 << 16)
                    if not chunk:
                        break
                    target.write(chunk)
                    done += len(chunk)
                    if total:
                        print(f'\r  {done / 1048576:.1f}/{total / 1048576:.1f} MB ({done * 100 // total}%)', end='', flush=True)
                    else:
                        print(f'\r  {done / 1048576:.1f} MB', end='', flush=True)
                print()
    print('Verifying checksum...')
    if hashlib.sha256(archive.read_bytes()).hexdigest() != '3af8b0e7e0f835ae9d414ce5df580237a3cfb08d586c9fbbb0f7ff29ad5b14ba':
        raise RuntimeError('Unexpected Vosk model checksum; archive not installed')
    print('Extracting model...')
    with zipfile.ZipFile(archive) as package:
        for entry in package.infolist():
            dest = (Path(work) / entry.filename).resolve()
            if Path(work).resolve() not in dest.parents:
                raise RuntimeError('Unsafe model archive member')
        package.extractall(work)
    shutil.move(str(Path(work) / name), str(root / name))
print('Installed Chinese offline model:', root / name)
PY
printf '\nAudio dependencies installed. Host audio also needs libportaudio2; pedal input needs python3-evdev.\n'
printf 'No microphone was opened; no robot or recording command was sent.\n'
