#!/usr/bin/env bash
# encode.sh — PNG frames -> graded web mp4 + poster + blackframe audit.
# Run via BASH, not PowerShell 5.1 (PS wraps ffmpeg stderr into
# NativeCommandError and aborts the script).
#
#   ./encode.sh <frames_dir> <out_base> [fps] [poster_frame] [vertical]
#   ./encode.sh frames/my_shot out/my_shot 60 300
#   ./encode.sh frames/short1 out/short1 60 120 vertical   # 9:16 shorts
#
# Output: <out_base>_raw.mp4 (1080p master), <out_base>.mp4 (graded web),
#         <out_base>_poster.jpg. Audit prints "blackframes=N" — must be 0.
set -euo pipefail

FRAMES_DIR="$1"
OUT_BASE="$2"
FPS="${3:-60}"
POSTER="${4:-40}"
MODE="${5:-horizontal}"

# ffmpeg: $FFMPEG if set, else the copy vendored inside Isaac's python
# (imageio-ffmpeg, located via config/assets.json), else whatever is on PATH.
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
FF="${FFMPEG:-}"
if [ -z "$FF" ]; then
  FF="$(python -c "
import glob, json, os, sys
try:
    cfg = json.load(open(os.path.join(sys.argv[1], 'config', 'assets.json')))
    root = os.path.dirname(cfg['isaac_python'])
    pat = '/Lib/site-packages/imageio_ffmpeg/binaries/ffmpeg-*'
    hits = glob.glob(root + '/python' + pat) + glob.glob(root + '/kit/python' + pat)
    print(hits[0].replace(chr(92), '/') if hits else '')
except Exception:
    print('')
" "$REPO")"
fi
[ -z "$FF" ] && FF=ffmpeg

mkdir -p "$(dirname "$OUT_BASE")"

# A) frames -> raw master (near-lossless)
"$FF" -y -framerate "$FPS" -i "$FRAMES_DIR/frame_%05d.png" \
  -c:v libx264 -pix_fmt yuv420p -crf 18 -movflags +faststart \
  "${OUT_BASE}_raw.mp4"

# B) grade (this is where the dark-tech look happens — raw frames are LIGHT)
#    + web downscale. horizontal -> 1280w; vertical shorts stay full-res 1080x1920.
if [ "$MODE" = "vertical" ]; then
  VF="eq=contrast=1.06:saturation=1.10:brightness=0.01,vignette=PI/5,unsharp=3:3:0.4"
else
  VF="eq=contrast=1.06:saturation=1.10:brightness=0.01,vignette=PI/5,unsharp=3:3:0.4,scale=1280:-2"
fi
"$FF" -y -i "${OUT_BASE}_raw.mp4" -vf "$VF" \
  -c:v libx264 -pix_fmt yuv420p -crf 21 -movflags +faststart \
  "${OUT_BASE}.mp4"

# C) poster frame
"$FF" -y -i "${OUT_BASE}.mp4" -vf "select=eq(n\,${POSTER})" -vframes 1 \
  "${OUT_BASE}_poster.jpg"

# D) blackframe audit — anything > 0 means re-render (see GOTCHAS 5.15/5.16)
N=$("$FF" -i "${OUT_BASE}.mp4" -vf "blackframe=amount=50:threshold=32" -f null - 2>&1 \
  | grep -c "Parsed_blackframe" || true)
echo "blackframes=$N"
ls -la "${OUT_BASE}"*
