#!/usr/bin/env bash
# One-time (or as needed): load v4l2loopback as "Kinect Webcam" for RGB/IR apps.
# The panel never calls this for privilege. If a loopback already exists, use it.
set -euo pipefail
LABEL="${KINECT_VCAM_LABEL:-Kinect Webcam}"
LABEL_FIRST="${LABEL%% *}"

find_loopback() {
  local n base name
  for n in /sys/class/video4linux/video*/name; do
    [[ -f "$n" ]] || continue
    base="$(basename "$(dirname "$n")")"
    name="$(cat "$n" 2>/dev/null || true)"
    if [[ "$name" == "$LABEL" ]]; then
      echo "/dev/$base"
      return 0
    fi
  done
  # Unquoted card_label=Kinect Webcam becomes card type "Kinect"
  for n in /sys/class/video4linux/video*/name; do
    [[ -f "$n" ]] || continue
    base="$(basename "$(dirname "$n")")"
    name="$(cat "$n" 2>/dev/null || true)"
    if [[ "$name" == "$LABEL_FIRST" || "$name" == "$LABEL_FIRST "* ]]; then
      echo "/dev/$base"
      return 0
    fi
  done
  for n in /sys/class/video4linux/video*/name; do
    [[ -f "$n" ]] || continue
    base="$(basename "$(dirname "$n")")"
    if [[ -e /sys/devices/virtual/video4linux/$base ]]; then
      echo "/dev/$base"
      return 0
    fi
  done
  return 1
}

if [[ "${1:-}" == "--check" ]]; then
  find_loopback
  exit $?
fi

# Already present? Never pkexec just to rename a live node.
if DEV=$(find_loopback); then
  name="$(cat /sys/class/video4linux/$(basename "$DEV")/name 2>/dev/null || echo "?")"
  echo "OK: $DEV ($name) — already loaded, no password"
  exit 0
fi

run_root() {
  if [[ "$(id -u)" -eq 0 ]]; then
    "$@"
  elif [[ -t 0 ]] && command -v sudo >/dev/null; then
    sudo "$@"
  elif command -v pkexec >/dev/null; then
    pkexec "$@"
  else
    sudo "$@"
  fi
}

if [[ "${1:-}" != "--load" && "${1:-}" != "--reload" ]]; then
  echo "No v4l2loopback node. Load once (will ask password):" >&2
  echo "  $0 --load" >&2
  exit 1
fi

if [[ "${1:-}" == "--reload" ]]; then
  echo "Reloading v4l2loopback as \"$LABEL\"…"
  run_root bash -c "
set -e
modprobe -r v4l2loopback || true
modprobe v4l2loopback devices=1 exclusive_caps=1 card_label='$LABEL' max_buffers=4
mkdir -p /etc/modules-load.d /etc/modprobe.d
echo v4l2loopback > /etc/modules-load.d/kinect-webcam.conf
printf '%s\n' 'options v4l2loopback devices=1 exclusive_caps=1 card_label=\"$LABEL\" max_buffers=4' \
  > /etc/modprobe.d/kinect-webcam.conf
"
else
  echo "Loading v4l2loopback as \"$LABEL\"…"
  run_root bash -c "
set -e
modprobe v4l2loopback devices=1 exclusive_caps=1 card_label='$LABEL' max_buffers=4
mkdir -p /etc/modules-load.d /etc/modprobe.d
echo v4l2loopback > /etc/modules-load.d/kinect-webcam.conf
printf '%s\n' 'options v4l2loopback devices=1 exclusive_caps=1 card_label=\"$LABEL\" max_buffers=4' \
  > /etc/modprobe.d/kinect-webcam.conf
"
fi

sleep 0.4
if DEV=$(find_loopback); then
  echo "OK: $DEV ($LABEL) — pick this device in Zoom/browser/OBS"
  exit 0
fi
echo "FAIL: module loaded but loopback device not found" >&2
ls -la /dev/video* 2>/dev/null || true
for n in /sys/class/video4linux/video*/name; do echo "  $(basename $(dirname $n)): $(cat $n)"; done
exit 1
