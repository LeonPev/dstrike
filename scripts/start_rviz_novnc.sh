#!/usr/bin/env bash
set -eo pipefail

export DISPLAY="${DISPLAY:-:1}"
export LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-1}"
export QT_X11_NO_MITSHM=1
export XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/tmp/runtime-root}"

VNC_PORT="${VNC_PORT:-5900}"
NOVNC_PORT="${NOVNC_PORT:-6080}"
VNC_GEOMETRY="${VNC_GEOMETRY:-1440x900x24}"
NOVNC_WEB_DIR="${NOVNC_WEB_DIR:-/usr/share/novnc}"
RVIZ_CONFIG="${RVIZ_CONFIG:-/workspace/config/drone.rviz}"

if [ ! -f "${RVIZ_CONFIG}" ]; then
  printf '[rviz-novnc] ERROR: RViz config not found: %s\n' "${RVIZ_CONFIG}" >&2
  exit 1
fi

mkdir -p "${XDG_RUNTIME_DIR}"
chmod 700 "${XDG_RUNTIME_DIR}"

source /opt/ros/humble/setup.bash
if [ -f /workspace/install/setup.bash ]; then
  source /workspace/install/setup.bash
fi

log() {
  printf '[rviz-novnc] %s\n' "$*"
}

pids=()
cleanup() {
  for pid in "${pids[@]:-}"; do
    if kill -0 "${pid}" >/dev/null 2>&1; then
      kill "${pid}" >/dev/null 2>&1 || true
    fi
  done
}
trap cleanup EXIT INT TERM

log "starting Xvfb on ${DISPLAY} with ${VNC_GEOMETRY}"
Xvfb "${DISPLAY}" -screen 0 "${VNC_GEOMETRY}" -nolisten tcp -noreset +extension GLX +render &
pids+=("$!")
sleep 1

log "starting fluxbox window manager"
fluxbox >/tmp/fluxbox.log 2>&1 &
pids+=("$!")
sleep 1

log "starting x11vnc on internal localhost:${VNC_PORT}"
x11vnc -display "${DISPLAY}" -rfbport "${VNC_PORT}" -localhost -nopw -forever -shared -noxdamage >/tmp/x11vnc.log 2>&1 &
pids+=("$!")
sleep 1

log "starting noVNC websockify on 0.0.0.0:${NOVNC_PORT}"
websockify --web "${NOVNC_WEB_DIR}" "0.0.0.0:${NOVNC_PORT}" "localhost:${VNC_PORT}" >/tmp/websockify.log 2>&1 &
pids+=("$!")
sleep 1

log "launching rviz2 with ${RVIZ_CONFIG}"
if command -v dbus-launch >/dev/null 2>&1; then
  dbus-launch --exit-with-session rviz2 -d "${RVIZ_CONFIG}" &
else
  rviz2 -d "${RVIZ_CONFIG}" &
fi
pids+=("$!")

log "open http://localhost:${NOVNC_PORT}/vnc.html?autoconnect=true&resize=scale"

wait -n "${pids[@]}"
