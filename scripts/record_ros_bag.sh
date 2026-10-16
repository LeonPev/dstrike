#!/usr/bin/env bash
set -euo pipefail

DURATION="${DURATION:-10}"
BAG_TOPICS="${BAG_TOPICS:-/drone/odom /sim/ground_truth/odom /tf /sim/scenario_markers}"
OPTIONAL_BAG_TOPICS="${OPTIONAL_BAG_TOPICS:-/sim/camera/front/image_raw}"
ARTIFACTS_DIR="${ARTIFACTS_DIR:-/workspace/artifacts}"
TOPIC_WAIT_TIMEOUT="${TOPIC_WAIT_TIMEOUT:-30}"

if [[ ! "${DURATION}" =~ ^[0-9]+([.][0-9]+)?$ ]]; then
  printf 'DURATION must be a positive number, got %s\n' "${DURATION}" >&2
  exit 2
fi

if [[ ! "${TOPIC_WAIT_TIMEOUT}" =~ ^[0-9]+([.][0-9]+)?$ ]]; then
  printf 'TOPIC_WAIT_TIMEOUT must be a positive number, got %s\n' "${TOPIC_WAIT_TIMEOUT}" >&2
  exit 2
fi

if ! command -v ros2 >/dev/null 2>&1; then
  printf 'ros2 command not found\n' >&2
  exit 2
fi

missing_required_topics() {
  local topics="$1"
  local missing=()
  local topic

  for topic in ${BAG_TOPICS}; do
    if ! grep -Fxq "${topic}" <<<"${topics}"; then
      missing+=("${topic}")
    fi
  done

  printf '%s\n' "${missing[@]}"
}

deadline_epoch="$(python3 -c "import time; print(time.time() + float('${TOPIC_WAIT_TIMEOUT}'))")"
current_topics="$(ros2 topic list || true)"
while [[ "$(python3 -c "import time; print(time.time() < float('${deadline_epoch}'))")" == "True" ]]; do
  if [[ -z "$(missing_required_topics "${current_topics}")" ]]; then
    break
  fi
  sleep 1
  current_topics="$(ros2 topic list || true)"
done

missing_topics="$(missing_required_topics "${current_topics}")"
if [[ -n "${missing_topics}" ]]; then
  printf 'Missing required topics before bag record:\n%s\n' "${missing_topics}" >&2
  exit 1
fi

record_topics=()
topic=
for topic in ${BAG_TOPICS}; do
  record_topics+=("${topic}")
done

for topic in ${OPTIONAL_BAG_TOPICS}; do
  if grep -Fxq "${topic}" <<<"${current_topics}"; then
    record_topics+=("${topic}")
  else
    printf 'Skipping optional topic not currently visible: %s\n' "${topic}"
  fi
done

storage_args=()
storage_label="default"
if ros2 pkg prefix rosbag2_storage_mcap >/dev/null 2>&1; then
  storage_args=(--storage mcap)
  storage_label="mcap"
fi

timestamp="$(date -u +%Y%m%dT%H%M%SZ)"
bag_root="${ARTIFACTS_DIR}/rosbags"
bag_path="${bag_root}/dfd-015-${timestamp}"
mkdir -p "${bag_root}"

printf 'Recording ROS bag storage=%s duration_s=%s output=%s\n' \
  "${storage_label}" "${DURATION}" "${bag_path}"
printf 'Recording topics:\n'
printf '  %s\n' "${record_topics[@]}"

record_status=0
timeout --signal=SIGINT --kill-after=5s "${DURATION}s" \
  ros2 bag record "${storage_args[@]}" -o "${bag_path}" "${record_topics[@]}" \
  || record_status="$?"

if [[ "${record_status}" != "0" && "${record_status}" != "124" && "${record_status}" != "130" ]]; then
  printf 'ros2 bag record failed with status %s\n' "${record_status}" >&2
  exit "${record_status}"
fi

if [[ ! -d "${bag_path}" ]]; then
  printf 'Expected bag directory was not created: %s\n' "${bag_path}" >&2
  exit 1
fi

if ! find "${bag_path}" -mindepth 1 -type f -print -quit | grep -q .; then
  printf 'Bag directory has no files: %s\n' "${bag_path}" >&2
  exit 1
fi

find "${bag_path}" -maxdepth 2 -type f -print | sort
printf 'PASS: DFD-015 bag record artifacts passed\n'
