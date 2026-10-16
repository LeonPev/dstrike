#!/bin/sh
set -eu

fail() {
  printf 'ERROR: %s\n' "$*" >&2
  exit 1
}

section() {
  printf '\n== %s ==\n' "$1"
}

docker_info_value() {
  docker info --format "$1" 2>/dev/null || true
}

is_number() {
  case "$1" in
    ''|*[!0-9]*) return 1 ;;
    *) return 0 ;;
  esac
}

section "Host"
host_os="$(uname -s 2>/dev/null || true)"
host_arch="$(uname -m 2>/dev/null || true)"
printf 'Host OS: %s\n' "${host_os:-unknown}"
printf 'Mac architecture: %s\n' "${host_arch:-unknown}"

if [ "$host_os" != "Darwin" ]; then
  fail "Docker Desktop for Mac is required; detected host OS '${host_os:-unknown}'."
fi

section "Docker CLI"
if ! command -v docker >/dev/null 2>&1; then
  fail "docker CLI not found. Install and start Docker Desktop for Mac."
fi
docker --version || fail "docker CLI is installed but did not run."

section "Docker Desktop / Engine"
docker version || fail "Docker Engine is not reachable. Start Docker Desktop for Mac and retry."

docker_os="$(docker_info_value '{{.OperatingSystem}}')"
docker_ostype="$(docker_info_value '{{.OSType}}')"
docker_arch="$(docker_info_value '{{.Architecture}}')"
docker_root="$(docker_info_value '{{.DockerRootDir}}')"

printf 'Docker operating system: %s\n' "${docker_os:-unknown}"
printf 'Docker OSType: %s\n' "${docker_ostype:-unknown}"
printf 'Docker architecture: %s\n' "${docker_arch:-unknown}"
printf 'Docker root dir: %s\n' "${docker_root:-unknown}"

case "$docker_os" in
  *"Docker Desktop"*) ;;
  *) fail "Docker Desktop is required; docker info reports '${docker_os:-unknown}'." ;;
esac

if [ "$docker_ostype" != "linux" ]; then
  fail "Linux containers are required; docker info reports OSType '${docker_ostype:-unknown}'."
fi

section "Docker Resources"
docker_cpus="$(docker_info_value '{{.NCPU}}')"
docker_mem_bytes="$(docker_info_value '{{.MemTotal}}')"

printf 'Docker CPU limit: %s\n' "${docker_cpus:-not reported}"
if is_number "$docker_mem_bytes"; then
  docker_mem_mib=$((docker_mem_bytes / 1024 / 1024))
  printf 'Docker memory limit: %s MiB (%s bytes)\n' "$docker_mem_mib" "$docker_mem_bytes"
else
  printf 'Docker memory limit: not reported\n'
fi

section "Docker Compose"
docker compose version || fail "Docker Compose plugin is not available through 'docker compose'."

section "Docker Disk Usage"
docker system df || fail "Unable to read Docker disk usage with 'docker system df'."
printf 'Free Docker disk space: not reported by Docker CLI; inspect Docker Desktop Resources if needed.\n'

section "Linux Container Smoke Test"
docker run --rm hello-world >/dev/null || fail "Failed to run the hello-world Linux container."
printf 'hello-world Linux container ran successfully.\n'

printf '\nPASS: DFD-000 Docker host preflight passed\n'
