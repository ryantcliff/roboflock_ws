#!/usr/bin/env bash

set -Eeuo pipefail

readonly ros_distro='humble'
readonly ubuntu_version='22.04'
readonly script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
readonly workspace_root="$(cd "${script_dir}/.." && pwd -P)"
dry_run=false
ros_apt_deb=''
ros_apt_deb_owned=false

usage() {
  cat <<'EOF'
Usage: scripts/bootstrap_ubuntu_22_04.sh [--dry-run]

Install and build the RoboFlock ROS 2 Humble development environment on
Ubuntu 22.04. --dry-run prints the complete command plan without changing the
machine or workspace.
EOF
}

die() {
  printf 'ERROR: %s\n' "$1" >&2
  exit 1
}

print_command() {
  printf '+'
  printf ' %q' "$@"
  printf '\n'
}

run() {
  print_command "$@"
  if [[ "${dry_run}" == false ]]; then
    "$@"
  fi
}

run_as_root() {
  if ((EUID == 0)); then
    run "$@"
  elif command -v sudo >/dev/null 2>&1; then
    run sudo "$@"
  else
    die "root privileges are required to run: $*"
  fi
}

cleanup() {
  if [[ "${ros_apt_deb_owned}" == true && -n "${ros_apt_deb}" && \
        -f "${ros_apt_deb}" ]]; then
    rm -f -- "${ros_apt_deb}"
  fi
}

validate_platform() {
  local os_release_file="${ROBOFLOCK_OS_RELEASE_FILE:-/etc/os-release}"
  local architecture

  [[ -r "${os_release_file}" ]] || die "cannot read ${os_release_file}"

  # shellcheck disable=SC1090
  source "${os_release_file}"
  if [[ "${ID:-}" != ubuntu || "${VERSION_ID:-}" != "${ubuntu_version}" ]]; then
    die "RoboFlock bootstrap requires Ubuntu 22.04; found ${ID:-unknown} ${VERSION_ID:-unknown}"
  fi

  architecture="$(dpkg --print-architecture)"
  case "${architecture}" in
    amd64 | arm64) ;;
    *) die "ROS 2 Humble packages require amd64 or arm64; found ${architecture}" ;;
  esac
}

install_ros_repository() {
  local apt_source_version
  local apt_source_url

  if [[ -e /etc/apt/sources.list.d/ros2.sources || \
        -e /etc/apt/sources.list.d/ros2.list ]] && [[ "${dry_run}" == false ]]; then
    printf 'ROS 2 apt repository is already configured.\n'
    return
  fi

  if [[ "${dry_run}" == true ]]; then
    apt_source_version="${ROS_APT_SOURCE_VERSION:-latest-release}"
    ros_apt_deb="${ROBOFLOCK_TEMP_DIR:-${TMPDIR:-/tmp}}/ros2-apt-source_jammy_all.deb"
    apt_source_url="https://github.com/ros-infrastructure/ros-apt-source/releases/download/${apt_source_version}/ros2-apt-source_${apt_source_version}.jammy_all.deb"
    run curl -L --fail --silent --show-error -o "${ros_apt_deb}" \
      "${apt_source_url}"
    run_as_root dpkg -i "${ros_apt_deb}"
    return
  fi

  apt_source_version="${ROS_APT_SOURCE_VERSION:-}"
  if [[ -z "${apt_source_version}" ]]; then
    apt_source_version="$(
      curl --fail --silent --show-error \
        https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest |
        sed -n 's/.*"tag_name": "\([^"]*\)".*/\1/p' |
        head -n 1
    )"
  fi
  [[ -n "${apt_source_version}" ]] || die 'could not determine ros2-apt-source version'

  ros_apt_deb="$(mktemp \
    --tmpdir="${ROBOFLOCK_TEMP_DIR:-${TMPDIR:-/tmp}}" \
    ros2-apt-source.XXXXXX.deb)"
  ros_apt_deb_owned=true
  apt_source_url="https://github.com/ros-infrastructure/ros-apt-source/releases/download/${apt_source_version}/ros2-apt-source_${apt_source_version}.jammy_all.deb"
  run curl -L --fail --silent --show-error -o "${ros_apt_deb}" "${apt_source_url}"
  run_as_root dpkg -i "${ros_apt_deb}"
}

initialize_rosdep() {
  if [[ "${dry_run}" == true || \
        ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
    run_as_root rosdep init
  else
    printf 'rosdep is already initialized.\n'
  fi
  run rosdep update --rosdistro "${ros_distro}"
}

install_dependencies() {
  run_as_root env DEBIAN_FRONTEND=noninteractive apt-get update
  run_as_root env DEBIAN_FRONTEND=noninteractive apt-get install -y \
    curl locales software-properties-common
  run_as_root add-apt-repository -y universe

  install_ros_repository

  run_as_root env DEBIAN_FRONTEND=noninteractive apt-get update
  run_as_root env DEBIAN_FRONTEND=noninteractive apt-get install -y \
    ros-humble-desktop ros-dev-tools python3-pip python3-venv \
    libi2c-dev i2c-tools

  initialize_rosdep

  if [[ "${dry_run}" == false ]]; then
    set +u
    # shellcheck disable=SC1091
    source "/opt/ros/${ros_distro}/setup.bash"
    set -u
  fi
  run rosdep install --from-paths "${workspace_root}/src" --ignore-src \
    --rosdistro "${ros_distro}" -r -y

  run python3 -m pip install --user --upgrade \
    'meshtastic==2.7.11' \
    'odrive==0.6.11.post1' \
    'adafruit-circuitpython-bno08x==1.3.3' \
    'adafruit-extended-bus==1.0.2' \
    'cffi==2.1.1'
  run mkdir -p "${HOME}/.ros/log" "${HOME}/.gazebo"
}

build_workspace() {
  if [[ "${dry_run}" == false ]]; then
    cd "${workspace_root}"
  else
    printf '+ cd %q\n' "${workspace_root}"
  fi
  run colcon build --symlink-install --event-handlers console_cohesion+
}

main() {
  while (($# > 0)); do
    case "$1" in
      --dry-run) dry_run=true ;;
      -h | --help)
        usage
        return 0
        ;;
      *)
        usage >&2
        die "unknown argument: $1"
        ;;
    esac
    shift
  done

  validate_platform
  trap cleanup EXIT
  install_dependencies
  build_workspace

  if [[ "${dry_run}" == true ]]; then
    printf '\nDry run complete; no changes were made.\n'
    return
  fi

  cat <<EOF

RoboFlock development environment is ready.
Activate it in each terminal with:
  source /opt/ros/${ros_distro}/setup.bash
  source ${workspace_root}/install/setup.bash

Run scripts/verify_dev_environment.sh for a complete software check.
EOF
}

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  main "$@"
fi
