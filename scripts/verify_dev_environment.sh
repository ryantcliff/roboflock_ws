#!/usr/bin/env bash

set -u -o pipefail

readonly ros_distro='humble'
readonly script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
readonly workspace_root="$(cd "${script_dir}/.." && pwd -P)"
skip_build=false
errors=0
warnings=0

usage() {
  cat <<'EOF'
Usage: scripts/verify_dev_environment.sh [--skip-build]

Verify the RoboFlock software environment and report attached hardware.
By default the script rebuilds and tests the workspace. --skip-build checks
the installed environment and existing workspace artifacts only.
EOF
}

pass() {
  printf 'PASS  %s\n' "$1"
}

warn() {
  printf 'WARN  %s\n' "$1"
  warnings=$((warnings + 1))
}

fail() {
  printf 'FAIL  %s\n' "$1" >&2
  errors=$((errors + 1))
}

check_command() {
  if command -v "$1" >/dev/null 2>&1; then
    pass "$1 is available"
  else
    fail "$1 is missing"
  fi
}

check_python_dependencies() {
  if python3 - <<'PY'
from importlib import import_module
from importlib.metadata import version

expected = {
    'meshtastic': '2.7.11',
    'odrive': '0.6.11.post1',
    'adafruit-circuitpython-bno08x': '1.3.3',
    'adafruit-extended-bus': '1.0.2',
    'cffi': '2.1.1',
}
modules = (
    'meshtastic.serial_interface',
    'odrive',
    'adafruit_bno08x',
    'adafruit_extended_bus',
    'cffi',
)

for module in modules:
    import_module(module)
for distribution, wanted in expected.items():
    found = version(distribution)
    if found != wanted:
        raise RuntimeError(f'{distribution}: expected {wanted}, found {found}')
PY
  then
    pass 'pinned Python hardware libraries import successfully'
  else
    fail 'pinned Python hardware libraries are missing or have wrong versions'
  fi

  if python3 -m pip check; then
    pass 'Python package requirements are consistent'
  else
    fail 'Python package requirements are inconsistent'
  fi
}

check_ros_environment() {
  local package

  if [[ ! -r "/opt/ros/${ros_distro}/setup.bash" ]]; then
    fail "ROS 2 ${ros_distro} is not installed under /opt/ros/${ros_distro}"
    return
  fi

  set +u
  # shellcheck disable=SC1091
  source "/opt/ros/${ros_distro}/setup.bash"
  set -u
  pass "ROS 2 ${ros_distro} setup is readable"

  if rosdep check --from-paths "${workspace_root}/src" --ignore-src \
      --rosdistro "${ros_distro}"; then
    pass 'rosdep reports all system dependencies installed'
  else
    fail 'rosdep reports missing system dependencies'
  fi

  if [[ ! -r "${workspace_root}/install/setup.bash" ]]; then
    fail 'workspace install/setup.bash is missing; run the bootstrap build'
    return
  fi

  set +u
  # shellcheck disable=SC1091
  source "${workspace_root}/install/setup.bash"
  set -u
  for package in beacon_pkg bring_up lidar_test mpu9250driver \
      rf2o_laser_odometry rplidar_ros ultrasonic_pkg urdf_description; do
    if ros2 pkg prefix "${package}" >/dev/null 2>&1; then
      pass "ROS package ${package} is installed"
    else
      fail "ROS package ${package} is not installed"
    fi
  done
}

build_and_test() {
  if [[ "${skip_build}" == true ]]; then
    printf 'Build and test run skipped by request.\n'
    return
  fi

  if (
    cd "${workspace_root}"
    colcon build --symlink-install --event-handlers console_cohesion+
  ); then
    pass 'workspace build completed'
  else
    fail 'workspace build failed'
    return
  fi

  set +u
  # shellcheck disable=SC1091
  source "${workspace_root}/install/setup.bash"
  set -u
  if (
    cd "${workspace_root}"
    colcon test --event-handlers console_cohesion+
  ) && colcon test-result --test-result-base "${workspace_root}/build"; then
    pass 'workspace tests completed without failures'
  else
    fail 'workspace tests failed'
  fi
}

report_hardware() {
  local label
  local pattern

  if grep -qi microsoft /proc/version 2>/dev/null; then
    printf 'WSL hardware access is informational; missing devices do not fail software checks.\n'
  else
    printf 'Hardware discovery is informational; missing devices do not fail software checks.\n'
  fi

  while IFS='|' read -r label pattern; do
    if compgen -G "${pattern}" >/dev/null; then
      pass "${label} device detected (${pattern})"
    else
      warn "no ${label} device detected (${pattern})"
    fi
  done <<'EOF'
I2C|/dev/i2c-*
USB serial|/dev/ttyUSB*
USB ACM serial|/dev/ttyACM*
joystick|/dev/input/js*
EOF
}

main() {
  while (($# > 0)); do
    case "$1" in
      --skip-build) skip_build=true ;;
      -h | --help)
        usage
        return 0
        ;;
      *)
        usage >&2
        printf 'ERROR: unknown argument: %s\n' "$1" >&2
        return 2
        ;;
    esac
    shift
  done

  check_command cmake
  check_command colcon
  check_command gcc
  check_command g++
  check_command python3
  check_command rosdep
  check_python_dependencies
  check_ros_environment
  build_and_test
  report_hardware

  if ((errors > 0)); then
    printf '%d software check(s) failed; %d hardware warning(s).\n' \
      "${errors}" "${warnings}" >&2
    return 1
  fi

  printf 'Software checks passed with %d hardware warning(s).\n' "${warnings}"
}

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  main "$@"
fi
