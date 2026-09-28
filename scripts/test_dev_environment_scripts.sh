#!/usr/bin/env bash

set -u

workspace_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
bootstrap="${workspace_root}/scripts/bootstrap_ubuntu_22_04.sh"
verify="${workspace_root}/scripts/verify_dev_environment.sh"
failures=0

fail() {
  printf 'FAIL: %s\n' "$1" >&2
  failures=$((failures + 1))
}

assert_contains() {
  local output="$1"
  local expected="$2"
  local context="$3"

  if [[ "${output}" != *"${expected}"* ]]; then
    fail "${context}: expected output to contain: ${expected}"
  fi
}

test_rejects_unsupported_ubuntu_release() {
  local fake_release
  local output
  local status

  fake_release="$(mktemp)"
  printf 'ID=ubuntu\nVERSION_ID="24.04"\n' > "${fake_release}"

  output="$(ROBOFLOCK_OS_RELEASE_FILE="${fake_release}" \
    bash "${bootstrap}" --dry-run 2>&1)"
  status=$?
  rm -f "${fake_release}"

  if ((status == 0)); then
    fail 'bootstrap accepted Ubuntu 24.04'
  fi
  assert_contains "${output}" 'requires Ubuntu 22.04' \
    'unsupported Ubuntu release'
}

test_dry_run_is_complete_and_read_only() {
  local before
  local dry_run_temp
  local sentinel
  local output
  local status
  local after

  dry_run_temp="$(mktemp -d)"
  sentinel="${dry_run_temp}/ros2-apt-source_jammy_all.deb"
  printf 'keep me\n' > "${sentinel}"
  before="$(git -C "${workspace_root}" status --porcelain)"
  output="$(ROBOFLOCK_TEMP_DIR="${dry_run_temp}" \
    bash "${bootstrap}" --dry-run 2>&1)"
  status=$?
  after="$(git -C "${workspace_root}" status --porcelain)"

  if ((status != 0)); then
    fail "bootstrap dry run exited ${status}"
  fi
  if [[ "${before}" != "${after}" ]]; then
    fail 'bootstrap dry run changed the working tree'
  fi
  if [[ ! -f "${sentinel}" || "$(<"${sentinel}")" != 'keep me' ]]; then
    fail 'bootstrap dry run changed an existing temporary file'
  fi
  assert_contains "${output}" "${sentinel}" 'dry-run temporary path'
  assert_contains "${output}" 'ros-humble-desktop' 'ROS installation plan'
  assert_contains "${output}" 'rosdep install --from-paths' 'rosdep plan'
  assert_contains "${output}" 'meshtastic==2.7.11' 'Meshtastic pin'
  assert_contains "${output}" 'odrive==0.6.11.post1' 'ODrive pin'
  assert_contains "${output}" 'adafruit-circuitpython-bno08x==1.3.3' 'BNO085 pin'
  assert_contains "${output}" 'adafruit-extended-bus==1.0.2' 'extended bus pin'
  assert_contains "${output}" 'colcon build --symlink-install' 'workspace build plan'
  rm -f "${sentinel}"
  rmdir "${dry_run_temp}"
}

test_verifier_treats_wsl_hardware_as_informational() {
  local output
  local status

  output="$(bash "${verify}" --skip-build 2>&1)"
  status=$?

  if ((status != 0)); then
    fail "environment verifier exited ${status}"
  fi
  assert_contains "${output}" 'WSL hardware access is informational' \
    'WSL hardware classification'
  assert_contains "${output}" 'Software checks passed' 'software result'
}

test_rejects_unsupported_ubuntu_release
test_dry_run_is_complete_and_read_only
test_verifier_treats_wsl_hardware_as_informational

if ((failures > 0)); then
  printf '%d development-environment test(s) failed\n' "${failures}" >&2
  exit 1
fi

printf 'All development-environment script tests passed\n'
