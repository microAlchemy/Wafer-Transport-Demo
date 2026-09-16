"""Run the installer with mocked system commands; never modify the host OS."""
import json
import os
from pathlib import Path
import subprocess
import sys

import pytest

ROOT = Path(__file__).resolve().parents[1]


def run_installer(tmp_path, release='26.04', arch='arm64', curl_fails=False):
    log = tmp_path / 'commands.log'
    os_release = tmp_path / 'os-release'
    codename = 'resolute' if release == '26.04' else 'noble'
    os_release.write_text(f'ID=ubuntu\nVERSION_ID={release}\nVERSION_CODENAME={codename}\n')
    setup = tmp_path / 'setup.bash'
    setup.write_text('export ROS_DISTRO=lyrical\n')
    binary_dir = tmp_path / 'bin'
    binary_dir.mkdir()
    # Every potentially mutating command is intercepted before invoking bash.
    mock = f'''#!{sys.executable}
import json, os, pathlib, sys
name = pathlib.Path(sys.argv[0]).name
with open(os.environ['WAFER_TEST_COMMAND_LOG'], 'a') as log:
    log.write(json.dumps([name, *sys.argv[1:]]) + '\\n')
if name == 'dpkg':
    print(os.environ['WAFER_TEST_ARCH'])
elif name == 'curl':
    if os.environ['WAFER_TEST_CURL_FAIL'] == '1':
        sys.exit(22)
    target = pathlib.Path(sys.argv[sys.argv.index('--output') + 1])
    target.write_text('{{"tag_name": "1.0.0"}}' if target.suffix == '.json' else 'mock deb')
'''
    for name in ('sudo', 'curl', 'dpkg', 'rosdep'):
        target = binary_dir / name
        target.write_text(mock)
        target.chmod(0o755)
    (binary_dir / 'python3').symlink_to(sys.executable)
    # Substitute only read-only system paths in a test copy of the real script.
    script = (ROOT / 'scripts/install_ubuntu.sh').read_text()
    script = script.replace('/etc/os-release', str(os_release))
    script = script.replace('/opt/ros/lyrical/setup.bash', str(setup))
    script = script.replace('/etc/ros/rosdep/sources.list.d/20-default.list',
                            str(tmp_path / 'absent-rosdep-list'))
    env = dict(os.environ, PATH=str(binary_dir) + os.pathsep + os.environ['PATH'],
               WAFER_TEST_COMMAND_LOG=str(log), WAFER_TEST_ARCH=arch,
               WAFER_TEST_CURL_FAIL='1' if curl_fails else '0')
    result = subprocess.run(['bash', '-c', script], env=env, text=True,
                            capture_output=True, timeout=30)
    commands = [json.loads(line) for line in log.read_text().splitlines()] if log.exists() else []
    return result, commands


@pytest.mark.parametrize('arch', ['arm64', 'amd64'])
def test_installs_native_resolute_lyrical_and_bootstraps_curl(tmp_path, arch):
    result, commands = run_installer(tmp_path, arch=arch)
    assert result.returncode == 0, result.stderr
    first_download = next(i for i, c in enumerate(commands) if c[0] == 'curl')
    bootstrap = next(i for i, c in enumerate(commands)
                     if c[:3] == ['sudo', 'apt-get', 'install'] and 'curl' in c)
    assert bootstrap < first_download
    assert any(any('.resolute_all.deb' in arg for arg in c) for c in commands)
    assert not any(any('noble' in arg or 'jazzy' in arg for arg in c) for c in commands)
    assert any('ros-lyrical-ros-gz' in c and 'ros-dev-tools' in c for c in commands)
    assert ['rosdep', 'update', '--rosdistro', 'lyrical'] in commands


def test_failed_download_stops_before_dpkg_or_ros_package_install(tmp_path):
    result, commands = run_installer(tmp_path, curl_fails=True)
    assert result.returncode != 0
    assert not any(c[:2] == ['sudo', 'dpkg'] for c in commands)
    assert not any('ros-lyrical-desktop' in c for c in commands)
    assert 'Installation stopped' in result.stderr
    assert 'JSONDecodeError' not in result.stderr


def test_other_ubuntu_release_is_rejected_before_mutation(tmp_path):
    result, commands = run_installer(tmp_path, release='24.04')
    assert result.returncode != 0
    assert not commands
    assert 'No packages were changed' in result.stderr
