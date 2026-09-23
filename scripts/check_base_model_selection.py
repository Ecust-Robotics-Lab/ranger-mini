#!/usr/bin/env python3
"""Compile the actual LoadParameters body with a parameter-only shim; no SDK/CAN."""
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
REPO = ROOT / 'ws_base/src/ranger_ros2'


def extract(source):
    start = source.index('void RangerROSMessenger::LoadParameters()')
    left = source.index('{', start)
    depth = 1
    end = left + 1
    while depth:
        depth += (source[end] == '{') - (source[end] == '}')
        end += 1
    return source[start:end]


def check(source, folder, name):
    header = (REPO / 'ranger_base/include/ranger_base/ranger_messenger.hpp').read_text()
    structs = header[header.index('  struct RobotParams'):header.index(' public:')]
    code = r'''
#include <string>
#include <memory>
#include <cassert>
#include <iostream>
#include "ranger_base/ranger_params.hpp"
#define RCLCPP_INFO(...) ((void)0)
namespace westonrobot {
struct ParameterShim {
  std::string model;
  template<typename T> T declare_parameter(const char*, T value) { return value; }
};
template<> std::string ParameterShim::declare_parameter(const char* name, std::string value) {
  return std::string(name) == "robot_model" ? model : value;
}
struct RangerROSMessenger {
''' + structs + r'''
  std::shared_ptr<ParameterShim> node_ = std::make_shared<ParameterShim>();
  std::string port_name_, robot_model_, odom_frame_, base_frame_, odom_topic_name_;
  int update_rate_;
  bool publish_odom_tf_, parking_mode_;
  RangerSubType robot_type_;
  RobotParams robot_params_;
  void LoadParameters();
};
''' + extract(source) + r'''
}
int main() {
  using namespace westonrobot;
  const char* models[] = {"ranger", "ranger_mini_v1", "ranger_mini_v2", "ranger_mini_v3"};
  const double tracks[] = {0.56, 0.36, 0.364, 0.364};
  const double bases[] = {0.90, 0.36, 0.494, 0.494};
  for (int i=0; i<4; ++i) {
    RangerROSMessenger m;
    m.node_->model = models[i];
    m.LoadParameters();
    if (static_cast<int>(m.robot_type_) != i || m.robot_params_.track != tracks[i] ||
        m.robot_params_.wheelbase != bases[i]) {
      std::cerr << models[i] << " selected wrong variant/geometry\n";
      return 1;
    }
    std::cout << models[i] << " variant/geometry PASS\n";
  }
}
'''
    cpp = folder / (name + '.cpp')
    binary = folder / name
    cpp.write_text(code)
    subprocess.run(['g++', '-std=c++14', '-I' + str(REPO / 'ranger_base/include'),
                    str(cpp), '-o', str(binary)], check=True)
    result = subprocess.run([str(binary)], capture_output=True, text=True)
    print(name, 'exit=', result.returncode, '\n' + result.stdout + result.stderr)
    return result.returncode


def main():
    current = (REPO / 'ranger_base/src/ranger_messenger.cpp').read_text()
    original = subprocess.check_output(['git', '-C', str(REPO), 'show',
                                        'HEAD:ranger_base/src/ranger_messenger.cpp'], text=True)
    with tempfile.TemporaryDirectory(prefix='base-model-test-') as temp:
        folder = Path(temp)
        assert check(original, folder, 'upstream') == 1, 'Regression must reproduce upstream V2 failure'
        assert check(current, folder, 'patched') == 0
    print('PASS: actual parameter method regression; no ROS node or CAN access.')


if __name__ == '__main__':
    main()
