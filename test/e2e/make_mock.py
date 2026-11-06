# 실제 xacro 를 펼쳐 하드웨어만 mock_components/GenericSystem 으로 바꾼 URDF 와, 테스트용 파라미터를 만든다.
import re, sys, yaml
urdf_in, yaml_in, out_dir, model, port, log_path = sys.argv[1:7]
cfg = yaml.safe_load(open(yaml_in))
p = cfg["kid_rl"]["ros__parameters"]
joints, defaults = p["joints"], p["default_positions"]
u = open(urdf_in).read()
u = re.sub(r"<hardware>.*?</hardware>",
           "<hardware><plugin>mock_components/GenericSystem</plugin>"
           "<param name=\"calculate_dynamics\">true</param></hardware>", u, flags=re.S)
u = re.sub(r"<gpio name=.*?</gpio>", "", u, flags=re.S)
u = re.sub(r"<transmission name=.*?</transmission>", "", u, flags=re.S)
u = re.sub(r"\s*<state_interface name=\"rx_missing_id\"/>", "", u)
for j, d in zip(joints, defaults):
    block = re.search(rf"<joint name=\"{j}\">\s*<command_interface.*?</joint>", u, flags=re.S)
    b = block.group(0)
    b2 = b.replace('<state_interface name="position"/>',
                   f'<state_interface name="position"><param name="initial_value">{d}</param></state_interface>')
    b2 = b2.replace('<state_interface name="velocity"/>',
                    '<state_interface name="velocity"><param name="initial_value">0.0</param></state_interface>')
    b2 = b2.replace('<state_interface name="hardware_state"/>',
                    '<state_interface name="hardware_state"><param name="initial_value">0.0</param></state_interface>')
    u = u.replace(b, b2)
open(f"{out_dir}/mock.urdf", "w").write(u)
p["policy_path"] = model
p["imu.port"] = port
p["log.path"] = log_path
yaml.safe_dump(cfg, open(f"{out_dir}/params.yaml", "w"))
print("joints", len(joints), "gpio left", u.count("<gpio"), "hardware_state", u.count('name="hardware_state"'))
