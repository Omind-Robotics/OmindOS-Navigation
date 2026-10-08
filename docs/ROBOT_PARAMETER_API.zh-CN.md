# 机器人参数 API

客户准备 URDF 和关节驱动参数，通过接口提交；无需修改 OmindOS 源码。接口保存一份不可覆盖的配置版本，返回 `profile_id`。修改后再次提交产生新编号；完全相同的请求返回同一编号。应用可按编号读取并校验内容。

当前已实现：参数校验、保存、版本编号和读取。尚未实现：ROS 2 控制端启用配置、驱动适配、任意模型的自动训练。`runtime_applied` 始终为 `false`。模型、减速比和零点变更的停止后加载流程将在控制端接入时实现。

## 启动与调用

开发环境使用 Python 标准库即可启动。当前服务仅监听本机，不作为公网多租户服务：

```bash
python3 scripts/robot_parameter_api.py --store ./robot-configurations --port 8085
curl --fail-with-body -X POST http://127.0.0.1:8085/v1/robot-profiles \
  -H 'Content-Type: application/json' --data-binary @robot-parameters.json
curl --fail-with-body http://127.0.0.1:8085/v1/robot-profiles/PROFILE_ID
```

客户准备的数据只有两组，另加机器标识：

| 字段 | 内容 |
| --- | --- |
| `robot_id` | 本机标识，1–64 个字母/数字/下划线/点/短横线，以字母或数字开头 |
| `kinematics.urdf_xml` | URDF XML 文本，含几何、关节、各连杆质量/质心/惯量和关节限位 |
| `dynamics.joints` | 以 URDF 关节名为键的电机与传动补充参数 |

每个活动关节的字段如下。第一版支持独立旋转关节以及固定连接，不支持 mimic、移动关节或闭环机构自动转换。

| 字段 | 类型/单位 | 含义 |
| --- | --- | --- |
| `motor_model` | 字符串 | 电机型号，实际驱动兼容性另行核对 |
| `reduction_ratio` | 正数 | 电机轴速度 / 关节速度的绝对比值 |
| `direction` | 整数 +1 或 −1 | 电机到关节的正方向 |
| `zero_offset_rad` | rad | 换算到关节侧后增加的零点偏移 |
| `torque_limit_nm` | 正数，N·m | 关节输出侧力矩上限，不高于 URDF effort |
| `velocity_limit_rad_s` | 正数，rad/s | 关节输出侧速度上限，不高于 URDF velocity |

坐标约定为 `q_joint = direction * q_motor / reduction_ratio + zero_offset_rad`。若 SDK 已输出关节侧角度，不得直接套用这个电机轴公式再次换算；驱动接入须明确 SDK 的反馈坐标。原机电机零点数组不可直接冒充本 API 的关节侧零点。

客户无需重复填写 URDF 已含的质量/惯量；关节摩擦和物理阻尼也由 URDF 提供。Kp/Kd、步态策略和任务速度属于控制配置，不在本注册请求中自动调优。

## 准备请求文件

将每个关节的六项参数填写到 `joint-drives.json`，其根对象为“关节名 → 参数”。然后打包 URDF 文本和该参数表；这是接口调用示例，不需要修改系统源码：

```python
import json
from pathlib import Path

request = {
    "robot_id": "robot-a",
    "kinematics": {"urdf_xml": Path("robot.urdf").read_text()},
    "dynamics": {"joints": json.loads(Path("joint-drives.json").read_text())},
}
Path("robot-parameters.json").write_text(json.dumps(request, ensure_ascii=False))
```

URDF 的网格资源必须在后续加载环境中另行提供。本接口不下载或检查 mesh 文件，也不自动生成 MuJoCo 模型；提交的物理连杆都要求有正质量和正定惯性张量。惯量检查是基本数值检查，不代替完整物理可实现性和模型仿真验证。

## 返回结果与参数更新

成功返回 HTTP 201：`profile_id`、`robot_id`、`joint_names`、检查范围，以及 `runtime_applied: false`、`hardware_validated: false`。错误参数返回 HTTP 400 和具体原因；请求大小不符合要求返回 413；编号不存在返回 404。请求体上限 2 MiB。

参数更新：修改自己的请求文件 → 再次 POST → 保存新编号。旧编号仍可 GET，便于对照或回退。编号校验覆盖整份参数，读取时会检测保存内容是否发生改变。

程序也可直接调用 `register_robot_parameters(payload, store_dir)` 和 `load_robot_profile(profile_id, store_dir)`。配置注册接口不会发送运动指令，也不会因参数更新重启控制器。

2026-10-08 已通过 13 项配置/API 测试，包括真实本机 HTTP 提交和读取、新旧版本保留、重复提交、关节映射错误、无效惯量、超限驱动参数和配置篡改检测。原版 MEVIUS2 的 20 秒 MuJoCo 基线是独立验证，未与此 API 的 ROS 2 加载链路联调。
