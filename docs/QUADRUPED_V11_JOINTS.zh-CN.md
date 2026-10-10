# 四足机器狗 V1.1：装配 R1 与 12 轴几何标定

装配 R1 补齐前腿摇臂组件，修正右后摇臂组件，开放四腿全部 12 轴几何联动。源 STEP 与原始 GLB 不改写；可追溯实例变换及共享网格补齐记录在 `kinematics.json` 的 `assembly` 中。修正后显示 988 个实例（原 954 + 补齐 34）。

## 装配修正

| 区域 | 原问题 | R1 处理 |
|---|---|---|
| 左前 FL | 缺少 j5 摇臂与配套零件 | 将有效左后摇臂、压盖、螺栓及传动杆轴承/垫圈组件沿 CAD Z 平移 -684 mm，共补齐 17 个实例 |
| 右前 FR | 缺少 j5 摇臂与配套零件 | 复用修正后的右后组件，沿 CAD Z 平移 -684 mm，共补齐 17 个实例 |
| 左后 RL | 解析孔轴已重合 | 保留源零位，明确摇臂螺栓及杆端轴承归属 |
| 右后 RR | 摇臂电机孔偏轴 3.3441 mm，杆端销孔偏轴 43.3441 mm | 用解析圆柱面拟合刚体位姿，保持轴向位置；按有效左后组件的局部配合关系重置同几何压盖和螺栓，共修正 11 个实例 |

所有补齐件均复用源 CAD 中已有几何；没有生成新制造零件。修正矩阵、来源实例、修正后的名字与包围范围均可检查。前后连杆与驱动中心的 684 mm 平移关系通过解析孔轴验证。四腿修正后两组孔轴距离的最大数值残差小于 `4e-8 mm`；这不是制造精度。

这次解决上述驱动链的缺件、错位及运动绑定。整机紧固件 BOM 尚未补全：169 个其他紧固件/小附件仍按邻近 CAD 包围范围暂定归属。电机内部随壳体运动，不模拟齿轮/转子。源文件中两个异常相机内部几何仍按原记录排除。

## 坐标、零位和驱动映射

显示坐标为 `X 前、Y 左、Z 上`，由 `(-CAD Z,-CAD X,CAD Y)/1000` 转换，单位米；以机头为前、机器人自身左右命名。

| 每腿关节 | 几何定义 | 角度关系 |
|---|---|---|
| roll / 侧摆 | 电机/轴承座共同轴 | 按显示箭头右手定则 |
| pitch / 大腿 | 第二驱动轴 | 大腿相对侧摆架的增量 |
| knee / 膝驱动 | 第三驱动轴，与大腿轴同轴 | 摇臂相对侧摆架的增量 = 大腿增量 + 膝驱动增量 |

零位为装配 R1 的修正姿态。小腿和传动杆由两圆闭合约束求解，保持原装配分支。侧摆 ±10°、大腿与膝驱动 ±20°是预览范围，未经全行程干涉验证，不是机械限位。

“导出 12 轴标定表”输出模型版本、源文件哈希、坐标系、零位约定和逐轴记录。`motor_id`、`motor_direction`、`encoder_zero_rad`、`mechanical_limits_rad` 保持 `null`，`hardware_validated=false`。这些值须通过真机回读填写；几何零位不能冒充编码器零位。CAD 控件不发送电机命令，也不替换参数页当前控制配置。

停止、离开 CAD 页、切换后台、失焦或 Esc 会停止动画。非法角度或无闭合解会在改变任何零件前拒绝，保留上一姿态。归零回到修正后的 R1 装配。

## 验证与复现

- 8 类几何检查通过，5,292 组角度采样；连接孔轴最大数值残差约 `1.01e-11 m`。
- 直接读取原始 GLB 实例矩阵，用原始解析孔轴验证修正后实际网格变换，不仅检查抽象连杆。
- 800 个腿部实例唯一归组；完整恢复修正零位，机身固定，非法请求原子拒绝。
- 21 项浏览器检查，包括 988 实例加载、12 轴开放、实际渲染随运动变化、标定表未知值保留、停止/归零、参数草稿保留及 390px 布局。无 JS 异常、资源错误或 CAD 运动请求。
- 31 项工作台和参数 API 回归通过。Ubuntu 发布流程另行重新编译后端、运行 79+3 项编译态回归，从真实安装包新目录安装后复验 CAD 与参考参数页。
- MEVIUS2 的四个动力学场景仅作原有参考功能回归，不能代表 V1.1 实机或接触动力学验收。

```sh
python scripts/build_quadruped_kinematics.py /absolute/path/四足机器狗总装V1.1.STEP
node web/robot-workbench/build-renderer.cjs
node scripts/check_quadruped_kinematics.cjs docs/validation/quadruped-v11-motion/KINEMATICS_VALIDATION.json
WORKBENCH_ROOT=/absolute/path/to/prepared-workbench node scripts/check_quadruped_cad.cjs docs/validation/quadruped-v11-motion
python -m unittest tests.test_robot_workbench tests.test_robot_parameter_api
```

几何证据包含原始与修正后的解析圆柱轴，位于 `joint_evidence.json`。结果和预览图位于 `docs/validation/quadruped-v11-motion/`。安装包成功与否以发布资产中的 `VALIDATION.json`、`BROWSER_VALIDATION.json` 和 `RELEASE_SUMMARY.json` 为准。
