# 四足机器狗 V1.1 CAD 工作台接入

本次把用户的《四足机器狗总装V1.1.STEP》作为静态装配模型接入现有四足工作台。四足版打开后默认显示自己的 V1.1 CAD，可旋转、缩放、切换视角、修改显示配色和点击选择原始零件。

“关节参数调试”保留原工作台及其当前 URDF 配置。两个视图在同一页切换，编辑器保持挂载，未保存草稿不会因为切换丢失。离开参数页会停止本地预览；CAD 视图不写入当前控制配置，也不发出关节、导航或规划运动请求。

CAD 保存姿态尚未与 12 关节运动链绑定。现有参考模型的关节动作、动力学结果不能作为这台 V1.1 机器狗的运动验收。接入运动前，需核对各腿三根转轴、父子连杆、零位、正方向及限位；接触动力学还需要质量、惯量和足端接触参数。

## 来源与显示资源

- 唯一源文件：用户上传的四足机器狗总装 V1.1 STEP，原件不修改。
- SHA-256：`e655d503882cb58208b053457ed11bf7cdbc90d1f476141a0e9e8f02b1ac4d34`。
- 文件：`config/robot_models/originmind_quadruped_v11/assembly.glb` 及 `manifest.json`。
- 以 OpenCASCADE 读取装配实例与变换；保留零件名称和共享几何。CAD 坐标转换为显示米制坐标：`(-Z, -X, Y) / 1000`。
- 网格离散参数：线性偏差 0.6、角偏差 0.35 rad；合并重复顶点后以二次误差方法简化。每件依次尝试保留 12%、25%、50% 面数，若包围范围变化超过 0.5 mm 则保留更高精度，必要时保留原网格。网格用于显示，不用于制造尺寸验收或精确干涉计算。
- 显示配色是工作台材质，不代表材料、表面处理或加工规范。
- 异常源几何的名称和包围范围在 `manifest.json` 中逐项记录，并在页面说明；不会补画或伪造遗漏几何。

## 开发复现

在私有开发仓使用 CadQuery 2.7.0 / OpenCASCADE、NumPy、trimesh、fast-simplification：

```sh
python scripts/build_quadruped_cad.py /absolute/path/四足机器狗总装V1.1.STEP
cd web/robot-workbench
npm install --ignore-scripts --no-audit --no-fund
node build-renderer.cjs
```

转换在 STEP 旁生成 `.geometry.xbf` 中间缓存。修改转换逻辑或更换源文件时应重新生成缓存；源文件 SHA 校验不匹配会拒绝继续。CAD 工具只用于开发转换，不加入客户运行环境。

`release/workbench/quadruped_variant.py` 在准备四足包时设置页面模式并复制 CAD 资源；共同模板仍服务机械臂版，机械臂版不显示这个四足入口。资源由后端固定白名单路径提供，不支持从导入 URDF 任意读取文件或访问远端地址。

当前为源码开发预览，Ubuntu 完整安装包尚未更新；没有真机、关节运动或 V1.1 接触动力学验收。

## 本轮验证

- 954 个显示实例、143 个共享网格，约 104 万实例三角面；GLB 5,750,684 字节。
- 每件简化前后包围范围最大偏差 0.4874 mm；这只是显示网格的保形检查。
- 25 项既有工作台单元/接口测试通过。
- Chromium 153 / SwiftShader 实际 WebGL 页面通过 12 项交互检查；零 JavaScript 异常、零意外资源请求失败。包含三维外廓回读、四视角、配色、旋转缩放、零件点击、草稿往返、参数写入、控制配置不被 CAD 替换以及 390px 横向溢出检查。
- 记录：`docs/validation/quadruped-v11-cad/BROWSER_VALIDATION.json`。本轮未测实体 GPU 帧率、Ubuntu 完整安装包或真机。
