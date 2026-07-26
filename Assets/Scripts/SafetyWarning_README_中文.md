# Unity 头显安全警示界面说明

## 功能效果

运行 `SampleScene` 后，系统会在头显/主摄像机前方生成一个安全警示 UI：

- 自动模拟机械臂周围环境物体的平均温度。
- 在头显画面中框选对应物体。
- 框线使用绿色、黄色、红色表示风险等级。
- 标签显示区域名称和平均温度。
- 右上角显示颜色与温度范围说明。

重要：该功能不会把物体本身改成红色、黄色或绿色。颜色只显示在头显 UI 框和文字上，符合“框选对应物体并加以颜色预警”的要求。

## 温度颜色规则

| 平均温度 | UI 颜色 | 含义 |
|---|---|---|
| T < 40 C | 绿色 | Safe / 正常移动 |
| 40-60 C | 黄色 | Caution / 显示警告 |
| 60-80 C | 红色 | Danger / 阻止继续深入危险区 |
| T >= 80 C | 闪烁红色 | Critical / 停止或急停 |

代码中实际判断为：

- `< 40 C`：Safe / 绿色
- `40-60 C`：Caution / 黄色
- `60-80 C`：Danger / 红色
- `>= 80 C`：Critical / 闪烁红色

## 使用方法

1. 打开 Unity 工程 `piper-unity-ros-sim`。
2. 打开 `Assets/Scenes/SampleScene.unity`。
3. 点击 Play。
4. 场景中会自动创建 `Safety Warning Runtime`。
5. 如果场景里还没有手动设置的 `SafetyTemperatureZone`，系统会在机械臂周围生成几个灰色模拟环境物体，并在头显 UI 中框选。

## 如何给真实物体添加温度区域

如果你想标注某个已有物体：

1. 在 Hierarchy 里选中该物体。
2. Add Component。
3. 添加 `SafetyTemperatureZone`。
4. 设置 `Zone Name`。
5. 设置 `Base Temperature C`，例如：
   - 25：绿色区域
   - 40：黄色区域
   - 55：红色区域
6. 再次 Play。

只要场景里存在 `SafetyTemperatureZone`，系统就会自动在头显 UI 里给这些物体画框。

## 主要脚本

- `SafetyTemperatureZone.cs`：模拟单个区域的平均温度并判断风险等级。
- `HeadsetSafetyWarningOverlay.cs`：把物体边界投影到头显 UI 上，并绘制框线、文字和右上角图例。
- `SafetyWarningDemoBootstrapper.cs`：运行时自动启用安全警示系统；没有手动区域时创建演示物体。

## 向老师解释的说法

本功能模拟机械臂周围不同环境物体的温度，并根据平均温度划分风险等级。系统没有直接改变物体材质颜色，而是在头显界面中根据摄像机视角对物体进行框选，并用红、黄、绿三色 UI 框进行实时预警。右上角图例持续显示温度范围与颜色对应关系，便于操作者在 VR/头显视角中快速理解当前风险状态。
