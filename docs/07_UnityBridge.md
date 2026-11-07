# 第七层:Unity桥接设计笔记

本部分包含第七层表现桥接目录 `FixedPhysics/UNITY` 

## 1. 第七层:Unity 表现桥接层

本层位于 `FixedPhysics/UNITY` 目录下, 是整个工程唯一引用 `UnityEngine` 的部分 核心原则是:定点数世界是唯一的数据源, Unity 的 `Transform` 仅作为初始数据

### 1.1 FPConvert:单向渲染转换与初始化截断
- `ToUnity`:每帧渲染阶段将 `FPVector3` / `FPQuaternion` 转为 Unity 的 `Vector3` / `Quaternion` 
- `ToFP`:仅在物体初始化(`Init`)读取 Inspector 面板参数时调用一次 通过 `ToString("F4")` 固定保留四位小数转为字符串, 再交由 `FP.FromString` 纯整数解析, 抹平不同平台 CPU 的浮点尾数差异 
- 严禁回流:游戏进入运行循环后, 严禁每帧将 Unity 的 `transform.position` 转回定点数覆盖 `FPState`, 保证定点数运算全程不受浮点数污染且零 GC 

### 1.2 FPBody 与 ColliderComponentBase:仿多碰撞盒挂载
为了兼顾Inspector 面板简洁与一物多形状组合, 组件设计对齐 Unity 原生模式:
- `FPBody`:代表物体本身(相当于 `Rigidbody`), 每个物体只挂 1 个 内部创建并持有唯一的 `FPState`, 并在 `OnEnable / OnDisable` 时自动向全局 `FPRunner` 注册或注销 
- `ColliderComponentBase`:碰撞盒形状基类, 派生出 `FPBoxCollider`、`FPSphereCollider`、`FPCapsuleCollider`、`FPConvexCollider` 四个独立脚本 每个脚本面板只显示当前形状专属的尺寸参数, 支持在同一个物体或子物体上挂载多个(挂在子物体上时自动通过 `InverseTransformPoint` 折算相对于 `FPBody` 的局部偏移), 全部绑定到同一个 `FPBody.State` 上, 并各自独立维护 `HitCount` 在 Scene 窗口绘制碰撞变红线框 

### 1.3 FPRunner 与 Player:固定步进与实战测试
- `FPRunner`:全局唯一的驱动器 在 `FixedUpdate` 中驱动 `World.Step()`, 随后遍历所有 `FPBody` 调用 `SyncToUnity()` 更新画面, 并在 `OnDrawGizmos` 中实时绘制三大粗相的空间结构线框 
- `Player`:基于新版 `InputSystem` 的测试控制器 在 `FixedUpdate` 中读取键盘与鼠标输入, 直接调用 `Body.State.Move` 与 `SetTransform` 在纯定点数层面控制移动与旋转, 并支持鼠标左键发射 `QueryRay` 激光检测目标、鼠标右键触发 `QueryAABB` 范围推挤