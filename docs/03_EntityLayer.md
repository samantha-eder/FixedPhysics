# 第三层:物理实体与组合状态层设计笔记

本层位于 `FixedPhysics/Core` 目录下, 包含 `FPState` 与 `FPCollider` 两个核心类 

## 1. 为什么要拆分为 FPState 与 FPCollider？(一物多形状问题)

在早期的单碰撞盒设计中, 如果让每个 `FPCollider` 各自独立持有一份值类型的 `FPTransform`(世界坐标与旋转), 在面对“同一个角色挂载多个碰撞盒”(例如:角色身体是一个胶囊体, 手里举着一个方盒盾牌, 或者由头、身体、四肢拼接而成的复合角色)时, 会出现两个严重问题:
1. 双倍推挤问题: 如果使用delta累加变化量到FPTransform上来处理多碰撞盒问题时, 如果角色的身体和盾牌在同一帧同时撞上一面平墙(比如都穿透了 0.2 米), 物理求解器会先把身体推退 0.2 米, 紧接着检查盾牌时, 由于盾牌保存的是自己独立的一份坐标, 它不知道角色已经被推退了, 求解器会再把盾牌推退 0.2 米 如果最后在表现层把各个部件的位移差值累加到角色身上, 角色就会被错误地推退 0.4 米 
2. 内部部件错位:在单帧多轮迭代求解(`SolverCount > 1`)的过程中, 某一个子碰撞盒被推开后, 同一个物体上的其他兄弟碰撞盒没有实时跟着移动并刷新包围盒, 导致角色各部位在求解过程中发生相对错位 

为了从根源上解决这个问题, 架构上严格对齐了商业物理引擎(如 Unity PhysX 中 `Rigidbody` 与 `Collider` 的关系), 将“物体的状态”与“碰撞盒形状”拆分为 `FPState` 和 `FPCollider` 

---

## 2. FPState:物体的唯一世界坐标源

一个游戏实体(无论身上挂了 1 个还是 10 个碰撞盒)在物理世界中只对应唯一的一个 `FPState` 实例:
- `Transform`:保存该物体在世界空间下唯一的权威位置与旋转 
- `IsStatic`:标记该物体是否为静态不可移动物体(动静属性属于整个物体, 不属于单个局部形状) 
- `FreezeX / FreezeY / FreezeZ`:仿照 Unity 的轴向约束开关 在 `Move(delta)` 时, 被锁定的坐标轴位移分量会被直接置为 `0`(例如在 2.5D 平面测试中勾选 `FreezeY`, 可防止多物体在墙角剧烈挤压时沿斜面法线的 Y 分量逃逸到空中或地下) 
- `Colliders`:记录挂载在该物体名下的所有 `FPCollider` 部件列表 

联动机制:
1. 当调用 `FPState.SetTransform` 修改物体姿态(可能包含旋转改变)时, 会遍历 `Colliders` 列表调用 `UpdateBounds()`, 重新通过四元数旋转计算各部件的世界 AABB 
2. 平移包围盒加速:当调用 `FPState.Move(delta)` 仅发生平移(未改变旋转四元数)时, 根据欧氏几何中 AABB 的平移不变性, 无需重新调用 `Shape.ComputeBounds`(避免 42 顶点的圆柱凸包或旋转方盒每帧重复执行上万次四元数乘法), 而是直接对名下所有 `Colliders[i].Bounds` 的 `Min` 和 `Max` 加上位移向量 `move` 

---

## 3. FPCollider:挂载在状态下的具体碰撞部件

`FPCollider` 代表参与空间划分和碰撞检测的具体碰撞盒, 在构造时强制要求传入所属的 `FPState`:
- `ID`:每个碰撞盒拥有全局唯一的自增无符号整数编号(从 `10001` 开始), 供第五层生成 64 位 `ColliderPair` 唯一键值、去重集合以及第六层确定性排序使用 
- `State`:指向所属的 `FPState` `FPCollider` 自身不再保存世界坐标, 其 `Move` 与 `SetTransform` 方法直接转发给 `State` 
- `IsTrigger`:标记是否为触发器
- `OffsetPosition` 与 `OffsetRotation`:该碰撞盒相对于 `FPState` 中心的局部位置偏移与局部旋转偏移 
- `Bounds`:该碰撞盒在世界空间下的独立 AABB 包围盒(每个子部件维护各自紧凑的小包围盒, 避免合成一个巨大的空心包围盒浪费粗相网格) 

---

## 4. 局部空间到世界空间的合成 (GetShapeWorldTransform)

在整个引擎流水线中, 局部偏移(`OffsetPosition`、`OffsetRotation`)只在 `FPCollider.GetShapeWorldTransform()` 这一个方法内被处理:
- 世界位置:`worldPos = State.Transform.ToWorldPos(OffsetPosition)`(先按物体朝向旋转局部偏移向量, 再加上物体世界坐标) 
- 世界旋转:`worldRot = State.Transform.Rotation * OffsetRotation`(物体世界旋转乘上碰撞盒局部旋转) 

合成出的 `FPTransform` 即为该碰撞形状在世界空间下的最终绝对姿态 将其传给第二层的 `Shape.ComputeBounds` 和第四层的 `PhysicsMath / GJKEPA` 后, 后续所有层级全部在统一的世界坐标系下运算, 无需再关心局部偏移细节 

## 5. FPState FPCollider成员变量安排说明

核心逻辑是为了解耦与扩展性
`IsTrigger` 设计在单个 `FPCollider` 级别, 这样同一个角色身上的“身体胶囊”可以设为实心阻挡(`IsTrigger = false`), 而手里的“武器攻击框”可以设为幽灵触发器(`IsTrigger = true`)
`IsStatic` 属于物体的属性而不属于碰撞箱Collider