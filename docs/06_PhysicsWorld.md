# 第六层:物理世界求解器设计笔记

本部分包含第六层核心控制层 `FixedPhysics/PhysicsWorld.cs`

## 1. PhysicsWorld 物理世界与求解器

`PhysicsWorld` 是串联前五层组件的调度核心, 负责管理碰撞体生命周期、驱动每帧的碰撞流水线、执行位置推挤求解、分发碰撞事件以及提供精确空间查询 

### 1.1 单帧主循环 (Step) 与高斯-赛德尔迭代求解
每一物理帧调用 `Step()` 时, 按以下顺序执行:
1. 刷新包围盒:遍历所有激活的 `Colliders` 更新其世界 AABB 
2. 多轮迭代求解(`for t = 1..SolverCount`):
   为什么需要支持多轮迭代？
   当只有两个物体相撞时, 1 次推挤即可完全分开 但当三个或更多物体在墙角挤压时(例如:墙壁 W — 物体 A — 物体 B), 第 1 轮推开 `(A, B)` 可能会导致 A 穿入墙壁 W；紧接着墙壁 W 把 A 顶回时, A 又会重新压入 B 体内 
   因此在每轮迭代内部都会重新执行 `BroadPhase.Update` 与 `GetPotentialPairs`(确保前一轮推开后新产生的接触对如 `(B, C)` 能被下一轮立即捕获), 通过 2 ~ 3 轮快速位置匀平, 即可彻底消除死角多物体穿模 
   (注:触发器 `IsTrigger` 不参与物理位置推挤, 仅在第 1 轮迭代时做窄相检测并记录命中事件, 后续迭代直接跳过) 
3. 帧末索引同步:全部迭代推挤完成后, 再次调用 `BroadPhase.Update(Colliders)`, 确保帧末立即执行射线或区域查询时使用的是推挤后的最终位置 

### 1.2 跨粗相算法的 100% 确定性保证 (PotentialPairs.Sort)
`SpatialHash`、`Octree`、`BVH` 底层遍历网格和树节点的顺序不同, 导致 `GetPotentialPairs` 收集到的候选对列表顺序不同 
在多物体连续推挤中, 先推 `(A, B)` 再推 `(B, C)`, 与先推 `(B, C)` 再推 `(A, B)` 得出的最终坐标存在差异 
为了保证无论切换哪一种粗相算法, 物理世界算出的推挤轨迹都 100% 相同, 在每轮获取 `PotentialPairs` 后直接调用一次 `PotentialPairs.Sort()`, 按 64 位 `PairKey` 从小到大固定求解顺序 

### 1.3 窄相分发 (NarrowPhase) 与位置推挤 (Resolve)
- `NarrowPhase`:利用 C# 元组模式匹配 `switch (a.Shape.Type, b.Shape.Type)` 分发至第四层 其中 `Box(A) vs Sphere(B)` 调换顺序复用 `Sphere vs Box` 解析解后, 将返回的 `Normal` 取反, 严格保证所有分支返回的推挤法线方向永远由 `a` 指向 `b` 
- `Resolve`:沿 `separation = Normal * Penetration` 将穿透物体推开:
  - `a` 静态、`b` 动态:`b.Move(+separation)`
  - `b` 静态、`a` 动态:`a.Move(-separation)`
  - 双方均为动态:各承担一半位移 `a.Move(-half); b.Move(+half);`
  当角色斜向撞击静态墙壁时, 求解器仅沿墙壁法线抵消穿入墙内的分量, 平行于墙面的移动分量天然保留, 从而自动实现丝滑的贴墙滑动效果
注: 本套框架目前尚未实现质量 摩擦 积分等模块 因此采用对半推开的简单设计原则

### 1.4 双集合乒乓交换与安全事件分发 (DispatchEvents)
为了零 GC 实现 `OnCollisionEnter`、`OnCollisionStay`、`OnCollisionExit` 状态机, 内部维护 `_preHits`(上一帧命中集合)与 `_curHits`(当前帧命中集合):
- 当前帧有、上一帧无 -> 放入 `_enterBuffer`
- 当前帧有、上一帧也有 -> 放入 `_stayBuffer`
- 上一帧有、当前帧无 -> 放入 `_exitBuffer`

安全回调原则:先将三个缓冲区排序, 并完成 `_preHits` 与 `_curHits` 的引用交换, 最后再遍历缓冲区触发外部回调 这样即使外部逻辑在 `OnCollisionEnter` 回调中立即调用了 `world.Clear()` 或删除物体, 也不会破坏正在遍历的集合或丢失本帧剩余的碰撞通知 

### 1.5 粗筛与窄相合一的精确空间查询 (QueryRay / QueryAABB)
对外仅暴露两个统一的精确查询入口:
- `QueryRay`:先调用 `BroadPhase.QueryRay` 粗筛出沿途 AABB 被穿过的候选人, 再依次调用 `PhysicsMath.Raycast` / `RaycastAny` 对具体几何形状求交(剔除仅擦过球体/旋转盒 AABB 空白角落的假命中), 封装为 `RayHit` 并按命中距离 `Length` 从小到大排序输出(`results[0]` 即为最近命中目标) 
- `QueryAABB`:先通过粗相捞出 AABB 重叠的候选人, 再将查询框构造为标准 `FPBox`, 原地倒序调用 `NarrowPhase` 剔除未接触真实几何体的对象 