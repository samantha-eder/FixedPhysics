# 第六层:物理世界求解器设计笔记

本部分包含第六层核心控制层 `FixedPhysics/PhysicsWorld.cs`

## 1. PhysicsWorld 物理世界与求解器

`PhysicsWorld` 是串联前五层组件的调度核心, 负责管理碰撞体生命周期、驱动每帧的碰撞流水线、执行位置推挤求解、分发碰撞事件以及提供精确空间查询 

### 1.1 单帧主循环 (Step) 与多轮迭代求解的工程取舍
每一物理帧调用 `Step()` 时, 按以下顺序执行:
1. 单次粗相过筛:`BroadPhase.Update(Colliders)` 与 `BroadPhase.GetPotentialPairs(PotentialPairs)`, 并按 `PairKey` 排序 
2. 多轮窄相迭代求解(`for t = 1..SolverCount`):
   针对粗筛出的 `PotentialPairs` 名单进行多轮窄相检测与 `Resolve` 推挤；若某轮没有任何实心物体发生穿透推挤(`!isResolved`), 则提前 `break` 结束迭代 

核心思考与工程取舍:为什么 `BroadPhase.Update` 放在 `SolverCount` 循环外部(每帧只执行 1 次), 而不是每轮迭代都重建粗相？
这里区分两种多体碰撞场景:
- 场景 A(墙角多物体堆叠挤压, 已在名单内):当墙壁 W、怪物 B、怪物 A 三个物体已经挤在一起时, `(B, A)` 与 `(W, B)` 本来就同时存在于本帧的 `PotentialPairs` 名单中 第 1 轮(`t = 1`)先推开 `(B, A)` 会导致 B 穿入墙壁 W, 随后墙壁 W 把 B 顶回又导致 B 重新压入 A 体内 此时无需重新运行粗相, 第 2 轮(`t = 2`)继续遍历同一份 `PotentialPairs` 名单即可再次处理 `(B, A)`, 将墙壁的阻挡力通过 B 完整传导给 A, 消除残余穿模 
- 场景 B(跨空隙的二次碰撞, 未在初始名单内):若帧初 B 与墙壁 W 之间还隔着一丝空气缝隙(未接触), 本帧 A 撞击 B 使 B 跨过缝隙穿入墙内 若要在同一帧内捕获 `(B, W)`, 就必须在每轮迭代内重新调用 `BroadPhase.Update` 全量重建空间结构 但在 1000 个移动物体的压力测试中, 每帧重建 3 次粗相会使粗相耗时增加 200% 
  遵循 Box2D 与 PhysX 等商业引擎的标准做法, 本框架选择每帧仅执行 1 次粗相:跨空隙新产生的二次接触会在 `0.02` 秒后的下一物理帧开头立即被粗相捕获并推开(人眼完全不可见), 从而换取粗相阶段 3 倍的性能提升(1000 个三维球体下空间哈希单步仅需 `0.50ms`) 

### 1.2 跨粗相算法的 100% 确定性保证 (PotentialPairs.Sort)
`SpatialHash`、`Octree`、`BVH` 底层遍历网格和树节点的顺序不同, 导致 `GetPotentialPairs` 收集到的候选对列表顺序不同 
在多物体连续推挤中, 先推 `(A, B)` 再推 `(B, C)`, 与先推 `(B, C)` 再推 `(A, B)` 得出的最终坐标存在差异 
为了保证无论切换哪一种粗相算法, 物理世界算出的推挤轨迹都 100% 相同, 在获取 `PotentialPairs` 后直接调用一次 `PotentialPairs.Sort()`, 按 64 位 `PairKey` 从小到大固定求解顺序 

### 1.3 窄相分发 (NarrowPhase) 与位置推挤 (Resolve)
- `NarrowPhase`:利用 C# 元组模式匹配 `switch (a.Shape.Type, b.Shape.Type)` 分发至第四层 其中 `Box(A) vs Sphere(B)` 调换顺序复用 `Sphere vs Box` 解析解后, 将返回的 `Normal` 取反, 严格保证所有分支返回的推挤法线方向永远由 `a` 指向 `b` 
- `Resolve`:沿 `separation = Normal * Penetration` 将穿透物体推开:
  - `a` 静态、`b` 动态:`b.Move(+separation)`
  - `b` 静态、`a` 动态:`a.Move(-separation)`
  - 双方均为动态:各承担一半位移 `a.Move(-half); b.Move(+half);`
  当角色斜向撞击静态墙壁时, 求解器仅沿墙壁法线抵消穿入墙内的分量, 平行于墙面的移动分量天然保留, 从而自动实现丝滑的贴墙滑动效果 

### 1.4 双集合乒乓交换与安全事件分发 (DispatchEvents)
为了零 GC 实现 `OnCollisionEnter`、`OnCollisionStay`、`OnCollisionExit` 状态机, 内部维护 `_preHits`(上一帧命中集合)与 `_curHits`(当前帧命中集合):
- 当前帧有、上一帧无 -> 放入 `_enterBuffer`
- 当前帧有、上一帧也有 -> 放入 `_stayBuffer`
- 上一帧有、当前帧无 -> 放入 `_exitBuffer`

安全回调原则:先将三个缓冲区按 `PairKey` 排序, 并完成 `_preHits` 与 `_curHits` 的引用交换, 最后再遍历缓冲区触发外部回调 这样即使外部逻辑在 `OnCollisionEnter` 回调中立即调用了 `world.Clear()` 或删除物体, 也不会破坏正在遍历的集合或丢失本帧剩余的碰撞通知 

### 1.5 粗筛与窄相合一的精确空间查询 (QueryRay / QueryAABB)
对外暴露两个统一的精确查询入口:
- `QueryRay`:先调用 `BroadPhase.QueryRay` 粗筛出沿途 AABB 被穿过的候选人存入 `_queryBuffer`, 再依次调用 `RaycastSingle` 对具体几何形状求交(剔除仅擦过球体/旋转盒 AABB 空白角落的假命中), 封装为 `RayHit` 并按命中距离 `Length` 从小到大排序输出(`results[0]` 即为最近命中目标) 
- `QueryAABB`:先通过粗相捞出外层 AABB 重叠的候选人, 再将查询框 `bounds` 构造为标准 `FPBox`, 原地倒序调用 `PhysicsMath.Intersect` / `IntersectAny` 细筛剔除未接触真实几何体的对象 