using FixedMath;
using System;
using System.Collections.Generic;

namespace FixedPhysics
{
    public class PhysicsWorld
    {
        //位置推挤求解次数
        public int SolverCount;

        //粗相空间划分
        public IBroadPhase BroadPhase;

        //世界空间中所有激活的碰撞体列表
        public HashSet<uint> Set;
        public List<FPCollider> Colliders;


        //粗相空间提供的潜在碰撞对
        public List<ColliderPair> PotentialPairs;

        //碰撞与触发器生命周期事件回调
        public Action<ColliderPair> OnCollisionEnter;
        public Action<ColliderPair> OnCollisionStay;
        public Action<ColliderPair> OnCollisionExit;

        //碰撞生命周期事件状态机集合
        private HashSet<ColliderPair> _preHits;
        private HashSet<ColliderPair> _curHits;

        private List<FPCollider> _queryBuffer;
        private List<ColliderPair> _enterBuffer;
        private List<ColliderPair> _stayBuffer;
        private List<ColliderPair> _exitBuffer;

        //碰撞检测
        public GJKEPA GJK_EPA;

        public PhysicsWorld(IBroadPhase phase, int count = 2, int capacity = 2048)
        {
            BroadPhase = phase;
            SolverCount = count;

            GJK_EPA = new GJKEPA();

            Set = new HashSet<uint>(capacity);
            Colliders = new List<FPCollider>(capacity);

            PotentialPairs = new List<ColliderPair>(capacity << 1);

            _preHits = new HashSet<ColliderPair>(256);
            _curHits = new HashSet<ColliderPair>(256);

            _queryBuffer = new List<FPCollider>(256);
            _enterBuffer = new List<ColliderPair>(256);
            _stayBuffer = new List<ColliderPair>(256);
            _exitBuffer = new List<ColliderPair>(256);
        }

        //在Step()之外手动移动了物体并希望立即执行空间查询时
        //调用此方法刷新一次空间索引
        public void RebuildBroadPhase()
        {
            foreach (FPCollider item in Colliders)
            {
                item.UpdateBounds();
            }
            BroadPhase.Update(Colliders);
        }

        //碰撞查询
        public void QueryAABB(FPBounds bounds, List<FPCollider> results)
        {
            BroadPhase.QueryAABB(bounds, results);

            //如果粗筛结果为null 则return
            if (results.Count == 0)
                return;

            //将查询框作为一个标准 FPBox 调用 NarrowPhase 细筛
            FPBox boxA = new FPBox(bounds.Extents);
            FPTransform transA = new FPTransform(bounds.Center, FPQuaternion.Identity);

            for (int i = results.Count - 1; i >= 0; i--)
            {
                FPCollider b = results[i];
                FPShape shapeB = b.Shape;
                FPTransform transB = b.GetShapeWorldTransform();

                bool isHit = b.Shape.Type switch
                {
                    ShapeType.Sphere => PhysicsMath.Intersect((FPSphere)shapeB, transB, boxA, transA).IsHit,
                    ShapeType.Box => PhysicsMath.Intersect(boxA, transA, (FPBox)shapeB, transB).IsHit,
                    _ => PhysicsMath.IntersectAny(GJK_EPA, boxA, transA, shapeB, transB).IsHit
                };

                if (!isHit)
                {
                    results.RemoveAt(i);
                }
            }
        }
        public void QueryRay(FPVector3 origin, FPVector3 direction, FP maxDistance, List<RayHit> results)
        {
            results.Clear();
            if (maxDistance < FP.Zero || direction == FPVector3.Zero)
                return;

            direction = FPVector3.Normalize(direction);

            //调用粗相筛选碰撞体存入_queryBuffer
            BroadPhase.QueryRay(origin, direction, maxDistance, _queryBuffer);
            if (_queryBuffer.Count == 0)
                return;

            //执行窄相精确射线求交装入results
            foreach (FPCollider collider in _queryBuffer)
            {
                if (RaycastSingle(origin, direction, maxDistance, collider, out FP length, out FPVector3 normal))
                {
                    results.Add(new RayHit(collider, length, origin + direction * length, normal));
                }
            }

            //按命中距离 Length 从小到大排序
            results.Sort();
        }
        private bool RaycastSingle(
            FPVector3 origin, FPVector3 direction, FP maxDistance,
            FPCollider collider,
            out FP length, out FPVector3 normal)
        {
            FPTransform transform = collider.GetShapeWorldTransform();

            switch (collider.Shape.Type)
            {
                case ShapeType.Sphere:
                    return PhysicsMath.Raycast(origin, direction, maxDistance, (FPSphere)collider.Shape, transform, out length, out normal);

                case ShapeType.Box:
                    return PhysicsMath.Raycast(origin, direction, maxDistance, (FPBox)collider.Shape, transform, out length, out normal);

                default:
                    return PhysicsMath.RaycastAny(GJK_EPA, origin, direction, maxDistance, collider.Shape, transform, out length, out normal);
            }
        }

        public void Draw()
        {
            BroadPhase.Draw();
        }

        //碰撞体生命周期管理
        public void Add(FPCollider collider)
        {
            if (collider == null)
                return;

            //如果碰撞体已经存在则跳过
            if (Set.Add(collider.ID))
            {
                collider.UpdateBounds();

                //添加进粗相碰撞管理空间中
                Colliders.Add(collider);
                BroadPhase.Add(collider);
            }
        }
        public void Remove(FPCollider collider)
        {
            if (collider == null)
                return;

            if (Set.Remove(collider.ID))
            {
                Colliders.Remove(collider);
                BroadPhase.Remove(collider);
            }
        }
        public void Clear()
        {
            BroadPhase.Clear();

            //清空粗相空间
            Set.Clear();
            Colliders.Clear();

            //清空潜在碰撞对
            PotentialPairs.Clear();

            //清空碰撞集合
            _preHits.Clear();
            _curHits.Clear();
        }

        //切换粗相空间划分算法
        public void SetBroadPhase(IBroadPhase phase)
        {
            if (phase == null || phase == BroadPhase)
                return;

            BroadPhase = phase;
            BroadPhase.Clear();
            BroadPhase.Update(Colliders);
        }

        //每帧调用
        //物理世界主循环驱动
        public void Step()
        {
            //计算精确碰撞并收集
            _curHits.Clear();

            //首次进入迭代前 更新粗相空间索引
            BroadPhase.Update(Colliders);

            //每轮更新粗相并按PairKey排序
            //确保推开产生的新接触被检测 且三大粗相结果一致
            BroadPhase.GetPotentialPairs(PotentialPairs);
            PotentialPairs.Sort();

            //多轮迭代求解
            for (int t = 1; t <= SolverCount; t++)
            {
                bool isResolved = false;

                //遍历所有潜在碰撞对
                foreach (ColliderPair pair in PotentialPairs)
                {
                    FPCollider a = pair.A;
                    FPCollider b = pair.B;

                    //对于共享同一个FPState
                    //属于同一个物体的直接跳过自碰撞
                    if (a.State == b.State)
                        continue;

                    //静态物体不做检测
                    if (a.State.IsStatic && b.State.IsStatic)
                        continue;

                    //触发器不参与物理位置推挤
                    bool trigger = a.IsTrigger || b.IsTrigger;
                    if (t > 1 && trigger)
                        continue;

                    if (t > 1 && !a.Bounds.Intersects(b.Bounds))
                        continue;

                    //精确检测
                    CollisionManifold manifold = NarrowPhase(a, b);
                    if (manifold.IsHit)
                    {
                        //如果相撞则记录此帧发生碰撞的碰撞对
                        _curHits.Add(pair);

                        //触发器只记录碰撞不发生物理推挤
                        if (!trigger && manifold.Penetration > FP.Zero)
                        {
                            Resolve(a, b, manifold);
                            isResolved = true;
                        }
                    }
                }

                //若本轮已无任何穿透推挤
                //提前结束迭代
                if (!isResolved)
                    break;
            }
            //分发碰撞生命周期事件
            DispatchEvents();
        }

        //分发碰撞生命周期事件
        private void DispatchEvents()
        {
            _enterBuffer.Clear();
            _stayBuffer.Clear();
            _exitBuffer.Clear();

            //检查 Enter 与 Stay
            foreach (ColliderPair pair in _curHits)
            {
                if (_preHits.Contains(pair))
                {
                    _stayBuffer.Add(pair);
                }
                else
                {
                    _enterBuffer.Add(pair);
                }
            }

            //检查 Exit
            foreach (ColliderPair pair in _preHits)
            {
                if (!_curHits.Contains(pair))
                {
                    _exitBuffer.Add(pair);
                }
            }

            //交换引用 准备下一帧
            HashSet<ColliderPair> temp = _preHits;
            _preHits = _curHits;
            _curHits = temp;

            //按确定性顺序排序后依次触发回调
            _enterBuffer.Sort();
            _stayBuffer.Sort();
            _exitBuffer.Sort();

            foreach (var item in _enterBuffer) OnCollisionEnter?.Invoke(item);
            foreach (var item in _stayBuffer) OnCollisionStay?.Invoke(item);
            foreach (var item in _exitBuffer) OnCollisionExit?.Invoke(item);
        }

        //保证推挤方向由a->b
        //根据两个碰撞体的Shape 通过PhysicsMath计算穿透深度与法线
        private CollisionManifold NarrowPhase(FPCollider a, FPCollider b)
        {
            FPTransform transA = a.GetShapeWorldTransform();
            FPTransform transB = b.GetShapeWorldTransform();

            switch (a.Shape.Type, b.Shape.Type)
            {
                case (ShapeType.Sphere, ShapeType.Sphere):
                    return PhysicsMath.Intersect((FPSphere)a.Shape, transA, (FPSphere)b.Shape, transB);

                case (ShapeType.Box, ShapeType.Box):
                    return PhysicsMath.Intersect((FPBox)a.Shape, transA, (FPBox)b.Shape, transB);

                case (ShapeType.Sphere, ShapeType.Box):
                    return PhysicsMath.Intersect((FPSphere)a.Shape, transA, (FPBox)b.Shape, transB);

                case (ShapeType.Box, ShapeType.Sphere):
                    CollisionManifold m = PhysicsMath.Intersect((FPSphere)b.Shape, transB, (FPBox)a.Shape, transA);
                    m.Normal = -m.Normal;
                    return m;

                default:
                    //任意凸包全部自动走 GJK+EPA
                    return PhysicsMath.IntersectAny(GJK_EPA, a.Shape, transA, b.Shape, transB);
            }
        }

        //位置推挤求解器
        //根据manifold将两个碰撞体推开
        private void Resolve(FPCollider a, FPCollider b, CollisionManifold manifold)
        {
            if (manifold.Penetration <= FP.Zero)
                return;

            //推挤向量
            FPVector3 separation = manifold.Normal * manifold.Penetration;

            //a是静态墙体 b是动态物体
            if (a.State.IsStatic && !b.State.IsStatic)
            {
                b.Move(separation);
            }
            //b是静态墙体 a是动态物体
            else if (!a.State.IsStatic && b.State.IsStatic)
            {
                a.Move(-separation);
            }
            //a和b都是动态物体
            else if (!a.State.IsStatic && !b.State.IsStatic)
            {
                FPVector3 half = separation * FP.Half;
                a.Move(-half);
                b.Move(half);
            }
        }
    }
}