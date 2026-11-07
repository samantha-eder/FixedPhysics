using FixedMath;
using System;
using System.Collections.Generic;

namespace FixedPhysics
{
    public class SpatialHash : IBroadPhase
    {
        public readonly FP CellSize;
        public readonly int HashMask;

        //链式前向星CFS建图
        private int _count;
        private int[] _head;
        private int[] _next;
        private FPCollider[] _to;

        //脏数据表 记录所有桶编号
        private int _dirtyCount;
        private int[] _dirty;

        //去重集合
        public readonly HashSet<uint> QuerySet;
        public readonly HashSet<ulong> PairSet;


        public SpatialHash(FP size, int shift = 16, int capacity = 4096)
        {
            CellSize = size;

            //默认 1<<16 = 65536 个桶 Mask = 65535
            int bucket = 1 << shift;
            _dirtyCount = 0;
            _dirty = new int[bucket];

            //初始化桶
            HashMask = bucket - 1;
            _head = new int[bucket];

            //初始化链式前向星CFS
            _next = new int[capacity + 1];
            _to = new FPCollider[capacity + 1];
            _count = 1;

            QuerySet = new HashSet<uint>(256);
            PairSet = new HashSet<ulong>(2048);
        }

        //坐标转换与哈希计算
        private void GetCellCoord(FPVector3 pos, out int x, out int y, out int z)
        {
            x = AxisToCell(pos.x);
            y = AxisToCell(pos.y);
            z = AxisToCell(pos.z);
        }
        //计算单轴坐标所在的网格编号
        private int AxisToCell(FP value)
        {
            long a = value.Value;
            long b = CellSize.Value;

            long q = a / b;
            long r = a % b;
            return (int)((r < 0) ? (q - 1) : q);
        }
        private int GetIndex(int x, int y, int z)
        {
            unchecked
            {
                int hash = 17;
                hash = hash * 23 + (x ^ 73856093);
                hash = hash * 23 + (y ^ 19349663);
                hash = hash * 23 + (z ^ 83492791);
                hash ^= (hash >> 16);

                return hash & HashMask;
            }
        }

        //链式前向星建图
        private void InsertCollider(FPCollider collider)
        {
            FPVector3 min = collider.Bounds.Min;
            FPVector3 max = collider.Bounds.Max;

            GetCellCoord(min, out int min_x, out int min_y, out int min_z);
            GetCellCoord(max, out int max_x, out int max_y, out int max_z);

            for (int x = min_x; x <= max_x; x++)
            {
                for (int y = min_y; y <= max_y; y++)
                {
                    for (int z = min_z; z <= max_z; z++)
                    {
                        int index = GetIndex(x, y, z);
                        AddEdge(index, collider);
                    }
                }
            }
        }
        private void AddEdge(int u, FPCollider collider)
        {
            // 扩容保护
            if (_count >= _next.Length)
            {
                int n = _next.Length << 1;
                Array.Resize(ref _next, n);
                Array.Resize(ref _to, n);
            }
            //添加进垃圾桶中
            if (_head[u] == 0)
            {
                _dirty[_dirtyCount] = u;
                _dirtyCount++;
            }
            //链式前向星加边
            _next[_count] = _head[u];
            _to[_count] = collider;
            _head[u] = _count;
            _count++;
        }
        private void ClearGraph()
        {
            for (int i = 0; i < _dirtyCount; i++)
            {
                _head[_dirty[i]] = 0;
            }
            _count = 1;
            _dirtyCount = 0;
        }

        //IBroadPhase生命周期接口
        public void Add(FPCollider collider)
        {
            InsertCollider(collider);
        }
        public void Remove(FPCollider collider)
        {

            //将链表中指向该collider的节点置空避免整图重建
            for (int e = 1; e < _count; e++)
            {
                if (_to[e] == collider)
                {
                    _to[e] = null;
                }
            }

        }
        public void Clear()
        {
            ClearGraph();
        }
        public void Update(IReadOnlyList<FPCollider> colliders)
        {
            ClearGraph();
            for (int i = 0; i < colliders.Count; i++)
            {
                InsertCollider(colliders[i]);
            }
        }

        //收集潜在碰撞对
        public void GetPotentialPairs(List<ColliderPair> results)
        {
            results.Clear();
            PairSet.Clear();

            ////遍历所有非空桶
            for (int i = 0; i < _dirtyCount; i++)
            {
                int u = _dirty[i];
                //链式前向星两两遍历桶内所有碰撞体
                for (int e1 = _head[u]; e1 != 0; e1 = _next[e1])
                {
                    FPCollider a = _to[e1];

                    //跳过被remove的空物体
                    if (a == null)
                        continue;

                    for (int e2 = _next[e1]; e2 != 0; e2 = _next[e2])
                    {
                        FPCollider b = _to[e2];

                        if (b == null || a.ID == b.ID)
                            continue;

                        ColliderPair pair = new ColliderPair(a, b);
                        //如果这对组合尚未检测过 且AABB相交 则加入结果列表
                        if (PairSet.Add(pair.PairKey))
                        {
                            if (a.Bounds.Intersects(b.Bounds))
                            {
                                results.Add(pair);
                            }
                        }
                    }
                }
            }
        }

        //AABB区域查询
        public void QueryAABB(FPBounds bounds, List<FPCollider> results)
        {
            results.Clear();
            QuerySet.Clear();

            //算出 bounds.Min 和 bounds.Max 覆盖的网格范围
            GetCellCoord(bounds.Min, out int min_x, out int min_y, out int min_z);
            GetCellCoord(bounds.Max, out int max_x, out int max_y, out int max_z);

            //三重循环遍历算出所给定的bounds所占据的空间哈希桶号
            for (int x = min_x; x <= max_x; x++)
            {
                for (int y = min_y; y <= max_y; y++)
                {
                    for (int z = min_z; z <= max_z; z++)
                    {
                        int u = GetIndex(x, y, z);

                        //遍历桶编号查询与桶内哪些Collider相碰
                        for (int e = _head[u]; e != 0; e = _next[e])
                        {
                            FPCollider collider = _to[e];
                            if (collider == null)
                                continue;

                            if (QuerySet.Add(collider.ID))
                            {
                                if (bounds.Intersects(collider.Bounds))
                                {
                                    results.Add(collider);
                                }
                            }
                        }
                    }
                }
            }
        }
        //Ray射线查询
        public void QueryRay(FPVector3 origin, FPVector3 direction, FP maxDistance, List<FPCollider> results)
        {
            results.Clear();
            QuerySet.Clear();

            //起点所在的网格坐标 (x, y, z)
            GetCellCoord(origin, out int x, out int y, out int z);

            //步进方向
            int stepX = direction.x >= FP.Zero ? 1 : -1;
            int stepY = direction.y >= FP.Zero ? 1 : -1;
            int stepZ = direction.z >= FP.Zero ? 1 : -1;

            //计算各分量跨越一个网格需要的射线长度
            //如果在该分量上绝对值小于Epsilon则认定与该轴平行 设为无穷大
            FP deltaX = FP.Abs(direction.x) > FP.Epsilon ? CellSize / FP.Abs(direction.x) : FP.MaxValue;
            FP deltaY = FP.Abs(direction.y) > FP.Epsilon ? CellSize / FP.Abs(direction.y) : FP.MaxValue;
            FP deltaZ = FP.Abs(direction.z) > FP.Epsilon ? CellSize / FP.Abs(direction.z) : FP.MaxValue;

            //计算从起点出发 第一次碰到各分量的网格边界的长度
            FP nx = FP.FromInt(x + ((stepX + 1) >> 1)) * CellSize;
            FP ny = FP.FromInt(y + ((stepY + 1) >> 1)) * CellSize;
            FP nz = FP.FromInt(z + ((stepZ + 1) >> 1)) * CellSize;

            FP maxX = FP.Abs(direction.x) > FP.Epsilon ? (nx - origin.x) / direction.x : FP.MaxValue;
            FP maxY = FP.Abs(direction.y) > FP.Epsilon ? (ny - origin.y) / direction.y : FP.MaxValue;
            FP maxZ = FP.Abs(direction.z) > FP.Epsilon ? (nz - origin.z) / direction.z : FP.MaxValue;

            //沿射线贪心步进
            FP t = FP.Zero;
            while (t <= maxDistance)
            {
                //先查看当前格子的桶中有无碰撞的collider
                CheckCell(x, y, z);

                //找出当前步距离最近的边界时间 tNext
                FP tNext = FP.Min(maxX, FP.Min(maxY, maxZ));

                //判断射线是否同时穿过多个轴的交界线
                bool hitX = (maxX == tNext);
                bool hitY = (maxY == tNext);
                bool hitZ = (maxZ == tNext);

                //如果同时穿过两个或三个轴的交界处 将相邻格子也纳入检查
                if (hitX && hitY) { CheckCell(x + stepX, y, z); CheckCell(x, y + stepY, z); }
                if (hitY && hitZ) { CheckCell(x, y + stepY, z); CheckCell(x, y, z + stepZ); }
                if (hitX && hitZ) { CheckCell(x + stepX, y, z); CheckCell(x, y, z + stepZ); }

                t = tNext;
                if (hitX) { maxX += deltaX; x += stepX; }
                if (hitY) { maxY += deltaY; y += stepY; }
                if (hitZ) { maxZ += deltaZ; z += stepZ; }
            }

            //检查当前格子的桶中有无碰撞的collider
            void CheckCell(int cx, int cy, int cz)
            {
                int u = GetIndex(cx, cy, cz);
                for (int ei = _head[u]; ei != 0; ei = _next[ei])
                {
                    FPCollider collider = _to[ei];
                    if (collider == null) continue;

                    if (QuerySet.Add(collider.ID))
                    {
                        if (PhysicsMath.IntersectRayAABB(origin, direction, maxDistance, collider.Bounds))
                        {
                            results.Add(collider);
                        }
                    }
                }
            }
        }

        public void Draw() { }
    }
}