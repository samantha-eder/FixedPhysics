using FixedMath;
using System;
using System.Collections.Generic;

namespace FixedPhysics
{
    public class Octree : IBroadPhase
    {
        //世界边界盒 八叉树递归数量与深度
        public readonly FPBounds WorldBounds;
        public readonly int MaxDepth;
        public readonly int MaxObjects;

        //八叉树节点数组
        private int _nodeCount;
        private FPBounds[] _nodeBounds;
        private int[] _child;
        private int[] _father;
        private int[] _objectCount;


        //链式前向星CFS建图
        private int _count;
        private int[] _head;
        private int[] _next;
        private FPCollider[] _to;

        public Octree(FPBounds world, int maxDepth = 6, int maxObjects = 8, int nodeCount = 65536, int objectCount = 65536)
        {
            WorldBounds = world;
            MaxDepth = maxDepth;
            MaxObjects = maxObjects;

            //初始化节点
            _child = new int[nodeCount];
            _father = new int[MaxDepth + 1];
            _objectCount = new int[nodeCount];
            _nodeBounds = new FPBounds[nodeCount];

            //CFS
            _head = new int[nodeCount];
            _next = new int[objectCount];
            _to = new FPCollider[objectCount];


            //建立 1 号根节点
            ClearGraph();
        }

        //清空八叉树
        private void ClearGraph()
        {
            //清空树结构
            for (int i = 1; i <= _nodeCount; i++)
            {
                //Node
                _child[i] = 0;
                _objectCount[i] = 0;

                //CFS
                _head[i] = 0;
            }
            //清空节点并重设根节点
            _nodeCount = 1;
            _nodeBounds[1] = WorldBounds;

            //清空CFS图
            _count = 1;
        }
        //将节点加入u号树节点中
        private void AddEdge(int u, FPCollider collider)
        {
            //边数组扩容保护
            if (_count >= _next.Length)
            {
                int n = _next.Length << 1;
                Array.Resize(ref _next, n);
                Array.Resize(ref _to, n);
            }

            _next[_count] = _head[u];
            _to[_count] = collider;
            _head[u] = _count;
            _count++;

            //该树节点内的Bound数量+1
            _objectCount[u]++;
        }

        //将单个碰撞体插入八叉树中
        private void InsertCollider(FPCollider collider)
        {
            int cur = 1;
            int depth = 0;
            FPBounds bounds = collider.Bounds;

            //如果物体超出了预设的WorldBounds
            //扩展1号根节点的包围盒
            //并直接将该越界物体留在1号根节点中
            if (!WorldBounds.Contains(bounds))
            {
                _nodeBounds[1].Encapsulate(bounds);
                AddEdge(1, collider);
                return;
            }

            //从根节点开始递归查询
            while (_child[cur] != 0)
            {
                //既然当前节点分裂过 则判断是否能塞入子节点中
                int index = GetChildIndex(_nodeBounds[cur].Center, bounds);

                //如果压线则直接保留在当前节点
                if (index == -1)
                    break;

                //继续递归查询
                cur = _child[cur] + index;
                depth++;
            }

            //将当前collider加入CFS中
            AddEdge(cur, collider);

            //如果节点cur为分裂, 人数超标且深度未满则分裂
            if (_child[cur] == 0 && _objectCount[cur] > MaxObjects && depth < MaxDepth)
            {
                Split(cur);
            }
        }
        //判断Bound属于父节点center的哪一个子节点中 压线则返回-1
        private int GetChildIndex(FPVector3 center, FPBounds bounds)
        {
            int index = 0;

            if (bounds.Min.x > center.x) index |= 1;
            else if (bounds.Max.x >= center.x) return -1;

            if (bounds.Min.y > center.y) index |= 2;
            else if (bounds.Max.y >= center.y) return -1;

            if (bounds.Min.z > center.z) index |= 4;
            else if (bounds.Max.z >= center.z) return -1;

            return index;
        }
        //将节点分成八份 并将节点内的collider重新下发
        //切分根节点时始终以标准WorldBounds为基准 不受越界物体撑大的影响
        private void Split(int cur)
        {
            //树节点数组扩容保护
            if (_nodeCount + 8 >= _nodeBounds.Length)
            {
                int n = _nodeBounds.Length << 1;
                Array.Resize(ref _nodeBounds, n);
                Array.Resize(ref _child, n);
                Array.Resize(ref _objectCount, n);
                Array.Resize(ref _head, n);
            }

            FPBounds pre = (cur == 1) ? WorldBounds : _nodeBounds[cur];
            FPVector3 min = pre.Min;
            FPVector3 center = pre.Center;

            //分配8个连续的子节点编号
            int first = _nodeCount + 1;
            _child[cur] = first;
            _nodeCount += 8;

            //构造8个子节点的 AABB 盒子
            for (int i = 0; i < 8; i++)
            {
                FPVector3 cm = new FPVector3(
                    (i & 1) == 0 ? min.x : center.x,
                    (i & 2) == 0 ? min.y : center.y,
                    (i & 4) == 0 ? min.z : center.z
                );

                //初始化盒子
                int index = first + i;
                _nodeBounds[index] = new FPBounds(cm, cm + pre.Extents);
            }

            //取出原父节点 cur 的所有物体
            //依次插入到新的CFS中
            int ei = _head[cur];

            //清空父节点的信息
            _head[cur] = 0;
            _objectCount[cur] = 0;

            while (ei != 0)
            {
                int next = _next[ei];
                FPCollider collider = _to[ei];

                //如果碰撞体不为null 则判断是留在当前父节点还是插入到子节点中
                if (collider != null)
                {
                    //得到要插入的子节点index
                    int index = GetChildIndex(center, collider.Bounds);
                    int target = (index == -1) ? cur : (first + index);

                    _next[ei] = _head[target];
                    _head[target] = ei;
                    _objectCount[target]++;
                }
                ei = next;
            }
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
                    //每个物体在八叉树中严格只占1条边
                    break;
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

        public void GetPotentialPairs(List<ColliderPair> results)
        {
            results.Clear();
            CollectPairsDFS(1, 0, results);
        }
        //DFS收集可能的碰撞对
        private void CollectPairsDFS(int cur, int depth, List<ColliderPair> results)
        {
            int first = _head[cur];

            //首先检查当前房间是否存在物体
            if (first != 0)
            {
                //遍历当前房间里所有物体的Collider
                for (int e1 = first; e1 != 0; e1 = _next[e1])
                {
                    FPCollider a = _to[e1];
                    if (a == null)
                        continue;

                    //首先将当前房间内的物体两两配对
                    for (int e2 = _next[e1]; e2 != 0; e2 = _next[e2])
                    {
                        FPCollider b = _to[e2];
                        if (b == null)
                            continue;
                        //判断两物体有没有相交
                        if (a.Bounds.Intersects(b.Bounds))
                        {
                            results.Add(new ColliderPair(a, b));
                        }
                    }

                    //将当前房间里的物体与父节点格子里的物体两两比较
                    for (int i = 0; i < depth; i++)
                    {
                        int f = _father[i];

                        for (int e2 = _head[f]; e2 != 0; e2 = _next[e2])
                        {
                            FPCollider b = _to[e2];
                            if (b == null)
                                continue;

                            if (a.Bounds.Intersects(b.Bounds))
                            {
                                results.Add(new ColliderPair(a, b));
                            }
                        }
                    }
                }
            }

            //如果当前房间分裂过子房间 
            //则递归进入八个子房间
            int child = _child[cur];
            if (child != 0)
            {
                //将当前房间加入到father中
                _father[depth] = cur;

                //DFS向下递归
                for (int i = 0; i < 8; i++)
                {
                    CollectPairsDFS(child + i, depth + 1, results);
                }
            }
        }

        public void QueryAABB(FPBounds bounds, List<FPCollider> results)
        {
            results.Clear();
            QueryAABBDFS(1, bounds, results);
        }
        private void QueryAABBDFS(int cur, FPBounds bounds, List<FPCollider> results)
        {
            if (!bounds.Intersects(_nodeBounds[cur]))
                return;

            //遍历当前房间内所有物体
            for (int e = _head[cur]; e != 0; e = _next[e])
            {
                FPCollider collider = _to[e];
                if (collider == null)
                    continue;

                if (bounds.Intersects(collider.Bounds))
                {
                    results.Add(collider);
                }
            }

            //向子房间递归
            int child = _child[cur];
            if (child != 0)
            {
                for (int i = 0; i < 8; i++)
                {
                    QueryAABBDFS(child + i, bounds, results);
                }
            }

        }

        public void QueryRay(FPVector3 origin, FPVector3 direction, FP maxDistance, List<FPCollider> results)
        {
            results.Clear();
            QueryRayDFS(1, origin, direction, maxDistance, results);
        }
        private void QueryRayDFS(int cur, FPVector3 origin, FPVector3 direction, FP maxDistance, List<FPCollider> results)
        {
            //如果射线没有碰到当前房间则剪枝跳过
            if (!PhysicsMath.IntersectRayAABB(origin, direction, maxDistance, _nodeBounds[cur]))
            {
                return;
            }

            //遍历当前房间内所有物体
            for (int ei = _head[cur]; ei != 0; ei = _next[ei])
            {
                FPCollider collider = _to[ei];
                if (collider == null)
                    continue;

                if (PhysicsMath.IntersectRayAABB(origin, direction, maxDistance, collider.Bounds))
                {
                    results.Add(collider);
                }
            }

            //向子房间递归
            int child = _child[cur];
            if (child != 0)
            {
                for (int i = 0; i < 8; i++)
                {
                    QueryRayDFS(child + i, origin, direction, maxDistance, results);
                }
            }
        }

        public void Draw()
        {
#if UNITY_EDITOR
            UnityEngine.Gizmos.color = new UnityEngine.Color(0f, 0.8f, 1f, 0.25f);
            for (int i = 1; i <= _nodeCount; i++)
            {
                FPBounds b = _nodeBounds[i];
                UnityEngine.Vector3 center = new UnityEngine.Vector3(b.Center.x.ToFloat(), b.Center.y.ToFloat(), b.Center.z.ToFloat());
                UnityEngine.Vector3 size = new UnityEngine.Vector3(b.Size.x.ToFloat(), b.Size.y.ToFloat(), b.Size.z.ToFloat());
                UnityEngine.Gizmos.DrawWireCube(center, size);
            }
#endif
        }
    }
}