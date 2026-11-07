using FixedMath;
using System;
using System.Collections.Generic;

namespace FixedPhysics
{
    public class BVH : IBroadPhase
    {
        //包围盒外扩边距
        //微小的移动无需更新树结构
        public readonly FP FAT;

        //BVH 树结构
        private int _root;
        private int _count;
        private int[] _left;
        private int[] _right;
        private int[] _parent;

        //回收节点池
        private int _freeCount;
        private int[] _freeNodes;

        //节点包围盒
        private FPBounds[] _bounds;
        //叶子节点的物体Collider
        private FPCollider[] _colliders;

        //key-碰撞体ID value-叶子节点编号
        public Dictionary<uint, int> LeafMap;


        public HashSet<uint> ActiveID;
        public List<uint> RemoveBuffer;

        public BVH(int count = 65536)
        {
            //默认四周外扩 0.2 米
            FAT = FP.FromString("0.2");

            _left = new int[count];
            _right = new int[count];
            _parent = new int[count];

            _freeNodes = new int[count];

            _bounds = new FPBounds[count];
            _colliders = new FPCollider[count];

            LeafMap = new Dictionary<uint, int>(count >> 1);

            ActiveID = new HashSet<uint>(count);
            RemoveBuffer = new List<uint>(count);

            ClearGraph();
        }

        private void ClearGraph()
        {
            _root = 0;
            _count = 0;
            _freeCount = 0;
            LeafMap.Clear();
        }
        //申请一个树节点编号
        private int AllocateNode()
        {
            int node;
            if (_freeCount > 0)
            {
                node = _freeNodes[--_freeCount];
            }
            else
            {
                node = ++_count;
                if (node >= _bounds.Length)
                {
                    int n = _bounds.Length << 1;
                    Array.Resize(ref _left, n);
                    Array.Resize(ref _right, n);
                    Array.Resize(ref _parent, n);

                    Array.Resize(ref _freeNodes, n);

                    Array.Resize(ref _bounds, n);
                    Array.Resize(ref _colliders, n);
                }
            }
            _left[node] = 0;
            _right[node] = 0;
            _parent[node] = 0;
            return node;
        }
        //回收一个树节点编号
        private void FreeNode(int node)
        {
            _colliders[node] = null;
            _freeNodes[_freeCount] = node;
            _freeCount++;
        }
        //插入节点node
        private void Insert(int node)
        {
            //如果树为空则将当前节点作为根节点
            if (_root == 0)
            {
                _root = node;
                _parent[node] = 0;
                return;
            }

            //SAH 向下递归寻找cur
            int cur = _root;
            FPBounds bound = _bounds[node];

            //任意非叶子节点必定有左孩子和右孩子
            while (_left[cur] != 0)
            {
                int l = _left[cur];
                int r = _right[cur];

                //计算cur l r三个的增量代价
                FP cost_cur = _bounds[cur].GetSurface();
                FP cost_l = FPBounds.Union(_bounds[l], bound).GetSurface() - _bounds[l].GetSurface();
                FP cost_r = FPBounds.Union(_bounds[r], bound).GetSurface() - _bounds[r].GetSurface();

                //如果和当前盒子cur Union的代价最小则不再往下拆
                if (cost_cur < cost_l && cost_cur < cost_r)
                {
                    break;
                }
                //否则向下拆解寻找表面积最小的Union方式
                cur = (cost_l < cost_r) ? l : r;
            }

            //申请一个新的节点 将cur与node (Union)装箱在一起
            int father = _parent[cur];
            int union = AllocateNode();

            //初始化union节点信息
            //将cur和node挂为union的左右子树
            _left[union] = cur;
            _right[union] = node;
            _parent[union] = father;

            _bounds[union] = FPBounds.Union(_bounds[cur], bound);

            //更新father节点的结构
            _parent[cur] = union;
            _parent[node] = union;

            if (father != 0)
            {
                if (_left[father] == cur) _left[father] = union;
                else _right[father] = union;
            }
            else { _root = union; }

            //沿着父节点向上回溯 更新所有父节点的AABB包围盒
            SyncHierarchy(father);
        }
        //移除节点node(叶子节点)
        private void Remove(int node)
        {
            if (node == _root)
            {
                _root = 0;
                return;
            }

            int f = _parent[node];
            int gf = _parent[f];
            int sibling = (_left[f] == node) ? _right[f] : _left[f];

            //让sibling接替father的位置 
            if (gf != 0)
            {
                if (_left[gf] == f) _left[gf] = sibling;
                else _right[gf] = sibling;

                _parent[sibling] = gf;
                //向上回溯收缩AABB包围盒
                SyncHierarchy(gf);
            }
            else
            {
                _root = sibling;
                _parent[sibling] = 0;
            }
            //回收多余的 father 节点
            FreeNode(f);
            _parent[node] = 0;
        }
        //向上递归更新节点AABB碰撞箱
        private void SyncHierarchy(int index)
        {
            while (index != 0)
            {
                int left = _left[index];
                int right = _right[index];

                _bounds[index] = FPBounds.Union(_bounds[left], _bounds[right]);
                index = _parent[index];
            }
        }
        //生成外扩的肥包围盒
        private FPBounds CreateFatBounds(FPBounds bound)
        {
            FPVector3 margin = new FPVector3(FAT, FAT, FAT);
            return new FPBounds(bound.Min - margin, bound.Max + margin);
        }

        //生命周期与更新接口
        public void Add(FPCollider collider)
        {
            //如果已经在树中则忽略
            if (LeafMap.ContainsKey(collider.ID))
                return;

            int node = AllocateNode();
            _bounds[node] = CreateFatBounds(collider.Bounds);
            _colliders[node] = collider;

            LeafMap[collider.ID] = node;
            Insert(node);
        }
        public void Remove(FPCollider collider)
        {
            if (LeafMap.TryGetValue(collider.ID, out int node))
            {
                LeafMap.Remove(collider.ID);
                Remove(node);
                FreeNode(node);
            }
        }
        public void Clear()
        {
            ClearGraph();
        }
        public void Update(IReadOnlyList<FPCollider> colliders)
        {
            ActiveID.Clear();
            for (int i = 0; i < colliders.Count; i++)
            {
                FPCollider collider = colliders[i];
                if (collider == null) continue;

                //统计此次更新的colliders
                ActiveID.Add(collider.ID);

                if (LeafMap.TryGetValue(collider.ID, out int node))
                {
                    //如果原有的肥包围盒依然能完全包住物体当前的真实Bounds则无需更新树结构
                    if (_bounds[node].Contains(collider.Bounds))
                        continue;

                    //从树上摘下 -> 更新肥胖包围盒 -> 重新插入树中
                    Remove(node);
                    _bounds[node] = CreateFatBounds(collider.Bounds);
                    Insert(node);
                }
                else
                {
                    Add(collider);
                }
            }

            //将更新列表同步到当前BVH树中 若有节点被删除则手动调用删除
            RemoveBuffer.Clear();
            foreach (var item in LeafMap)
            {
                if (ActiveID.Contains(item.Key))
                    continue;
                RemoveBuffer.Add(item.Key);
            }
            foreach (uint ID in RemoveBuffer)
            {
                int node = LeafMap[ID];
                LeafMap.Remove(ID);
                Remove(node);
                FreeNode(node);
            }
        }

        //收集潜在碰撞对
        public void GetPotentialPairs(List<ColliderPair> results)
        {
            results.Clear();
            if (_root == 0)
                return;
            QuerySelfDFS(_root, results);
        }
        //双节点自我递归
        private void QuerySelfDFS(int cur, List<ColliderPair> results)
        {
            //如果是叶子节点直接返回
            if (_left[cur] == 0)
                return;

            int l = _left[cur];
            int r = _right[cur];

            //左右子树自己检查
            QuerySelfDFS(l, results);
            QuerySelfDFS(r, results);

            //左右子树交叉检查
            QueryPairDFS(l, r, results);
        }
        private void QueryPairDFS(int a, int b, List<ColliderPair> results)
        {
            //如果a和b的肥包装盒都没碰到直接return
            if (!_bounds[a].Intersects(_bounds[b]))
                return;

            bool leafA = (_left[a] == 0);
            bool leafB = (_left[b] == 0);

            //如果两个都是叶子节点
            if (leafA && leafB)
            {
                FPCollider ca = _colliders[a];
                FPCollider cb = _colliders[b];

                if (ca.Bounds.Intersects(cb.Bounds))
                {
                    results.Add(new ColliderPair(ca, cb));
                }
                return;
            }

            if (leafB || (!leafA && _bounds[a].GetSurface() > _bounds[b].GetSurface()))
            {
                QueryPairDFS(_left[a], b, results);
                QueryPairDFS(_right[a], b, results);
            }
            else
            {
                QueryPairDFS(a, _left[b], results);
                QueryPairDFS(a, _right[b], results);
            }
        }

        //空间区域查询与射线查询
        public void QueryAABB(FPBounds bounds, List<FPCollider> results)
        {
            results.Clear();
            if (_root == 0)
                return;

            QueryAABBDFS(_root, bounds, results);
        }
        private void QueryAABBDFS(int cur, FPBounds bounds, List<FPCollider> results)
        {
            if (!bounds.Intersects(_bounds[cur]))
                return;

            //如果是叶子节点，检查真实Bounds是否相交
            if (_left[cur] == 0)
            {
                FPCollider collider = _colliders[cur];
                if (bounds.Intersects(collider.Bounds))
                {
                    results.Add(collider);
                }
                return;
            }

            //内部节点则继续递归左右子树
            QueryAABBDFS(_left[cur], bounds, results);
            QueryAABBDFS(_right[cur], bounds, results);
        }

        public void QueryRay(FPVector3 origin, FPVector3 direction, FP maxDistance, List<FPCollider> results)
        {
            results.Clear();
            if (_root == 0)
                return;

            QueryRayDFS(_root, origin, direction, maxDistance, results);
        }
        private void QueryRayDFS(int cur, FPVector3 origin, FPVector3 direction, FP maxDistance, List<FPCollider> results)
        {
            if (!PhysicsMath.IntersectRayAABB(origin, direction, maxDistance, _bounds[cur]))
                return;

            //如果是叶子节点，检查射线是否穿过真实Bounds
            if (_left[cur] == 0)
            {
                FPCollider collider = _colliders[cur];
                if (PhysicsMath.IntersectRayAABB(origin, direction, maxDistance, collider.Bounds))
                {
                    results.Add(collider);
                }
                return;
            }

            //内部节点则继续递归左右子树
            QueryRayDFS(_left[cur], origin, direction, maxDistance, results);
            QueryRayDFS(_right[cur], origin, direction, maxDistance, results);
        }

        public void Draw()
        {
#if UNITY_EDITOR
            if (_root == 0) return;

            DFS(_root);

            void DFS(int cur)
            {
                bool leaf = (_left[cur] == 0);
                // 叶子节点的 Fat AABB 画青色，外层打包的树枝节点画半透明白色
                UnityEngine.Gizmos.color = leaf
                    ? new UnityEngine.Color(0f, 1f, 0.5f, 0.4f)
                    : new UnityEngine.Color(1f, 1f, 1f, 0.15f);

                FPBounds b = _bounds[cur];
                UnityEngine.Vector3 center = new UnityEngine.Vector3(b.Center.x.ToFloat(), b.Center.y.ToFloat(), b.Center.z.ToFloat());
                UnityEngine.Vector3 size = new UnityEngine.Vector3(b.Size.x.ToFloat(), b.Size.y.ToFloat(), b.Size.z.ToFloat());
                UnityEngine.Gizmos.DrawWireCube(center, size);

                if (!leaf)
                {
                    DFS(_left[cur]);
                    DFS(_right[cur]);
                }
            }
#endif
        }
    }
}