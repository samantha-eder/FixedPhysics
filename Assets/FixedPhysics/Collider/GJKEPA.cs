using FixedMath;
using System;

namespace FixedPhysics
{
    public class GJKEPA
    {
        private FaceHeap FHeap;
        private EdgeBuffer EBuffer;
        private readonly FP PHY_TOLERANCE;

        public GJKEPA(int capacity = 256)
        {
            FHeap = new FaceHeap(capacity);
            EBuffer = new EdgeBuffer(capacity);
            PHY_TOLERANCE = FP.FromValue(429496L);
        }


        //利用GJK 与 EPA 算法计算任意Shape
        public CollisionManifold IntersectAny(
            FPShape shapeA, FPTransform transA,
            FPShape shapeB, FPTransform transB)
        {
            //GJK 检测是否碰撞 并返回包含原点的简单型
            if (GJK(shapeA, transA, shapeB, transB, out Simplex simplex))
            {
                //发生碰撞 使用 EPA 计算穿透深度和法线
                return EPA(shapeA, transA, shapeB, transB, simplex);
            }
            return CollisionManifold.NoHit();
        }

        //计算射线与任意Shape是否相撞 并返回length与相撞面法线
        public bool Raycast(
            FPVector3 origin, FPVector3 direction, FP distance,
            FPShape shape, FPTransform trans,
            out FP length, out FPVector3 normal)
        {
            length = FP.Zero;
            normal = FPVector3.Zero;

            Simplex simplex = default;
            simplex.Count = 0;

            //若起点在形状内部 默认法线为射线反方向
            FPVector3 cur = origin;
            FPVector3 nor = -direction;

            //初始搜索方向从物体中心指向射线起点
            FPVector3 dir = cur - trans.Position;
            if (FPVector3.Magnitude(dir) < FP.Epsilon)
            {
                dir = FPVector3.Right;
            }

            int iterations = 32;
            while (iterations-- > 0)
            {
                //归一化搜索方向
                dir = FPVector3.Normalize(dir);

                //多胞体上摸出最靠近原点的极点
                FPVector3 p = shape.GetSupportPoint(trans, dir);
                FPVector3 A = p - cur;

                //判断当前方向是否可以向前推进
                //原点(cur)是否还在极点 A 的外侧 即Dot(A, dir) < 0
                FP proj = FPVector3.Dot(A, dir);
                if (proj < -PHY_TOLERANCE)
                {
                    //计算射线飞行方向 direction 迎着外法线 dir 的推进速度
                    FP vr = -FPVector3.Dot(dir, direction);

                    //如果射线与分离面平行或背离
                    if (vr <= FP.Epsilon)
                    {
                        return false;
                    }

                    //推进距离 Δt = 平面到原点的距离(-proj) / 推进速度(vr)
                    FP delta = -proj / vr;
                    length += delta;

                    //超过最大检测距离则未击中
                    if (length > distance)
                    {
                        return false;
                    }

                    //沿着射线方向 direction 推进起点 cur
                    //并把单纯形里现有的点往反方向同步平移
                    FPVector3 shift = direction * delta;
                    cur += shift;
                    A -= shift;

                    //清空单纯形
                    simplex.Count = 0;

                    //记录法线
                    nor = dir;
                }
                //如果距离已收敛到 0 说明射线点cur已经在多胞体表面上
                else if (simplex.Count > 0 && FP.Abs(proj) <= PHY_TOLERANCE)
                {
                    normal = nor;
                    return true;
                }

                //将点加入单纯形
                simplex.Push(A);
                if (NextSimplex(ref simplex, ref dir))
                {
                    normal = nor;
                    return true;
                }
            }
            return false;
        }


        //GJK算法
        private bool GJK(
            FPShape shapeA, FPTransform transA,
            FPShape shapeB, FPTransform transB,
            out Simplex simplex)
        {
            //初始化单纯形
            simplex = default;
            simplex.Count = 0;

            //首先随机选择一个初始搜索方向
            //默认选择两物体的中心点连线
            FPVector3 dir = transB.Position - transA.Position;

            //如果中心点重合也计算GJK 得到一个Simplex交给EPA算法
            if (FPVector3.Magnitude(dir) < FP.Epsilon) { dir = FPVector3.Right; }

            //提取第一个点 添加进Simplex中
            FPVector3 A = GetPoint(dir);
            simplex.Push(A);

            //下一个方向直接指向原点
            dir = -A;

            //设定最大迭代次数
            int iterations = 32;
            while (iterations-- > 0)
            {
                //向新方向dir摸出下一个极点
                A = GetPoint(dir);

                //如果在该方向上摸到的最远极点都没有跨越原点
                //说明闵可夫斯基差绝对不可能包含原点 这两个物体没有相交
                //该方向向量dir即为分离轴
                if (FPVector3.Dot(A, dir) < FP.Zero)
                {
                    return false;
                }

                simplex.Push(A);

                //判断当前单纯形是否能包含原点
                //如果不能则削减无用的点并给出下一个搜索方向dir
                if (NextSimplex(ref simplex, ref dir))
                {
                    return true;
                }
            }
            //如果超过最大迭代次数则判断未相交
            return false;


            //给定一个方向direction 获取这两个Shape在该方向下的最远点
            //相减后得到世界空间下的具体的坐标点point
            FPVector3 GetPoint(FPVector3 direction)
            {
                // Minkowski Support = SupportA(direction) - SupportB(-direction)
                FPVector3 pA = shapeA.GetSupportPoint(transA, direction);
                FPVector3 pB = shapeB.GetSupportPoint(transB, -direction);
                return pA - pB;
            }
        }
        private bool NextSimplex(ref Simplex simplex, ref FPVector3 direction)
        {
            switch (simplex.Count)
            {
                case 1: direction = -simplex.A; return false;
                case 2: return Do2(ref simplex, ref direction);
                case 3: return Do3(ref simplex, ref direction);
                case 4: return Do4(ref simplex, ref direction);
            }
            return false;
        }
        private bool Do2(ref Simplex simplex, ref FPVector3 direction)
        {
            FPVector3 AB = simplex.B - simplex.A;
            FPVector3 AO = -simplex.A;

            //如果原点落在端点A的外侧 
            //说明此时点B已经没有用了 直接丢弃B
            //该情况只发生于Raycast 而不发生于普通的GJK算法

            //因为在普通的GJK算法中
            //此时的原点O在AB线段上的投影
            //必定落在线段AB内 ∠OAB必定为锐角
            if (FPVector3.Dot(AB, AO) <= FP.Zero)
            {
                simplex.Set(simplex.A);
                direction = AO;
                return false;
            }

            //得到垂直AB向量指向原点的dir向量
            AB = FPVector3.Normalize(AB);
            direction = FPVector3.Cross(FPVector3.Cross(AB, AO), AB);

            //如果原点正好在AB延长线上则给一个垂直于AB的任意线段即可
            if (direction == FPVector3.Zero)
            {
                //判断direction是否是z轴方向的向量 
                //如果是则给Right向量即可 不是则给AB向量在xy平面上的垂直向量
                direction = ((FP.Abs(AB.x) < FP.Epsilon) && (FP.Abs(AB.y) < FP.Epsilon)) ?
                            FPVector3.Right : new FPVector3(AB.y, -AB.x, FP.Zero);
            }
            return false;
        }
        private bool Do3(ref Simplex simplex, ref FPVector3 direction)
        {
            FPVector3 AB = simplex.B - simplex.A;
            FPVector3 AC = simplex.C - simplex.A;
            FPVector3 AO = -simplex.A;

            //ABC面法线normal
            //AC AB边朝外的法线
            FPVector3 normal = FPVector3.Cross(AB, AC);
            FPVector3 outAC = FPVector3.Cross(normal, AC);
            FPVector3 outAB = FPVector3.Cross(AB, normal);

            //判断原点是否在AC侧
            if (FPVector3.Dot(outAC, AO) > FP.Zero)
            {
                //舍弃B 并交给Do2自动判断钝角判断
                //囊括了GJK与Raycast的两种情况
                simplex.Set(simplex.A, simplex.C);
                return Do2(ref simplex, ref direction);
            }
            //判断原点是否在AB侧
            if (FPVector3.Dot(outAB, AO) > FP.Zero)
            {
                //舍弃C 并交给Do2自动判断钝角判断
                //囊括了GJK与Raycast的两种情况
                simplex.Set(simplex.A, simplex.B);
                return Do2(ref simplex, ref direction);
            }
            //原点在三角形ABC内
            //判断在ABC上侧还是下侧就行了
            //如果在ABC上侧 选择法线即可
            if (FPVector3.Dot(normal, AO) > FP.Zero)
            {
                direction = normal;
            }
            else
            {
                //反转顶点顺序保证法线正确
                simplex.Set(simplex.A, simplex.C, simplex.B);
                direction = -normal;
            }
            return false;
        }
        private bool Do4(ref Simplex simplex, ref FPVector3 direction)
        {
            FPVector3 A = simplex.A; FPVector3 B = simplex.B;
            FPVector3 C = simplex.C; FPVector3 D = simplex.D;

            FPVector3 AB = B - A;
            FPVector3 AC = C - A;
            FPVector3 AD = D - A;
            FPVector3 AO = -A;

            //算出三个侧面朝外的法线
            FPVector3 ABC_N = FPVector3.Cross(AB, AC);
            FPVector3 ACD_N = FPVector3.Cross(AC, AD);
            FPVector3 ADB_N = FPVector3.Cross(AD, AB);

            //检查原点是否在 ABC 面外侧
            if (FPVector3.Dot(ABC_N, AO) > FP.Zero)
            {
                simplex.Set(A, B, C); // 移除D 退化为三角形 ABC
                return Do3(ref simplex, ref direction);
            }
            //检查原点是否在 ACD 面外侧
            if (FPVector3.Dot(ACD_N, AO) > FP.Zero)
            {
                simplex.Set(A, C, D); // 移除B 退化为三角形 ACD
                return Do3(ref simplex, ref direction);
            }
            //检查原点是否在 ADB 面外侧
            if (FPVector3.Dot(ADB_N, AO) > FP.Zero)
            {
                simplex.Set(A, D, B); // 移除C 退化为三角形 ADB
                return Do3(ref simplex, ref direction);
            }
            //如果原点不在任何一个面的外侧 那必定在四面体内部
            //已找到simplex 退出循环返回给EPA
            return true;
        }

        //EPA算法
        private CollisionManifold EPA(
                FPShape shapeA, FPTransform transA,
                FPShape shapeB, FPTransform transB,
                Simplex simplex)
        {
            //如果GJK返回的单纯形不是完整的四面体(共面/共线情况)
            //直接返回一个安全的0深度Manifold交由下一次物理帧处理
            if (simplex.Count < 4)
            {
                return new CollisionManifold(true, FP.Zero, FPVector3.Up);
            }

            //初始化清空缓存
            FHeap.Clear();
            EBuffer.Clear();

            //提取出GJK计算得到的简单型的四个顶点
            FPVector3 A = simplex.A; FPVector3 B = simplex.B;
            FPVector3 C = simplex.C; FPVector3 D = simplex.D;

            //将四面体的四个面添加进FHeap中 并保证法线朝外
            AddFace(A, B, C); AddFace(A, C, D); AddFace(A, D, B); AddFace(B, D, C);

            //设置最大膨胀迭代次数
            int iterations = 32;
            EPAFace face = default;

            while (iterations-- > 0 && FHeap.Count > 0)
            {
                //首先获取离原点最近的面
                face = FHeap.Pop();

                //根据该面的法向量normal获取边缘上的极点
                FPVector3 point = GetPoint(face.Normal);

                //计算该极点在法向量上的投影距离
                FP pdist = FPVector3.Dot(point, face.Normal);

                //将投影距离与原点到平面ABC的距离相减
                //判断得到的增量值是否小于Epsilon
                //如果小于则停止迭代 已寻找到最近边缘点
                if (pdist - face.Distance < PHY_TOLERANCE)
                {
                    return new CollisionManifold(true, face.Distance, face.Normal);
                }

                //如果发现更远点
                //开始破壁 清空边缓存
                //将所有P点能看到的面都删去 并留下轮廓线
                EBuffer.Clear();

                //倒序遍历堆中剩下的所有面 考察是否要删去
                for (int i = FHeap.Count - 1; i >= 0; i--)
                {
                    EPAFace item = FHeap.Heap[i];

                    //判断当前的面是否能"看见"P
                    //只要面法线与AP的点乘大于零即P在自己这个面法线的外侧 删去
                    if (FPVector3.Dot(item.Normal, point - item.A) > PHY_TOLERANCE)
                    {
                        //将这个要删除的面的三条边加入缓冲区
                        EBuffer.AddEdge(item.A, item.B);
                        EBuffer.AddEdge(item.B, item.C);
                        EBuffer.AddEdge(item.C, item.A);

                        //从堆中删去这个面
                        FHeap.RemoveAt(i);
                    }
                }
                FHeap.Rebuild();

                //此时堆中只剩一些P"看不到"的面
                //EdgeBuffer中记录了删去面之后剩余面的需要缝合的边
                //但还需要把一开始的那个被POP的面的三条边也加入Buffer
                EBuffer.AddEdge(face.A, face.B);
                EBuffer.AddEdge(face.B, face.C);
                EBuffer.AddEdge(face.C, face.A);

                //遍历缓存的边 与极点P组成新的三角面加入Heap中构成新的扩展多面体
                for (int i = 0; i < EBuffer.Count; i++)
                {
                    EPAEdge item = EBuffer.Edges[i];
                    AddFace(item.A, item.B, point);
                }
                //继续递归
            }
            //如果由于某种原因达到了最大迭代次数还没收敛
            //就返回最后一次迭代得到的最近面作为近似解
            return new CollisionManifold(true, face.Distance, face.Normal);

            //创建一个面 并确保法线朝外
            void AddFace(FPVector3 a, FPVector3 b, FPVector3 c)
            {
                EPAFace face = new EPAFace(a, b, c);
                FHeap.Push(face);
            }

            //获取某个方向direction的闵可夫斯基差上的支撑点
            //与GJK代码完全一致
            FPVector3 GetPoint(FPVector3 direction)
            {
                // Minkowski Support = SupportA(direction) - SupportB(-direction)
                FPVector3 pA = shapeA.GetSupportPoint(transA, direction);
                FPVector3 pB = shapeB.GetSupportPoint(transB, -direction);
                return pA - pB;
            }
        }

        //单纯形
        private struct Simplex
        {
            public FPVector3 A, B, C, D;
            public int Count;

            public void Push(FPVector3 point)
            {
                //新加入的点永远放在最前面 位置A
                D = C; C = B; B = A; A = point;
                Count = Count == 4 ? 4 : Count + 1;
            }
            public void Set(FPVector3 a)
            {
                this.A = a;
                Count = 1;
            }
            public void Set(FPVector3 a, FPVector3 b)
            {
                this.A = a; this.B = b;
                Count = 2;
            }
            public void Set(FPVector3 a, FPVector3 b, FPVector3 c)
            {
                this.A = a; this.B = b; this.C = c;
                Count = 3;
            }
            public void Translate(FPVector3 offset)
            {
                if (Count >= 1) A += offset;
                if (Count >= 2) B += offset;
                if (Count >= 3) C += offset;
                if (Count == 4) D += offset;
            }
        }

        //小根堆存储所有的三角形面
        //根据面到原点的距离组织小根堆
        private class FaceHeap
        {
            public int Count;
            public EPAFace[] Heap;

            public FaceHeap(int capacity)
            {
                Count = 0;
                Heap = new EPAFace[capacity];
            }

            public void Clear()
            {
                Count = 0;
            }
            public void Push(EPAFace face)
            {
                //防越界保护
                if (Count >= Heap.Length)
                    return;

                Heap[Count] = face;
                UP(Count);
                Count++;
            }
            public EPAFace Pop()
            {
                EPAFace face = Heap[0];
                Count--;
                Heap[0] = Heap[Count];
                Down(0);
                return face;
            }
            public void RemoveAt(int index)
            {
                Count--;
                Heap[index] = Heap[Count];
            }
            public void Rebuild()
            {
                for (int i = (Count >> 1) - 1; i >= 0; i--)
                {
                    Down(i);
                }
            }

            //小根堆向上调整
            private void UP(int index)
            {
                while (index > 0)
                {
                    int parent = (index - 1) >> 1;
                    //当前index的distance大于father的distance则停止
                    //否则继续向上调整
                    if (Heap[index].Distance >= Heap[parent].Distance)
                        break;

                    //交换
                    EPAFace temp = Heap[index];
                    Heap[index] = Heap[parent];
                    Heap[parent] = temp;

                    //继续向上调整堆
                    index = parent;
                }
            }
            //小根堆向下调整
            private void Down(int index)
            {
                while (true)
                {
                    int left = (index << 1) + 1;
                    int right = (index << 1) + 2;
                    int smallest = index;

                    //找出最小子节点
                    if (left < Count && Heap[left].Distance < Heap[smallest].Distance)
                        smallest = left;
                    if (right < Count && Heap[right].Distance < Heap[smallest].Distance)
                        smallest = right;

                    //如果最小子节点是自己则停止
                    if (smallest == index)
                        break;

                    //交换头节点与最小子节点
                    EPAFace temp = Heap[index];
                    Heap[index] = Heap[smallest];
                    Heap[smallest] = temp;

                    //继续向下调整堆
                    index = smallest;
                }
            }
        }
        //轮廓边缓冲区
        //自动剔除三角形共享边
        private class EdgeBuffer
        {
            public int Count;
            public EPAEdge[] Edges;

            public EdgeBuffer(int capacity)
            {
                Count = 0;
                Edges = new EPAEdge[capacity];
            }

            public void Clear()
            {
                this.Count = 0;
            }
            public void AddEdge(FPVector3 a, FPVector3 b)
            {
                EPAEdge edge = new EPAEdge(a, b);
                for (int i = 0; i < Count; i++)
                {
                    //添加边时如果边已经存在
                    //说明这条边是两个面的共享边
                    //则利用末尾元素覆盖这条边代替删除
                    //添加到最后则数组内所有边即为需要删除的面
                    //和不需要删除的面之间的轮廓边
                    if (Edges[i].IsSame(edge))
                    {
                        Edges[i] = Edges[Count - 1];
                        Count--;
                        return;
                    }
                }
                //容量不足时翻倍扩容
                if (Count >= Edges.Length)
                {
                    Array.Resize(ref Edges, Edges.Length << 1);
                }

                Edges[Count] = edge;
                Count++;
            }
        }
        //三角形边结构
        private struct EPAEdge
        {
            public FPVector3 A, B;
            public EPAEdge(FPVector3 a, FPVector3 b)
            {
                A = a; B = b;
            }

            //比较两条边是否是同一条 共享边
            //因为相邻的两个面 按照顺时针遍历
            //共享的那条边顶点顺序必然相反 A->B B->A
            public bool IsSame(EPAEdge other)
            {
                return (this.A == other.B && this.B == other.A) ||
                       (this.A == other.A && this.B == other.B);
            }
        }
        //三角形面结构
        private struct EPAFace
        {
            public FPVector3 A, B, C;
            public FPVector3 Normal;
            public FP Distance;

            public EPAFace(FPVector3 a, FPVector3 b, FPVector3 c)
            {
                A = a; B = b; C = c;

                //计算法线并归一化
                FPVector3 ab = B - A;
                FPVector3 ac = C - A;
                Normal = FPVector3.Normalize(FPVector3.Cross(ab, ac));

                //计算原点到面的距离
                //利用法线和面上一点的点乘
                //从原点出发到A点的向量与法线做点乘
                Distance = FPVector3.Dot(Normal, A);

                //如果 Distance < 0 说明法线朝内指向原点
                //对调 B C 并取反
                if (Distance < FP.Zero)
                {
                    B = c; C = b;
                    Normal = -Normal;
                    Distance = -Distance;
                }
            }
        }
    }
}