using FixedMath;

namespace FixedPhysics
{
    public static class PhysicsMath
    {
        //计算容差 0.0001 避免精度过高产生死锁
        private static readonly FP PHY_TOLERANCE = FP.FromValue(429496L);

        //球体与球体
        public static CollisionManifold Intersect(
            FPSphere sphereA, FPTransform transA,
            FPSphere sphereB, FPTransform transB)
        {
            FPVector3 posA = transA.Position;
            FPVector3 posB = transB.Position;

            //计算方向向量
            FPVector3 dir = posB - posA;

            //比较距离
            FP distance = FPVector3.Magnitude(dir);
            FP sr = sphereA.Radius + sphereB.Radius;
            if (distance > sr)
            {
                return CollisionManifold.NoHit();
            }

            FPVector3 normal = distance > FP.Epsilon ? dir / distance : FPVector3.Up;
            return new CollisionManifold(true, sr - distance, normal);
        }
        //球体与Box
        public static CollisionManifold Intersect(
            FPSphere sphereA, FPTransform transA,
            FPBox boxB, FPTransform transB)
        {
            //首先将sphere的坐标转换到box的局部坐标系
            FPVector3 center = transB.ToLocalPos(transA.Position);

            //将球心Clamped到Box的范围
            //如果不在盒子内则会得到距离球心最近的Box上的点
            FPVector3 point = new FPVector3
            (
                FP.Clamp(center.x, -boxB.Extents.x, boxB.Extents.x),
                FP.Clamp(center.y, -boxB.Extents.y, boxB.Extents.y),
                FP.Clamp(center.z, -boxB.Extents.z, boxB.Extents.z)
            );

            //计算最近点point和球心之间的距离
            FPVector3 dir = center - point;

            //比较距离
            FP distance = FPVector3.Magnitude(dir);
            if (distance > sphereA.Radius)
            {
                return CollisionManifold.NoHit();
            }

            //如果球心在Box外
            if (distance > FP.Epsilon)
            {
                //现在的法线是 boxB 局部空间下的法线
                //从盒子最近点指向球心
                //需要利用TransB的ToWorldDir转回世界空间
                FPVector3 normal = -transB.ToWorldDir(dir / distance);
                return new CollisionManifold(true, sphereA.Radius - distance, normal);
            }
            else
            {
                //寻找一个离球心最近的面将球推出去
                FP xd = boxB.Extents.x - FP.Abs(center.x);
                FP yd = boxB.Extents.y - FP.Abs(center.y);
                FP zd = boxB.Extents.z - FP.Abs(center.z);

                FP penetration;
                FPVector3 normal;
                if (xd <= yd && xd <= zd)
                {
                    penetration = sphereA.Radius + xd;
                    normal = center.x > FP.Zero ? FPVector3.Right : FPVector3.Left;
                }
                else if (yd <= xd && yd <= zd)
                {
                    penetration = sphereA.Radius + yd;
                    normal = center.y > FP.Zero ? FPVector3.Up : FPVector3.Down;
                }
                else
                {
                    penetration = sphereA.Radius + zd;
                    normal = center.z > FP.Zero ? FPVector3.Forward : FPVector3.Back;
                }
                normal = -transB.ToWorldDir(normal);
                return new CollisionManifold(true, penetration, normal);
            }
        }
        //Box与Box SAT分离轴算法
        public static CollisionManifold Intersect(
            FPBox boxA, FPTransform transA,
            FPBox boxB, FPTransform transB)
        {
            //首先提取出两个BOX的三个局部坐标轴在世界空间下的向量
            FPVector3 a0 = transA.Right, a1 = transA.Up, a2 = transA.Forward;
            FPVector3 b0 = transB.Right, b1 = transB.Up, b2 = transB.Forward;

            FPVector3 extA = boxA.Extents;
            FPVector3 extB = boxB.Extents;

            //中心点连线向量
            FPVector3 T = transB.Position - transA.Position;

            //记录最小重叠量 初始为最大值
            FP penetration = FP.MaxValue;
            FPVector3 normal = FPVector3.Zero;

            //测试BoxA的三个面法线
            for (int i = 0; i < 3; i++)
            {
                if (IsSeparated((i == 0 ? a0 : i == 1 ? a1 : a2)))
                {
                    return CollisionManifold.NoHit();
                }
            }
            //测试BoxB的三个面法线
            for (int i = 0; i < 3; i++)
            {
                if (IsSeparated((i == 0 ? b0 : i == 1 ? b1 : b2)))
                {
                    return CollisionManifold.NoHit();
                }
            }
            //测试BoxA与BoxB的三根轴的叉乘
            for (int i = 0; i < 3; i++)
            {
                for (int j = 0; j < 3; j++)
                {
                    FPVector3 cross = FPVector3.Cross((i == 0 ? a0 : i == 1 ? a1 : a2), (j == 0 ? b0 : j == 1 ? b1 : b2));
                    //过滤掉平行向量并归一化
                    if (FPVector3.Magnitude(cross) <= PHY_TOLERANCE)
                        continue;
                    cross = FPVector3.Normalize(cross);
                    if (IsSeparated(cross))
                    {
                        return CollisionManifold.NoHit();
                    }
                }
            }

            //确保此时碰撞法线由A指向B
            normal = FPVector3.Dot(normal, T) < FP.Zero ? -normal : normal;
            return new CollisionManifold(true, penetration, normal);

            //判断某根轴上两个BOX是否有分离
            bool IsSeparated(FPVector3 axis)
            {
                //计算BoxA在该轴axis上的投影半径
                FP rA = extA.x * FP.Abs(FPVector3.Dot(a0, axis)) +
                        extA.y * FP.Abs(FPVector3.Dot(a1, axis)) +
                        extA.z * FP.Abs(FPVector3.Dot(a2, axis));
                //计算BoxB在该轴axis上的投影半径
                FP rB = extB.x * FP.Abs(FPVector3.Dot(b0, axis)) +
                        extB.y * FP.Abs(FPVector3.Dot(b1, axis)) +
                        extB.z * FP.Abs(FPVector3.Dot(b2, axis));
                //计算中心点连线在该轴上的投影距离
                FP distance = FP.Abs(FPVector3.Dot(T, axis));
                FP overlap = (rA + rB) - distance;
                if (overlap <= FP.Zero)
                    return true;

                //记录此时的penetration和axis
                if (overlap < penetration)
                {
                    penetration = overlap;
                    normal = axis;
                }
                return false;
            }
        }

        //利用GJK 与 EPA 算法计算任意Shape
        public static CollisionManifold IntersectAny(
                GJKEPA g,
                FPShape shapeA, FPTransform transA,
                FPShape shapeB, FPTransform transB)
        {
            return g.IntersectAny(shapeA, transA, shapeB, transB);
        }

        //判断射线与Box是否相交 
        //供SpatialHash Octree DynamicBVH 调用
        public static bool IntersectRayAABB(
            FPVector3 origin, FPVector3 direction, FP distance,
            FPBounds bounds)
        {
            FP tMin = FP.Zero;
            FP tMax = distance;

            if (!CheckAxis(origin.x, direction.x, bounds.Min.x, bounds.Max.x, ref tMin, ref tMax)) return false;
            if (!CheckAxis(origin.y, direction.y, bounds.Min.y, bounds.Max.y, ref tMin, ref tMax)) return false;
            if (!CheckAxis(origin.z, direction.z, bounds.Min.z, bounds.Max.z, ref tMin, ref tMax)) return false;
            return true;

            //检查每一个分量的轴是否与Box的start end相交
            bool CheckAxis(FP o, FP d, FP min, FP max, ref FP s, ref FP e)
            {
                if (d == FP.Zero)
                {
                    return o >= min && o <= max;
                }
                FP l = FP.SafeDiv(min - o, d);
                FP r = FP.SafeDiv(max - o, d);

                //确保进入时间小于出去时间
                if (l > r) { FP t = l; l = r; r = t; }

                //与已有时间取交集
                if (l > s) s = l;
                if (r < e) e = r;
                return s <= e;
            }
        }

        //射线与球体
        public static bool Raycast(
            FPVector3 origin, FPVector3 direction, FP distance,
            FPSphere sphere, FPTransform trans,
            out FP length, out FPVector3 normal)
        {
            length = FP.Zero;
            normal = FPVector3.Zero;

            if (distance < FP.Zero || direction == FPVector3.Zero)
                return false;

            FPVector3 s = trans.Position - origin;
            FP rem = FPVector3.Magnitude(s) - sphere.Radius;

            //如果射线起点在球体内部 
            if (rem <= FP.Zero)
            {
                length = FP.Zero;
                normal = -direction;
                return true;
            }

            //如果射线方向背离球心
            //d = Dot(s,direction);
            FP d = FPVector3.Dot(s, direction);
            if (d <= FP.Zero)
                return false;

            //计算圆心到射线的投影距离
            FPVector3 p = s - direction * d;
            FP h = FPVector3.Magnitude(p);
            if (h > sphere.Radius)
                return false;

            //利用平方差公式 (R - h) * (R + h) 计算半弦长
            FP offset = FP.Sqrt((sphere.Radius - h) * (sphere.Radius + h));

            //判断射线长度是否足够
            length = d - offset;
            if (distance < length)
                return false;

            FPVector3 hit = origin + direction * length;
            normal = (hit - trans.Position) / sphere.Radius;
            return true;
        }
        //射线与Box
        public static bool Raycast(
            FPVector3 origin, FPVector3 direction, FP distance,
            FPBox box, FPTransform trans,
            out FP length, out FPVector3 normal)
        {
            length = FP.Zero;
            normal = FPVector3.Zero;

            if (distance < FP.Zero || direction == FPVector3.Zero)
                return false;

            //保留世界空间下的归一化射线方向direction
            direction = FPVector3.Normalize(direction);

            //并转换到局部坐标系中
            origin = trans.ToLocalPos(origin);
            FPVector3 local = trans.ToLocalDir(direction);
            FPVector3 ext = box.Extents;

            FP tMin = FP.Zero;
            FP tMax = distance;


            if (!Check(origin.x, local.x, -ext.x, ext.x, FPVector3.Left, ref tMin, ref tMax, ref normal)) return false;
            if (!Check(origin.y, local.y, -ext.y, ext.y, FPVector3.Down, ref tMin, ref tMax, ref normal)) return false;
            if (!Check(origin.z, local.z, -ext.z, ext.z, FPVector3.Back, ref tMin, ref tMax, ref normal)) return false;

            length = tMin;
            //若起点在盒外则将局部面法线转回世界空间
            //若起点在盒内则返回世界空间的direction
            normal = (tMin > FP.Zero) ? trans.ToWorldDir(normal) : -direction;
            return true;

            //s-进入时间 e-退出时间
            //o-起点 v-当前轴分量上速度 min-当前轴起点 max-当前轴终点
            //axis-当前轴方向上的法线 n-接触面的法向量
            bool Check(FP o, FP v, FP min, FP max, FPVector3 axis, ref FP s, ref FP e, ref FPVector3 n)
            {
                if (v == FP.Zero)
                {
                    return o >= min && o <= max;
                }

                FP l = FP.SafeDiv(min - o, v);
                FP r = FP.SafeDiv(max - o, v);

                //纠正方向
                if (l > r)
                {
                    FP temp = l; l = r; r = temp;
                    //翻转法线
                    axis = -axis;
                }

                //与窗口做交集 并记录碰撞面的法线
                if (l > s) { s = l; n = axis; }
                if (r < e) { e = r; }
                return s <= e;
            }
        }
        //射线与任意凸Shape
        public static bool RaycastAny(
            GJKEPA g,
            FPVector3 origin, FPVector3 direction, FP distance,
            FPShape shape, FPTransform trans,
            out FP length, out FPVector3 normal)
        {
            return g.Raycast(origin, direction, distance, shape, trans, out length, out normal);
        }
    }
}