using FixedMath;

namespace FixedPhysics
{
    public struct FPBounds
    {
        public FPVector3 Min;
        public FPVector3 Max;

        public FPVector3 Center => (Min + Max) * FP.Half;
        public FPVector3 Size => Max - Min;
        public FPVector3 Extents => (Max - Min) * FP.Half;

        public FPBounds(FPVector3 min, FPVector3 max)
        {
            Min = FPVector3.Min(min, max);
            Max = FPVector3.Max(min, max);
        }
        public static FPBounds CreateFromCenter(FPVector3 center, FPVector3 size)
        {
            FPVector3 extents = new FPVector3(FP.Abs(size.x), FP.Abs(size.y), FP.Abs(size.z)) * FP.Half;
            return new FPBounds(center - extents, center + extents);
        }


        //判断两个 AABB 是否相交
        public bool Intersects(FPBounds other)
        {
            if (Max.x < other.Min.x || Min.x > other.Max.x) return false;
            if (Max.y < other.Min.y || Min.y > other.Max.y) return false;
            if (Max.z < other.Min.z || Min.z > other.Max.z) return false;
            return true;
        }

        //判断该 AABB 是否包含一个点或者另一个包围盒
        public bool Contains(FPVector3 point)
        {
            return Min.x <= point.x && Max.x >= point.x &&
                   Min.y <= point.y && Max.y >= point.y &&
                   Min.z <= point.z && Max.z >= point.z;
        }
        public bool Contains(FPBounds other)
        {
            return Min.x <= other.Min.x && Max.x >= other.Max.x &&
                   Min.y <= other.Min.y && Max.y >= other.Max.y &&
                   Min.z <= other.Min.z && Max.z >= other.Max.z;
        }


        //扩展包围盒以包含某一个特定的点或者另一个包围盒
        public void Encapsulate(FPVector3 point)
        {
            Min = FPVector3.Min(Min, point);
            Max = FPVector3.Max(Max, point);
        }
        public void Encapsulate(FPBounds other)
        {
            Min = FPVector3.Min(Min, other.Min);
            Max = FPVector3.Max(Max, other.Max);
        }

        //得到包围盒的表面积
        public FP GetSurface()
        {
            FP x = Max.x - Min.x;
            FP y = Max.y - Min.y;
            FP z = Max.z - Min.z;

            FP xy = x * y;
            FP yz = y * z;
            FP zx = z * x;

            return xy + xy + yz + yz + zx + zx;
        }
        //合并两个包围盒
        public static FPBounds Union(FPBounds a, FPBounds b)
        {
            return new FPBounds(FPVector3.Min(a.Min, b.Min), FPVector3.Max(a.Max, b.Max));
        }

        public bool Equals(FPBounds other)
        {
            return Min == other.Min && Max == other.Max;
        }
        public override string ToString()
        {
            return $"Bounds: Min={Min}, Max={Max}";
        }
    }
}