using FixedMath;
using System;

namespace FixedPhysics
{
    public struct RayHit : IComparable<RayHit>
    {
        public FPCollider Collider;
        public FP Length;
        public FPVector3 Point;
        public FPVector3 Normal;

        public RayHit(FPCollider collider, FP length, FPVector3 point, FPVector3 normal)
        {
            Collider = collider;
            Length = length;
            Point = point;
            Normal = normal;
        }

        //按射线命中距离从小到大排序
        //距离相同时按FPCollider ID排序保证确定性
        public int CompareTo(RayHit other)
        {
            int c = Length.CompareTo(other.Length);
            return c != 0 ? c : Collider.ID.CompareTo(other.Collider.ID);
        }
    }
}