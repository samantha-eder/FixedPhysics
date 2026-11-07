using FixedMath;

namespace FixedPhysics
{
    public enum ShapeType
    {
        Sphere,
        Box,
        Capsule,
        Convex,
    }
    public abstract class FPShape
    {
        public readonly ShapeType Type;
        protected FPShape(ShapeType type)
        {
            Type = type;
        }

        //给定一个世界空间的 Transform 计算出这个形状在世界中的 AABB 包围盒
        public abstract FPBounds ComputeBounds(FPTransform transform);

        //给定一个世界方向 返回该形状在该世界方向上的最远世界坐标点 (GJ K 算法)
        public abstract FPVector3 GetSupportPoint(FPTransform transform, FPVector3 direction);
    }
}