using FixedMath;

namespace FixedPhysics
{
    public class FPSphere : FPShape
    {
        public FP Radius { get; private set; }

        public FPSphere(FP radius) : base(ShapeType.Sphere)
        {
            Radius = radius;
        }

        public override FPBounds ComputeBounds(FPTransform transform)
        {
            FPVector3 extents = new FPVector3(Radius, Radius, Radius);
            return new FPBounds(transform.Position - extents, transform.Position + extents);
        }
        public override FPVector3 GetSupportPoint(FPTransform transform, FPVector3 direction)
        {
            //返回球心 + 归一化方向*半径
            return transform.Position + FPVector3.Normalize(direction) * Radius;
        }
    }
}