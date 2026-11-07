using FixedMath;

namespace FixedPhysics
{
    public class FPCapsule : FPShape
    {
        //半球的半径 圆柱的半高
        public FP Radius { get; private set; }
        public FP Height { get; private set; }

        public FPCapsule(FP radius, FP height) : base(ShapeType.Capsule)
        {
            Radius = radius;
            Height = height;
        }

        public override FPBounds ComputeBounds(FPTransform transform)
        {
            //计算世界空间下胶囊体上下两个球心坐标
            FPVector3 offset = transform.Up * Height;
            FPVector3 a = transform.Position + offset;
            FPVector3 b = transform.Position - offset;

            //向外扩展Radius
            FPVector3 r = new FPVector3(Radius, Radius, Radius);
            FPVector3 min = FPVector3.Min(a, b) - r;
            FPVector3 max = FPVector3.Max(a, b) + r;

            return new FPBounds(min, max);
        }

        public override FPVector3 GetSupportPoint(FPTransform transform, FPVector3 direction)
        {
            //判断direction与胶囊体Y轴夹角 
            FP dot = FPVector3.Dot(direction, transform.Up);

            //选择direction方向的球心
            FPVector3 offset = transform.Up * Height;
            FPVector3 center = (dot >= FP.Zero) ?
                               transform.Position + offset :
                               transform.Position - offset;

            return center + FPVector3.Normalize(direction) * Radius;
        }
    }
}