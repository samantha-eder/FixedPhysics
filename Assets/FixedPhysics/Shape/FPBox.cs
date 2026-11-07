using FixedMath;

namespace FixedPhysics
{
    public class FPBox : FPShape
    {
        //半尺寸
        public FPVector3 Extents { get; private set; }

        public FPBox(FPVector3 extents) : base(ShapeType.Box)
        {
            Extents = new FPVector3(FP.Abs(extents.x), FP.Abs(extents.y), FP.Abs(extents.z));
        }

        public override FPBounds ComputeBounds(FPTransform transform)
        {
            //利用基向量投影的绝对值之和
            //拿到盒子当前的三个局部坐标轴在世界中的方向向量
            FPVector3 right = transform.Right;
            FPVector3 up = transform.Up;
            FPVector3 forward = transform.Forward;

            //将盒子的半宽(Extents.x) 半高(Extents.y) 半长(Extents.z)
            //分别投影到世界坐标系的 X、Y、Z 轴上并取绝对值
            FP x = FP.Abs(right.x) * Extents.x + FP.Abs(up.x) * Extents.y + FP.Abs(forward.x) * Extents.z;
            FP y = FP.Abs(right.y) * Extents.x + FP.Abs(up.y) * Extents.y + FP.Abs(forward.y) * Extents.z;
            FP z = FP.Abs(right.z) * Extents.x + FP.Abs(up.z) * Extents.y + FP.Abs(forward.z) * Extents.z;

            FPVector3 extents = new FPVector3(x, y, z);
            return new FPBounds(transform.Position - extents, transform.Position + extents);
        }
        public override FPVector3 GetSupportPoint(FPTransform transform, FPVector3 direction)
        {
            //首先将世界方向转化到局部坐标系
            direction = transform.ToLocalDir(direction);
            //在局部空间寻找当前direction的最远点
            FPVector3 point = new FPVector3(
                direction.x > FP.Zero ? Extents.x : -Extents.x,
                direction.y > FP.Zero ? Extents.y : -Extents.y,
                direction.z > FP.Zero ? Extents.z : -Extents.z
            );
            //转化回世界空间返回
            return transform.ToWorldPos(point);
        }
    }
}