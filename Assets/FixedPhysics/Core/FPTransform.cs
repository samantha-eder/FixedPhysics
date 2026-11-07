using FixedMath;

namespace FixedPhysics
{
    public struct FPTransform
    {
        public FPVector3 Position;
        public FPQuaternion Rotation;
        public static readonly FPTransform Identity = new FPTransform(FPVector3.Zero, FPQuaternion.Identity);

        public FPTransform(FPVector3 position, FPQuaternion rotation)
        {
            Position = position;
            Rotation = rotation;
        }

        #region 世界空间下的局部向量
        public FPVector3 Forward => Rotation * FPVector3.Forward;
        public FPVector3 Back => Rotation * FPVector3.Back;
        public FPVector3 Up => Rotation * FPVector3.Up;
        public FPVector3 Down => Rotation * FPVector3.Down;
        public FPVector3 Right => Rotation * FPVector3.Right;
        public FPVector3 Left => Rotation * FPVector3.Left;
        #endregion

        //将局部坐标转换为世界坐标
        public FPVector3 ToWorldPos(FPVector3 local)
        {
            return Position + (Rotation * local);
        }
        //将世界坐标转换为局部坐标
        public FPVector3 ToLocalPos(FPVector3 world)
        {
            FPVector3 dir = world - Position;
            return FPQuaternion.Inverse(Rotation) * dir;
        }
        //将局部向量转换为世界向量
        public FPVector3 ToWorldDir(FPVector3 local)
        {
            return Rotation * local;
        }
        //将世界向量转换为局部向量
        public FPVector3 ToLocalDir(FPVector3 world)
        {
            return FPQuaternion.Inverse(Rotation) * world;
        }
    }
}