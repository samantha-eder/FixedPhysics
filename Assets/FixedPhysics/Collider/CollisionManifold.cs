using FixedMath;

namespace FixedPhysics
{
    public struct CollisionManifold
    {
        public bool IsHit;
        public FP Penetration;
        public FPVector3 Normal;

        public CollisionManifold(bool isHit, FP penetration, FPVector3 normal)
        {
            this.IsHit = isHit;
            this.Penetration = penetration;
            this.Normal = normal;
        }

        //返回一个未碰撞的空对象
        public static CollisionManifold NoHit()
        {
            return new CollisionManifold
            {
                IsHit = false,
                Penetration = FP.Zero,
                Normal = FPVector3.Zero
            };
        }
    }
}