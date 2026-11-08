using FixedMath;
using System.Collections.Generic;

namespace FixedPhysics
{
    //物体的唯一定点数状态
    //被同一物体上的多个 FPCollider 共享
    public class FPState
    {
        //物体的属性 触发器 static
        public bool IsStatic;

        //绑定外部的游戏实体 与 世界坐标系下的Transform 记录了位置与旋转
        public object UserData;
        public FPTransform Transform;

        //冻结轴向位移
        public bool FreezeX;
        public bool FreezeY;
        public bool FreezeZ;

        //该物体身上所挂载的所有Collider碰撞盒
        public List<FPCollider> Colliders;

        public FPState(FPTransform transform, bool isStatic = false, object userData = null)
        {
            IsStatic = isStatic;

            UserData = userData;
            Transform = transform;

            Colliders = new List<FPCollider>(4);
        }

        public void Add(FPCollider collider)
        {
            if (Colliders.Contains(collider))
                return;
            Colliders.Add(collider);
        }
        public void Remove(FPCollider collider)
        {
            Colliders.Remove(collider);
        }

        //移动物体中心并同步所有碰撞盒
        public void Move(FPVector3 delta)
        {
            FP dx = FreezeX ? FP.Zero : delta.x;
            FP dy = FreezeY ? FP.Zero : delta.y;
            FP dz = FreezeZ ? FP.Zero : delta.z;

            FPVector3 move = new FPVector3(dx, dy, dz);
            Transform = new FPTransform(Transform.Position + move, Transform.Rotation);

            for (int i = 0; i < Colliders.Count; i++)
            {
                FPBounds bound = Colliders[i].Bounds;
                Colliders[i].Bounds = new FPBounds(bound.Min + move, bound.Max + move);
            }
        }
        //设置物体姿态并同步所有碰撞盒
        public void SetTransform(FPTransform transform)
        {
            Transform = transform;
            UpdateColliders();
        }

        public void UpdateColliders()
        {
            for (int i = 0; i < Colliders.Count; i++)
            {
                Colliders[i].UpdateBounds();
            }
        }
    }
}