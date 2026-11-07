using FixedMath;

namespace FixedPhysics
{
    public class FPCollider
    {
        private static uint _nextID = 10001;

        //碰撞盒ID
        public uint ID;
        public bool IsTrigger;

        //所属的物体的状态
        public FPState State;

        //碰撞盒的几何形状与局部偏移
        public FPShape Shape;
        public FPVector3 OffsetPosition;
        public FPQuaternion OffsetRotation;

        //碰撞盒的AABB包围盒
        public FPBounds Bounds;

        public static void ResetIdCounter() => _nextID = 10001;
        public FPCollider(FPState state, FPShape shape, FPVector3 offsetPosition) : this(state, shape, offsetPosition, FPQuaternion.Identity)
        {
        }
        public FPCollider(FPState state, FPShape shape, FPVector3 offsetPosition, FPQuaternion offsetRotation)
        {
            ID = _nextID++;

            State = state;
            Shape = shape;

            OffsetPosition = offsetPosition;
            OffsetRotation = offsetRotation;

            //绑定进FPState
            State.Add(this);
            UpdateBounds();
        }

        //直接调用其所属的共享物体状态State
        public void Move(FPVector3 delta)
        {
            State.Move(delta);
        }
        public void SetTransform(FPTransform transform)
        {
            State.SetTransform(transform);
        }

        //根据共享State的世界姿态 + 自身局部偏移
        //计算出碰撞形状在世界空间下的实际姿态
        public FPTransform GetShapeWorldTransform()
        {
            FPTransform t = State.Transform;
            if (OffsetPosition == FPVector3.Zero && OffsetRotation == FPQuaternion.Identity)
            {
                return t;
            }

            //根据共享State的Transform
            //将碰撞盒的Tranform(中心点/旋转)转换到世界空间
            FPVector3 worldPos = t.ToWorldPos(OffsetPosition);
            FPQuaternion worldRot = t.Rotation * OffsetRotation;
            return new FPTransform(worldPos, worldRot);
        }
        //当物体的Transform发生改变后必须调用此方法刷新包围盒
        public void UpdateBounds()
        {
            FPTransform transform = GetShapeWorldTransform();
            Bounds = Shape.ComputeBounds(transform);
        }
    }
}