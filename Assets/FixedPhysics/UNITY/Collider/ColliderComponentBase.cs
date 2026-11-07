using FixedMath;
using UnityEngine;

namespace FixedPhysics.Bridge
{
    public abstract class ColliderComponentBase : MonoBehaviour
    {
        [Header("碰撞盒属性")]
        public bool IsTrigger = false;
        public bool ShowAABB = false;

        [Header("局部偏移")]
        public Vector3 Center = Vector3.zero;
        public Vector3 Euler = Vector3.zero;

        [HideInInspector]
        public int HitCount = 0;

        //当前碰撞盒的FPCollider与共享物体状态State
        public FPBody Body { get; set; }
        public FPCollider Collider { get; set; }

        protected abstract FPShape CreateShape();
        protected abstract void Draw();

        //由所属的FPBody调用
        //创建底层FPCollider并指向共享的State
        public void Init(FPBody body)
        {
            if (Collider != null) return;

            //绑定Body
            Body = body;

            //如果挂在子物体上
            //自动折算相对于父级的局部偏移与旋转
            Vector3 localPos = transform == body.transform ?
                               Center :
                               body.transform.InverseTransformPoint(transform.TransformPoint(Center));

            Quaternion localRot = transform == body.transform ?
                                  Quaternion.Euler(Euler) :
                                  Quaternion.Inverse(body.transform.rotation) * (transform.rotation * Quaternion.Euler(Euler));

            //转换到FP定点数并初始化底层FPCollider
            FPVector3 position = FPConvert.ToFP(localPos);
            FPQuaternion rotation = FPConvert.ToFP(localRot);
            Collider = new FPCollider(body.State, CreateShape(), position, rotation)
            {
                IsTrigger = this.IsTrigger
            };
        }

        private void OnDrawGizmos()
        {
            if (Collider == null)
            {
                //编辑器未运行时的实时预览绘制
                FPBody body = GetComponentInParent<FPBody>();

                if (IsTrigger) Gizmos.color = new Color(0.8f, 0.2f, 1f);
                else if (body != null && body.IsStatic) Gizmos.color = Color.cyan;
                else Gizmos.color = Color.green;

                Gizmos.matrix = Matrix4x4.TRS(transform.TransformPoint(Center), transform.rotation * Quaternion.Euler(Euler), Vector3.one);
                Draw();
                Gizmos.matrix = Matrix4x4.identity;
                return;
            }

            //同步Inspector面板上的IsTrigger开关
            Collider.IsTrigger = this.IsTrigger;

            if (ShowAABB)
            {
                Gizmos.color = new Color(1f, 0.9f, 0.1f, 0.35f);
                Gizmos.matrix = Matrix4x4.identity;
                Gizmos.DrawWireCube(FPConvert.ToUnity(Collider.Bounds.Center), FPConvert.ToUnity(Collider.Bounds.Size));
            }

            if (HitCount > 0) Gizmos.color = Color.red;
            else if (IsTrigger) Gizmos.color = new Color(0.8f, 0.2f, 1f);
            else if (Body != null && Body.IsStatic) Gizmos.color = Color.cyan;
            else Gizmos.color = Color.green;

            FPTransform shapeTrans = Collider.GetShapeWorldTransform();
            Gizmos.matrix = Matrix4x4.TRS(FPConvert.ToUnity(shapeTrans.Position), FPConvert.ToUnity(shapeTrans.Rotation), Vector3.one);

            Draw();
            Gizmos.matrix = Matrix4x4.identity;
        }
    }
}