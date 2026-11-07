using UnityEngine;

namespace FixedPhysics.Bridge
{
    public class FPSphereCollider : ColliderComponentBase
    {
        [Header("球体半径")]
        public float Radius = 0.5f;

        protected override FPShape CreateShape()
        {
            return new FPSphere(FPConvert.ToFP(Radius));
        }
        protected override void Draw()
        {
            Gizmos.DrawWireSphere(Vector3.zero, Radius);
        }
    }
}