using UnityEngine;

namespace FixedPhysics.Bridge
{
    public class FPBoxCollider : ColliderComponentBase
    {
        [Header("盒体尺寸")]
        public Vector3 Size = Vector3.one;

        protected override FPShape CreateShape()
        {
            return new FPBox(FPConvert.ToFP(Size * 0.5f));
        }
        protected override void Draw()
        {
            Gizmos.DrawWireCube(Vector3.zero, Size);
        }
    }
}