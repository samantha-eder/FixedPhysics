using UnityEngine;

namespace FixedPhysics.Bridge
{
    public class FPCapsuleCollider : ColliderComponentBase
    {
        [Header("胶囊体半径与半高")]
        public float Radius = 0.5f;
        public float Height = 0.5f;

        protected override FPShape CreateShape()
        {
            return new FPCapsule(FPConvert.ToFP(Radius), FPConvert.ToFP(Height));
        }
        protected override void Draw()
        {
            Vector3 top = new Vector3(0f, Height, 0f), bot = new Vector3(0f, -Height, 0f);
            Gizmos.DrawWireSphere(top, Radius);
            Gizmos.DrawWireSphere(bot, Radius);
            Gizmos.DrawLine(top + new Vector3(Radius, 0, 0), bot + new Vector3(Radius, 0, 0));
            Gizmos.DrawLine(top + new Vector3(-Radius, 0, 0), bot + new Vector3(-Radius, 0, 0));
            Gizmos.DrawLine(top + new Vector3(0, 0, Radius), bot + new Vector3(0, 0, Radius));
            Gizmos.DrawLine(top + new Vector3(0, 0, -Radius), bot + new Vector3(0, 0, -Radius));
        }
    }
}