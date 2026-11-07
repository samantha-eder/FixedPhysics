using FixedMath;
using System.Collections.Generic;
using UnityEngine;

namespace FixedPhysics.Bridge
{
    public class FPConvexCollider : ColliderComponentBase
    {
        [Header("自定义网格 默认生成八面体钻石")]
        public Mesh Mesh;
        public float Radius = 0.5f;

        protected override FPShape CreateShape()
        {
            Mesh m = Mesh != null ? Mesh : (TryGetComponent(out MeshFilter mf) ? mf.sharedMesh : null);
            if (m != null && m.vertexCount > 0)
            {
                Vector3[] raw = m.vertices;
                Vector3 scale = transform.localScale;
                List<FPVector3> verts = new List<FPVector3>(raw.Length);
                for (int i = 0; i < raw.Length; i++)
                {
                    FPVector3 v = FPConvert.ToFP(Vector3.Scale(raw[i], scale));
                    if (!verts.Contains(v)) verts.Add(v);
                }
                return new FPConvex(verts.ToArray());
            }

            FP r = FPConvert.ToFP(Radius);
            return new FPConvex(new FPVector3[]
            {
                new FPVector3(FP.Zero, r, FP.Zero), new FPVector3(FP.Zero, -r, FP.Zero),
                new FPVector3(r, FP.Zero, FP.Zero), new FPVector3(-r, FP.Zero, FP.Zero),
                new FPVector3(FP.Zero, FP.Zero, r), new FPVector3(FP.Zero, FP.Zero, -r)
            });
        }
        protected override void Draw()
        {
            Mesh m = Mesh != null ? Mesh : (TryGetComponent(out MeshFilter mf) ? mf.sharedMesh : null);
            if (m != null)
            {
                Gizmos.DrawWireMesh(m, Vector3.zero, Quaternion.identity, transform.localScale);
            }
            else
            {
                Vector3 top = new Vector3(0, Radius, 0), bot = new Vector3(0, -Radius, 0);
                Vector3[] ring = { new Vector3(Radius, 0, 0), new Vector3(0, 0, Radius), new Vector3(-Radius, 0, 0), new Vector3(0, 0, -Radius) };
                for (int i = 0; i < 4; i++)
                {
                    Gizmos.DrawLine(top, ring[i]);
                    Gizmos.DrawLine(bot, ring[i]);
                    Gizmos.DrawLine(ring[i], ring[(i + 1) % 4]);
                }
            }
        }
    }
}