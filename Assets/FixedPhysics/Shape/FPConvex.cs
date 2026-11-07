using FixedMath;
using System;

namespace FixedPhysics
{
    public class FPConvex : FPShape
    {
        //凸包顶点数组
        public FPVector3[] Vertices;

        public FPConvex(FPVector3[] vertices) : base(ShapeType.Convex)
        {
            if (vertices == null || vertices.Length == 0)
                throw new ArgumentException("凸多面体顶点数组不能为空");

            Vertices = (FPVector3[])vertices.Clone();
        }

        public override FPBounds ComputeBounds(FPTransform transform)
        {
            FPVector3 min = transform.ToWorldPos(Vertices[0]);
            FPVector3 max = transform.ToWorldPos(Vertices[0]);

            foreach (FPVector3 item in Vertices)
            {
                FPVector3 position = transform.ToWorldPos(item);
                min = FPVector3.Min(min, position);
                max = FPVector3.Max(max, position);
            }

            return new FPBounds(min, max);
        }

        public override FPVector3 GetSupportPoint(FPTransform transform, FPVector3 direction)
        {
            //将世界方向转回局部坐标系
            direction = transform.ToLocalDir(direction);

            //寻找direction方向上 Dot最大的顶点
            FPVector3 vertex = Vertices[0];
            FP max = FPVector3.Dot(vertex, direction);

            for (int i = 1; i < Vertices.Length; i++)
            {
                FP dot = FPVector3.Dot(Vertices[i], direction);
                if (dot > max)
                {
                    max = dot;
                    vertex = Vertices[i];
                }
            }

            //将找到的局部最远极点转回世界空间
            return transform.ToWorldPos(vertex);
        }
    }
}