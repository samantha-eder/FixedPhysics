using FixedMath;
using UnityEngine;
using System.Globalization;

namespace FixedPhysics.Bridge
{
    public static class FPConvert
    {
        //定点数转UNITY浮点数
        public static float ToUnity(FP value)
        {
            return value.ToFloat();
        }
        public static Vector3 ToUnity(FPVector3 value)
        {
            return new Vector3(value.x.ToFloat(), value.y.ToFloat(), value.z.ToFloat());
        }
        public static Quaternion ToUnity(FPQuaternion quaternion)
        {
            return new Quaternion(quaternion.x.ToFloat(), quaternion.y.ToFloat(), quaternion.z.ToFloat(), quaternion.w.ToFloat());
        }

        //UNITY浮点数转定点数 
        //注 : 仅读取Inspector使用
        public static FP ToFP(float v)
        {
            return FP.FromString(v.ToString("F4", CultureInfo.InvariantCulture));
        }
        public static FPVector3 ToFP(Vector3 v)
        {
            return new FPVector3(ToFP(v.x), ToFP(v.y), ToFP(v.z));
        }
        public static FPQuaternion ToFP(Quaternion q)
        {
            return FPQuaternion.Normalize(new FPQuaternion(ToFP(q.x), ToFP(q.y), ToFP(q.z), ToFP(q.w)));
        }
    }
}