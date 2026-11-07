using FixedMath;
using System;
using System.Collections.Generic;

namespace FixedPhysics
{
    /// <summary>
    /// 粗相空间划分统一接口
    /// </summary>
    public interface IBroadPhase
    {
        //碰撞体加入时调用
        void Add(FPCollider collider);

        //碰撞体移除时调用
        void Remove(FPCollider collider);

        //清空
        void Clear();

        //每帧调用来更新空间结构
        //colliders为当前世界中所有激活的碰撞体列表
        void Update(IReadOnlyList<FPCollider> colliders);

        //计算并收集当前帧所有AABB碰撞盒相交的潜在碰撞对
        //并将结果填入results列表
        void GetPotentialPairs(List<ColliderPair> results);

        //找出所有与指定AABB盒子bounds相交的碰撞体
        //并将结果保存在results数组中
        void QueryAABB(FPBounds bounds, List<FPCollider> results);

        //找出射线路径上所有碰到的碰撞体
        //参数为: 射线起点 归一化的射线方向 最大检测距离 结果接收列表
        void QueryRay(FPVector3 origin, FPVector3 direction, FP maxDistance, List<FPCollider> results);

        //调试时使用
        //在Unity Scene视图中绘制空间划分结构
        void Draw();
    }
    public readonly struct ColliderPair : IEquatable<ColliderPair>, IComparable<ColliderPair>
    {
        public readonly FPCollider A;
        public readonly FPCollider B;
        //唯一64位组合Key
        public readonly ulong PairKey;

        public ColliderPair(FPCollider a, FPCollider b)
        {
            //强制编号从小到大 对AB-BA去重
            if (a.ID < b.ID)
            {
                A = a;
                B = b;
            }
            else
            {
                A = b;
                B = a;
            }

            //将AB的Collider ID组合为唯一的64位ulong PairKey
            PairKey = ((ulong)A.ID << 32) | B.ID;
        }

        public bool Equals(ColliderPair other)
        {
            return PairKey == other.PairKey;
        }
        public int CompareTo(ColliderPair other)
        {
            return PairKey.CompareTo(other.PairKey);
        }

        public override bool Equals(object obj)
        {
            return obj is ColliderPair other && Equals(other);
        }
        public override int GetHashCode()
        {
            return PairKey.GetHashCode();
        }
        public override string ToString()
        {
            return $"Pair({A.ID}, {B.ID})";
        }
    }
}