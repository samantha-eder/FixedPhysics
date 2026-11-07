using FixedMath;
using System.Collections.Generic;
using UnityEngine;

namespace FixedPhysics.Bridge
{
    public class FPBody : MonoBehaviour
    {
        [Header("物理属性")]
        public bool IsStatic = false;

        [Header("冻结位置轴 (Constraints)")]
        public bool FreezeX = false;
        public bool FreezeY = false;
        public bool FreezeZ = false;

        //该物体唯一的定点数状态
        public FPState State { get; set; }

        //挂载在该物体上的所有碰撞盒组件
        public List<ColliderComponentBase> Colliders = new List<ColliderComponentBase>();


        private void Awake()
        {
            Init();
        }
        private void OnEnable()
        {
            if (FPRunner.Instance != null)
            {
                FPRunner.Instance.Register(this);
            }
        }
        private void OnDisable()
        {
            if (FPRunner.Instance != null)
            {
                FPRunner.Instance.Unregister(this);
            }
        }

        public void Init()
        {
            if (State != null) return;

            //初始化唯一的物体共享状态State
            FPTransform t = new FPTransform(
                FPConvert.ToFP(transform.position),
                FPConvert.ToFP(transform.rotation)
            );
            State = new FPState(t, IsStatic, this)
            {
                FreezeX = this.FreezeX,
                FreezeY = this.FreezeY,
                FreezeZ = this.FreezeZ
            };

            //收集自身所有FPCollider 绑定到State上
            GetComponentsInChildren(false, Colliders);
            foreach (var c in Colliders) c.Init(this);
        }

        //初始化Init时已将Unity坐标同步至FP
        //该方法仅测试时使用
        public void SyncToFP()
        {
            if (State == null) return;

            State.IsStatic = this.IsStatic;

            State.FreezeX = this.FreezeX;
            State.FreezeY = this.FreezeY;
            State.FreezeZ = this.FreezeZ;

            FPVector3 position = FPConvert.ToFP(transform.position);
            FPQuaternion rotation = FPConvert.ToFP(transform.rotation);
            State.SetTransform(new FPTransform(position, rotation));
        }
        public void SyncToUnity()
        {
            if (State == null) return;

            transform.position = FPConvert.ToUnity(State.Transform.Position);
            transform.rotation = FPConvert.ToUnity(State.Transform.Rotation);
        }
    }
}