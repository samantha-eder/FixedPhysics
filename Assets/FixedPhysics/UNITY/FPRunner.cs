using FixedMath;
using UnityEngine;
using System.Collections.Generic;
using System;

namespace FixedPhysics.Bridge
{
    public enum BroadPhaseMode
    {
        SpatialHash,
        Octree,
        BVH
    }
    [DefaultExecutionOrder(100)]
    public class FPRunner : MonoBehaviour
    {
        public static FPRunner Instance { get; private set; }

        [Header("粗相空间划分算法切换")]
        public BroadPhaseMode Mode = BroadPhaseMode.BVH;
        public BroadPhaseMode CurrentMode;
        public bool DrawBroadPhase = true;

        [Range(1, 4)]
        public int SolverCount = 2;

        [Header("算法参数配置")]
        public float HashCellSize = 5.0f;
        public Vector3 OctreeWorldSize = new Vector3(50f, 50f, 50f);

        public PhysicsWorld World { get; private set; }
        public readonly List<FPBody> Bodies = new List<FPBody>();

        //算法耗时测试
        public float StepTime;
        private System.Diagnostics.Stopwatch _watch = new System.Diagnostics.Stopwatch();

        private void Awake()
        {
            if (Instance != null && Instance != this)
            {
                Destroy(this);
                return;
            }
            Instance = this;

            //创建初始粗相加速器与物理世界
            CurrentMode = Mode;
            World = new PhysicsWorld(CreateBroadPhase(Mode), SolverCount);

            //绑定碰撞事件
            World.OnCollisionEnter += (pair) =>
            {
                AddHitCount(pair.A, 1);
                AddHitCount(pair.B, 1);
            };
            World.OnCollisionExit += (pair) =>
            {
                AddHitCount(pair.A, -1);
                AddHitCount(pair.B, -1);
            };

            //收集场景中已激活的所有 FPBody 并注册进物理世界
            FPBody[] Bodies = FindObjectsByType<FPBody>();
            foreach (FPBody body in Bodies)
            {
                if (body.isActiveAndEnabled)
                {
                    Register(body);
                }
            }
        }

        public void Register(FPBody body)
        {
            if (body == null || Bodies.Contains(body)) return;

            //初始化Body
            body.Init();

            //将Body的Coliders中的所有FPCollider加入FPPhysicsWorld管理器
            foreach (var item in body.Colliders)
            {
                World.Add(item.Collider);
            }

            //添加进全局Body中
            Bodies.Add(body);
        }
        public void Unregister(FPBody body)
        {
            if (body == null) return;

            if (Bodies.Remove(body))
            {
                foreach (var item in body.Colliders)
                {
                    if (item == null || item.Collider == null)
                        continue;
                    World.Remove(item.Collider);
                    item.HitCount = 0;
                }
            }
        }

        private void AddHitCount(FPCollider collider, int delta)
        {
            if (collider.State != null && collider.State.UserData is FPBody body)
            {
                //根据collider所属的State得到UserData body
                //遍历挂载在该物体上的所有碰撞盒组件
                //找到当前collider所在的ColliderBase组件 
                foreach (var item in body.Colliders)
                {
                    if (item.Collider == collider)
                    {
                        //增加HitCount
                        item.HitCount = Math.Max(0, item.HitCount + delta);
                        break;
                    }
                }
            }
        }

        private IBroadPhase CreateBroadPhase(BroadPhaseMode mode)
        {
            switch (mode)
            {
                case BroadPhaseMode.SpatialHash:
                    return new SpatialHash(FPConvert.ToFP(HashCellSize));

                case BroadPhaseMode.Octree:
                    FPBounds worldBounds = FPBounds.CreateFromCenter(FPVector3.Zero, FPConvert.ToFP(OctreeWorldSize));
                    return new Octree(worldBounds);

                case BroadPhaseMode.BVH:
                default: return new BVH();
            }
        }

        private void FixedUpdate()
        {
            if (Mode != CurrentMode)
            {
                CurrentMode = Mode;
                World.SetBroadPhase(CreateBroadPhase(Mode));
            }
            World.SolverCount = SolverCount;

            //驱动定点数物理世界完成本帧计算
            //并测量定点数的单步耗时 (毫秒)
            _watch.Restart();
            World.Step();
            _watch.Stop();
            StepTime = (float)_watch.Elapsed.TotalMilliseconds;

            //将定点数结果同步到 Unity 表现层
            for (int i = 0; i < Bodies.Count; i++)
            {
                Bodies[i].SyncToUnity();
            }
        }
        private void OnDestroy()
        {
            if (Instance == this)
            {
                Instance = null;
            }
        }
        private void OnDrawGizmos()
        {
            if (World != null && DrawBroadPhase)
            {
                World.Draw();
            }
        }
    }
}