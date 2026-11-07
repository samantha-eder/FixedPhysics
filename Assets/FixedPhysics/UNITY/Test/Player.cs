using FixedMath;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.InputSystem;

namespace FixedPhysics.Bridge
{
    [RequireComponent(typeof(FPBody))]
    public class Player : MonoBehaviour
    {
        [Header("移动与旋转速度")]
        public float MoveSpeed = 5.0f;
        public float RotateSpeed = 120.0f;

        [Header("测试配置 (左键发射射线 / 右键开启范围检测)")]
        public float MaxDistance = 15.0f;
        public Vector3 AOESize = new Vector3(4.0f, 2.0f, 4.0f);
        public float PushForce = 0.2f;

        //朝向与Body
        public FP Yaw { get; set; }
        public FPBody Body { get; set; }

        //射线结果与范围检测结果查询
        private List<RayHit> _rayHits = new List<RayHit>(32);
        private List<FPCollider> _aoeHits = new List<FPCollider>(32);

        //是否发射射线
        private bool _isRay;
        private bool _isHit;
        private RayHit _closestHit;

        //是否范围检测
        private bool _isAOE;


        private void Awake()
        {
            Body = GetComponent<FPBody>();
        }
        private void Start()
        {
            Yaw = FPConvert.ToFP(transform.eulerAngles.y);
        }
        private void FixedUpdate()
        {
            if (Body == null || Body.State == null || FPRunner.Instance == null)
                return;

            Keyboard board = Keyboard.current;
            Mouse mouse = Mouse.current;

            FPState state = Body.State;
            FP dt = FP.FromString("0.02");

            //Q / E 键旋转
            int turn = 0;
            if (board.eKey.isPressed) turn += 1;
            if (board.qKey.isPressed) turn -= 1;

            if (turn != 0)
            {
                FP rotSpeed = FPConvert.ToFP(RotateSpeed);
                Yaw += FP.FromInt(turn) * (rotSpeed * dt);
                FPQuaternion rot = FPQuaternion.Euler(FP.Zero, Yaw, FP.Zero);
                state.SetTransform(new FPTransform(state.Transform.Position, rot));
            }

            //W / A / S / D 移动
            int moveX = 0;
            int moveZ = 0;

            if (board.dKey.isPressed) moveX += 1;
            if (board.aKey.isPressed) moveX -= 1;
            if (board.wKey.isPressed) moveZ += 1;
            if (board.sKey.isPressed) moveZ -= 1;

            if (moveX != 0 || moveZ != 0)
            {
                FPTransform cur = state.Transform;
                FPVector3 dir = FPVector3.Normalize(cur.Right * FP.FromInt(moveX) + cur.Forward * FP.FromInt(moveZ));
                FP speed = FPConvert.ToFP(MoveSpeed);
                state.Move(dir * (speed * dt));
            }

            //按住鼠标左键 射线检测
            _isRay = mouse != null && mouse.leftButton.isPressed;
            if (_isRay)
            {
                RunRay();
            }

            //按住鼠标右键 范围检测
            _isAOE = mouse != null && mouse.rightButton.isPressed;
            if (_isAOE)
            {
                RunAOE();
            }
        }
        private void OnDrawGizmos()
        {
            if (!Application.isPlaying || Body == null || Body.State == null)
                return;

            //绘制短绿线指示主角当前正前方Forward
            Vector3 position = FPConvert.ToUnity(Body.State.Transform.Position);
            Vector3 forward = FPConvert.ToUnity(Body.State.Transform.Forward);
            Gizmos.color = Color.green;
            Gizmos.DrawLine(position, position + forward * 1.5f);

            //绘制激光射线与命中点法线
            if (_isRay && _isHit)
            {
                Vector3 hitPoint = FPConvert.ToUnity(_closestHit.Point);
                Vector3 hitNormal = FPConvert.ToUnity(_closestHit.Normal);

                Gizmos.color = Color.red;
                Gizmos.DrawLine(position, hitPoint);
                Gizmos.DrawSphere(hitPoint, 0.12f);

                Gizmos.color = Color.yellow;
                Gizmos.DrawLine(hitPoint, hitPoint + hitNormal * 1.0f);
            }

            //绘制右键的冲击波检测指示框
            if (_isAOE)
            {
                Gizmos.color = Color.magenta;
                Gizmos.matrix = Matrix4x4.identity;
                Gizmos.DrawWireCube(position, AOESize);
            }
        }

        private void RunRay()
        {
            PhysicsWorld world = FPRunner.Instance.World;

            FPTransform transform = Body.State.Transform;
            FP distance = FPConvert.ToFP(MaxDistance);

            world.QueryRay(transform.Position, transform.Forward, distance, _rayHits);

            _isHit = false;
            for (int i = 0; i < _rayHits.Count; i++)
            {
                //找到排在最前面的且不属于物体自己的碰撞体
                if (_rayHits[i].Collider.State != Body.State && !_rayHits[i].Collider.IsTrigger)
                {
                    _isHit = true;
                    _closestHit = _rayHits[i];

                    ////顺着射线方向把它推开
                    //if (!_closestHit.Collider.State.IsStatic)
                    //{
                    //    _closestHit.Collider.Move(transform.Forward * FPConvert.ToFP(PushForce));
                    //}
                    break;
                }
            }
        }
        private void RunAOE()
        {
            PhysicsWorld world = FPRunner.Instance.World;

            FPVector3 center = Body.State.Transform.Position;
            FPBounds bounds = FPBounds.CreateFromCenter(center, FPConvert.ToFP(AOESize));

            world.QueryAABB(bounds, _aoeHits);

            for (int i = 0; i < _aoeHits.Count; i++)
            {
                FPCollider col = _aoeHits[i];
                if (col.State == Body.State || col.State.IsStatic || col.IsTrigger)
                    continue;

                //把进入范围框内的所有动态敌人往外推开
                FPVector3 pushDir = FPVector3.Normalize(col.State.Transform.Position - center);
                col.Move(pushDir * FPConvert.ToFP(PushForce));
            }
        }
    }
}