using FixedMath;
using UnityEngine;
using UnityEngine.InputSystem;
using System.Collections.Generic;

namespace FixedPhysics.Bridge
{
    [DefaultExecutionOrder(0)]
    public class StressTest : MonoBehaviour
    {
        [Header("测试配置")]
        public uint Seed = 10086;
        public int InitialCount = 200;
        public int AddCount = 100;

        //场地半宽
        public float Size = 20.0f;

        //球体半径与移动速度
        public float Radius = 0.4f;
        public float MoveSpeed = 6.0f;

        //混合生成
        public bool Mix = true;
        public Material Material;

        [Header("可视化面板")]
        public bool StatsGUI = true;
        public int VerifyStep = 300;

        //定点数校验
        private int _step;
        private ulong _resHash;
        private ulong _curHash;
        private string _result = "等待到达校验step";

        //单个运动测试实体
        private struct Item
        {
            public GameObject Go;
            public FPState State;
            public FPVector3 Velocity;
            public ColliderComponentBase ColComp;
        }

        private FPRandom _r;
        private List<Item> _items = new List<Item>(1024);
        private bool _showGizmos = true;

        //耗时统计
        private float _avgTime;

        private void Start()
        {
            ResetItem(InitialCount);
        }
        private void Update()
        {
            Keyboard k = Keyboard.current;

            if (k == null || FPRunner.Instance == null) return;

            // q w e 切换三种粗相算法
            if (k.qKey.wasPressedThisFrame) { FPRunner.Instance.Mode = BroadPhaseMode.SpatialHash; ResetItem(_items.Count); }
            if (k.wKey.wasPressedThisFrame) { FPRunner.Instance.Mode = BroadPhaseMode.Octree; ResetItem(_items.Count); }
            if (k.eKey.wasPressedThisFrame) { FPRunner.Instance.Mode = BroadPhaseMode.BVH; ResetItem(_items.Count); }

            // g 键开关Gizmos线框绘制
            if (k.gKey.wasPressedThisFrame)
            {
                _showGizmos = !_showGizmos;
                FPRunner.Instance.DrawBroadPhase = _showGizmos;

                for (int i = 0; i < _items.Count; i++)
                {
                    if (_items[i].ColComp != null)
                        _items[i].ColComp.ShowGizmos = _showGizmos;
                }
            }

            //空格键追加生成物体
            if (k.spaceKey.wasPressedThisFrame) AddItem(AddCount);

            //R键同种子重置
            if (k.rKey.wasPressedThisFrame) ResetItem(InitialCount);
        }
        private void FixedUpdate()
        {
            if (FPRunner.Instance == null || FPRunner.Instance.World == null)
                return;

            _step++;
            FP dt = FP.FromString("0.02");
            FP limit = FPConvert.ToFP(Size - Radius - 0.8f);

            //驱动所有物体在空间内移动
            for (int i = 0; i < _items.Count; i++)
            {
                Item item = _items[i];
                if (item.State == null) continue;

                FPVector3 p = item.State.Transform.Position;
                FPVector3 v = item.Velocity;

                //遇到边界时将速度反向
                if ((p.x > limit && v.x > FP.Zero) || (p.x < -limit && v.x < FP.Zero)) v = new FPVector3(-v.x, v.y, v.z);
                if ((p.y > limit && v.y > FP.Zero) || (p.y < -limit && v.y < FP.Zero)) v = new FPVector3(v.x, -v.y, v.z);
                if ((p.z > limit && v.z > FP.Zero) || (p.z < -limit && v.z < FP.Zero)) v = new FPVector3(v.x, v.y, -v.z);

                item.Velocity = v;
                _items[i] = item;

                item.State.Move(v * dt);
            }

            //统计单步耗时
            float cur = FPRunner.Instance.StepTime;
            _avgTime = (_avgTime == 0f) ? cur : Mathf.Lerp(_avgTime, cur, 0.05f);

            //到达指定步数时计算所有物体的哈希码
            if (_step == VerifyStep)
            {
                _curHash = ComputeHash();
                if (_resHash == 0)
                {
                    _resHash = _curHash;
                    _result = $"0x{_curHash:X16} [记录初始哈希码]";
                }
                else if (_curHash == _resHash)
                {
                    _result = $"0x{_curHash:X16} [哈希码一致]";
                }
                else
                {
                    _result = $"0x{_curHash:X16} [哈希码不一致]";
                }
            }
        }

        private void OnGUI()
        {
            if (!StatsGUI || FPRunner.Instance == null || FPRunner.Instance.World == null)
                return;

            PhysicsWorld world = FPRunner.Instance.World;
            GUIStyle style = new GUIStyle(GUI.skin.box)
            {
                fontSize = 15,
                alignment = TextAnchor.UpperLeft,
                normal = { textColor = Color.white }
            };

            string gizo = _showGizmos ? "开启 (按G关闭)" : "已关闭";
            string info = $"当前粗相算法 : {FPRunner.Instance.CurrentMode}\n" +
                          $"激活碰撞体数 : {world.Colliders.Count} (步数: {_step})\n" +
                          $"粗筛候选对数 : {world.PotentialPairs.Count}\n" +
                          $"单步当前耗时 : {FPRunner.Instance.StepTime:F2} ms\n" +
                          $"单步平均耗时 : {_avgTime:F2} ms\n" +
                          $"Gizmos 线框  : {gizo}\n" +
                          $"第{VerifyStep}步指纹 : {_result}\n\n" +
                          $"[快捷键] Q:Hash | W:Octree | E:BVH | G:线框\n" +
                          $"Space: +{AddCount} 物体 | R: 重置验证确定性";
            GUI.Box(new Rect(15, 15, 360, 230), info, style);
        }


        //重置随机种子并重新生成指定数量的碰撞体
        public void ResetItem(int count)
        {
            ClearItem();

            //重置物理世界与ID计数器
            //确保每次重置从完全相同的起点开始
            if (FPRunner.Instance != null && FPRunner.Instance.World != null)
            {
                FPRunner.Instance.World.Clear();
                FPCollider.ResetIdCounter();
            }

            _r = new FPRandom(Seed);

            _avgTime = 0f;

            _step = 0;

            AddItem(count);
        }

        //清空所有Item
        public void ClearItem()
        {
            foreach (var item in _items)
            {
                if (item.Go != null)
                {
                    item.Go.SetActive(false);
                    Destroy(item.Go);
                }
            }
            _items.Clear();
        }

        //添加count数量的随机物体到场景中
        //并且设置随机的位置与速度信息
        public void AddItem(int count)
        {
            if (_r == null) _r = new FPRandom(Seed);

            for (int i = 0; i < count; i++)
            {
                Set(out FPTransform transform, out FPVector3 velocity);

                if (!Mix)
                {
                    AddSphere(transform, velocity);
                    continue;
                }

                //按序号轮流生成5种不同形状
                switch (_items.Count % 5)
                {
                    case 0: AddBox(transform, velocity); break;
                    case 1: AddSphere(transform, velocity); break;
                    case 2: AddCapsule(transform, velocity); break;
                    case 3: AddDiamond(transform, velocity); break;
                    case 4: AddCylinder(transform, velocity); break;
                }
            }
        }
        private void Set(out FPTransform transform, out FPVector3 velocity)
        {
            FP range = FPConvert.ToFP(Size - 2.0f);
            FP speed = FPConvert.ToFP(MoveSpeed);

            //随机三维坐标
            FP x = _r.Range(-range, range);
            FP y = _r.Range(-range, range);
            FP z = _r.Range(-range, range);

            //三维随机欧拉角旋转
            FP rx = _r.Range(FP.Zero, FP.FromInt(360));
            FP ry = _r.Range(FP.Zero, FP.FromInt(360));
            FP rz = _r.Range(FP.Zero, FP.FromInt(360));
            FPQuaternion quaternion = FPQuaternion.Euler(rx, ry, rz);

            //三维全向随机速度
            FP vx = _r.Range(FP.NegOne, FP.One);
            FP vy = _r.Range(FP.NegOne, FP.One);
            FP vz = _r.Range(FP.NegOne, FP.One);

            FPVector3 direction = FPVector3.Normalize(new FPVector3(vx, vy, vz));
            if (direction == FPVector3.Zero) direction = FPVector3.Up;

            transform = new FPTransform(new FPVector3(x, y, z), quaternion);
            velocity = direction * speed;
        }

        //生成具体形状的物体并绑定
        public void AddBox(FPTransform transform, FPVector3 velocity)
        {
            float cube = Radius * 1.6f;
            Vector3 size = new Vector3(cube, cube, cube);
            GameObject item = Create(PrimitiveType.Cube, "Box", size);

            FPBoxCollider collider = item.AddComponent<FPBoxCollider>();
            collider.Size = size;

            Bind(item, transform, velocity);
        }
        public void AddSphere(FPTransform transform, FPVector3 velocity)
        {
            float diameter = Radius * 2f;
            GameObject item = Create(PrimitiveType.Sphere, "Sphere", new Vector3(diameter, diameter, diameter));

            FPSphereCollider collider = item.AddComponent<FPSphereCollider>();
            collider.Radius = Radius;

            Bind(item, transform, velocity);
        }
        public void AddCapsule(FPTransform transform, FPVector3 velocity)
        {
            float radius = Radius * 0.8f;
            float height = Radius * 0.8f;
            GameObject item = Create(PrimitiveType.Capsule, "Capsule", new Vector3(radius * 2f, height + radius, radius * 2f));

            FPCapsuleCollider collider = item.AddComponent<FPCapsuleCollider>();
            collider.Radius = radius;
            collider.Height = height;

            Bind(item, transform, velocity);
        }
        public void AddDiamond(FPTransform transform, FPVector3 velocity)
        {
            //内接小Cube方便观察
            //碰撞盒采用外接八面体钻石
            float cube = Radius * 1.1f;
            float radius = Radius * 1.3f;
            GameObject item = Create(PrimitiveType.Cube, "Diamond", new Vector3(cube, cube, cube));

            //留空mesh FPConvexCollider内部自动生成八面体钻石
            FPConvexCollider collider = item.AddComponent<FPConvexCollider>();
            collider.Radius = radius;
            collider.Mesh = null;

            Bind(item, transform, velocity);
        }
        public void AddCylinder(FPTransform transform, FPVector3 velocity)
        {
            float radius = Radius * 0.9f;
            GameObject item = Create(PrimitiveType.Cylinder, "Cylinder", new Vector3(radius * 2f, radius, radius * 2f));

            //将圆柱体自带的完整Mesh赋给FPConvexCollider
            FPConvexCollider collider = item.AddComponent<FPConvexCollider>();
            collider.Mesh = item.GetComponent<MeshFilter>().sharedMesh;

            Bind(item, transform, velocity);
        }

        private GameObject Create(PrimitiveType type, string prefix, Vector3 scale)
        {
            GameObject item = GameObject.CreatePrimitive(type);

            item.name = $"{prefix}_{_items.Count}";
            item.transform.SetParent(transform);

            //移除自带的Collider组件
            DestroyImmediate(item.GetComponent<Collider>());

            //设置材质
            if (Material != null && item.TryGetComponent(out MeshRenderer mr))
            {
                mr.sharedMaterial = Material;
            }

            item.transform.localScale = scale;
            item.SetActive(false);
            return item;
        }
        private void Bind(GameObject item, FPTransform transform, FPVector3 velocity)
        {
            item.transform.position = FPConvert.ToUnity(transform.Position);
            item.transform.rotation = FPConvert.ToUnity(transform.Rotation);

            ColliderComponentBase colcomp = item.GetComponent<ColliderComponentBase>();
            if (colcomp != null) colcomp.ShowGizmos = _showGizmos;

            //添加FPBody组件
            FPBody body = item.AddComponent<FPBody>();
            item.SetActive(true);

            //重新写入精确的定点数初始姿态
            //因为初始化时填入的position rotation有截断
            body.State.SetTransform(transform);

            _items.Add(new Item
            {
                Go = item,
                State = body.State,
                Velocity = velocity,
                ColComp = colcomp
            });
        }

        //计算全场所有运动物体定点数底层的哈希指纹
        private ulong ComputeHash()
        {
            unchecked
            {
                ulong hash = 14695981039346656037UL;
                for (int i = 0; i < _items.Count; i++)
                {
                    FPVector3 p = _items[i].State.Transform.Position;
                    hash = (hash ^ (ulong)p.x.Value) * 1099511628211UL;
                    hash = (hash ^ (ulong)p.y.Value) * 1099511628211UL;
                    hash = (hash ^ (ulong)p.z.Value) * 1099511628211UL;
                }
                return hash;
            }
        }
    }
}