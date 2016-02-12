using System;
using System.IO;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Windows.Forms;
using System.Configuration;
using System.Diagnostics;
using Digihail.AVE.Controls.GIS3D.Extend.CustomRenderPrimitives;
using Digihail.AVE.Media3D.Library;
using OpenTK;
using Digihail.AVE.Controls.GIS3D.Core.EntityComponent.Controller;
using Digihail.AVE.Controls.GIS3D.Core;
using Digihail.AVE.Controls.GIS3D.Extend.EntityComponent.Visual;
using Digihail.AVE.Media3D.EntityFramework;
using Digihail.AVE.Controls.GIS3D.Core.EntityComponent.Transform;
using Digihail.AVE.Media3D.EntityFramework.EntityComponent.Transform;  
using Digihail.AVE.Controls.GIS3D.Utils;
using Digihail.AVE.Media3D.EntityFramework.EntityComponent.Visual;
using Digihail.AVE.Media3D.EntityFramework.EntityComponent.Visual.BillboardStyles;
using System.Xml.Serialization;
namespace Digihail.AVE.Controls.GIS3D.Sample.RadarPower
{
    public partial class Form1 : Form
    {                
        private PackedBillboardMaterialStyle m_PackedMarkStyle = null;
        public Form1()
        {
            InitializeComponent();
        }

        #region Private Fields

        private delegate void SimulatorUpdatedEventHandler(double time, double elapsedTime, ref DateTime simDateTime);

        private DateTime m_StartDT = DateTime.Now;

        private GlobeSimulator m_Simulator = null;

        private Timer m_SimTimer = new Timer();

        private DetectRangeComponent m_DetectRange = null;

        private GlobeLayer m_RadarLayer = null;

        private GlobeLayer m_SatelliteLayer = null;

        #endregion
        
        private bool double_equals(double a, double b)
        {
            double ZERO = 1e-4;
            double delta = Math.Abs(a - b);
            return Math.Abs(a - b) < ZERO;
        }
        private double distance_sqr(Vector2d a, Vector2d b)
        {
            return Math.Pow(a.X - b.X, 2.0f) + Math.Pow(a.Y - b.Y, 2.0f);
        }  
        private double distance(Vector2d a, Vector2d b)
        {
            return Math.Sqrt(Math.Pow(a.X - b.X, 2.0f) + Math.Pow(a.Y - b.Y, 2.0f));
        }
        private int intersect_circle(Vector2d o1, double r1, Vector2d o2, double r2, ref Vector2d[] points)
        {
            double d, a, b, c, p, q, r;
            double[] cos_value = new double[2];
            double[] sin_value = new double[2];
            if (double_equals(o1.X, o2.X)&& double_equals(o1.Y, o2.Y)&& double_equals(r1,r2)) 
            {
                return -1;
            }

            d = distance(o1, o2);
            if (d > r1 + r2|| d < Math.Abs(r1 - r2)) 
            {
                return 0;
            }

            a = 2.0 * r1 * (o1.X - o2.X);
            b = 2.0 * r1 * (o1.Y - o2.Y);
            double dd = distance_sqr(o1, o2);
            c = r2 * r2 - r1 * r1 - Math.Pow(o1.X, 2) - Math.Pow(o1.Y, 2);
            p = a * a + b * b;
            q = -2.0 * a * c;
            if (double_equals(d, r1 + r2) || double_equals(d, Math.Abs(r1 - r2)))
            {
                cos_value[0] = -q / p / 2.0;
                sin_value[0] = Math.Sqrt(1 - cos_value[0] * cos_value[0]);

                points[0].X = (float)(r1 * cos_value[0] + o1.X);
                points[0].Y = (float)(r1 * sin_value[0] + o1.Y);
                                                
                if (!double_equals(distance_sqr(points[0], o2), r1 * r1)) 
                {
                    points[0].Y = (float)(o1.Y - r1 * sin_value[0]);
                }
                return 1;
            }

            r = c * c - b * b;
            double delta = Math.Sqrt(q * q - 4.0 * p * r);
            cos_value[0] = (delta - q) / p / 2.0;
            cos_value[1] = (-delta - q) / p / 2.0;
            sin_value[0] = Math.Sqrt(1 - cos_value[0] * cos_value[0]);
            sin_value[1] = -Math.Sqrt(1 - cos_value[1] * cos_value[1]);

            points[0].X = (float)(r1 * cos_value[0] + o1.X);
            points[1].X = (float)(r1 * cos_value[1] + o1.X);
            points[0].Y = (float)(r1 * sin_value[0] + o1.Y);
            points[1].Y = (float)(r1 * sin_value[1] + o1.Y);

            if (!double_equals(distance_sqr(points[0], o2),r2 * r2))
            {
                points[0].Y = (float)(o1.Y - r1 * sin_value[0]);
            }
            if (!double_equals(distance_sqr(points[1], o2), r2 * r2))
            {
                points[1].Y = (float)(o1.Y - r1 * sin_value[1]);
            }
            if (double_equals(points[0].Y, points[1].Y) && double_equals(points[0].X, points[1].X))
            {
                if (points[0].Y > 0) 
                {
                    points[1].Y = -points[1].Y;
                } else {
                    points[0].Y = -points[0].Y;
                }
            }
            return 2;
        }
        private void GetY0Point(double single_range,Vector3d f0,double af,double of,double sf,double os,ref Vector2d[] points)
        {
            double tan_half = Math.Sin(single_range) / Math.Cos(single_range);
            double r0 = Math.Abs(af * tan_half);                              //f0为圆心的半径
            double r1 = Math.Sqrt(Math.Pow(r0, 2.0f) + Math.Pow(of, 2.0f));   //f为圆心的半径
           
            Vector2d o1 = new Vector2d();                  
            Vector2d f0_temp = new Vector2d();
            f0_temp.X = f0.X;
            f0_temp.Y = f0.Y;

            int ret = intersect_circle(f0_temp, r0, o1, r1, ref points);
        }
        private List<Vector3d> RadarIntersect(Vector3d a, double theta, double range, uint precision)
        {    
            List<Vector3d> circle_left = new List<Vector3d>();
            List<Vector3d> circle_right = new List<Vector3d>(); 
            Vector2d[] points = new Vector2d[2];
            double rad = 3.1415926f / 180 * theta;
            Vector3d f0 = new Vector3d();
            double sint = Math.Sin(rad);
            double cost = Math.Cos(rad);
            double af = Math.Abs(1 / sint);
            double sf = Math.Abs(af * cost);
            double os = Math.Sqrt(Math.Pow(a.X, 2.0f) + Math.Pow(a.Y, 2.0f));
            double of = Math.Abs(sf - os);
            if (sf < os)
            {
                return circle_left;
            }
            f0.X = -(of * a.X / os);
            f0.Y = -(of * a.Y / os);
            f0.Z = (theta / Math.Abs(theta));

            for (int i = 0; i < precision / 2; i++)
            {                                  
                double single_range = 3.1415926f / 180 * range / precision * (i + 1);
                GetY0Point(single_range, f0, af, of, sf, os, ref points);
                Vector3d point1 = new Vector3d(points[0].X, points[0].Y, f0.Z);
                Vector3d point2 = new Vector3d(points[1].X, points[1].Y, f0.Z);
                circle_left.Add(point1);
                circle_right.Add(point2);
            }

            List<Vector3d> Y0Point = new List<Vector3d>();
            circle_left.Reverse(0, circle_left.Count);
            foreach (Vector3d p in circle_left)
            {
                Y0Point.Add(p);
            }
            Y0Point.Add(f0);
            foreach (Vector3d p in circle_right)
            {
                Y0Point.Add(p);
            }

            List<Vector3d> result = new List<Vector3d>();
            for (int i = 0; i < Y0Point.Count; i++)
            {                            
                Vector3d f = Y0Point[i];
                Vector3d p1 = new Vector3d();
                Vector3d p2 = new Vector3d();

                double A = Math.Pow(f.X - a.X, 2.0f) + Math.Pow(f.Y - a.Y, 2) + Math.Pow(f.Z - a.Z, 2);
                double B = a.X * f.X + a.Y * f.Y + a.Z * f.Z - Math.Pow(a.X, 2) - Math.Pow(a.Y, 2) - Math.Pow(a.Z, 2);
                B *= 2;
                double C = Math.Pow(a.X, 2) + Math.Pow(a.Y, 2) + Math.Pow(a.Z, 2) - 1;
                double delta = B * B - 4 * A * C;
                if (delta < 0)
                    continue;
                double u1 = -B + Math.Sqrt(B * B - 4 * A * C);
                u1 /= 2 * A;
                double u2 = -B - Math.Sqrt(B * B - 4 * A * C);
                u2 /= 2 * A;
                p1.X = (float)(a.X + u1 * f.X - u1 * a.X);
                p1.Y = (float)(a.Y + u1 * f.Y - u1 * a.Y);
                p1.Z = (float)(a.Z + u1 * f.Z - u1 * a.Z);
                p2.X = (float)(a.X + u2 * f.X - u2 * a.X);
                p2.Y = (float)(a.Y + u2 * f.Y - u2 * a.Y);
                p2.Z = (float)(a.Z + u2 * f.Z - u2 * a.Z);

                double d1 = Math.Sqrt(Math.Pow(p1.X - a.X, 2) + Math.Pow(p1.Y - a.Y, 2) + Math.Pow(p1.Z - a.Z, 2));
                double d2 = Math.Sqrt(Math.Pow(p2.X - a.X, 2) + Math.Pow(p2.Y - a.Y, 2) + Math.Pow(p2.Z - a.Z, 2));
                double d0 = Math.Sqrt(Math.Pow(a.X, 2) + Math.Pow(a.Y, 2) + Math.Pow(a.Z, 2));
                Vector3d p = d1 < d0 ? p1 : p2;
                result.Add(p);
            }
            return result;
        }          
        private void Form1_Load(object sender, EventArgs e)
        {
            try
            {
                // 加载ArcGIS地图
                string fileName = ConfigurationManager.AppSettings["3DDFile"];
                this.globe3DControl1.Load3DDFile(fileName);

                string contentRoot = ConfigurationManager.AppSettings["ContentRoot"];

                // 初始化三维渲染
                this.globe3DControl1.Initialize3D(0.001, 2.5, true, true, contentRoot);

                this.globe3DControl1.ShowSun = false;
                this.globe3DControl1.ShowMoon = false;

                // Fix me: 这里需要手动初始化一下DetectRange的硬件资源
                // 能否放在一个统一的扩展初始化函数中？
                DetectRange.InitGeometries(globe3DControl1.World.World.ContentManager);

                // 制作雷达范围的测试数据
                // 360个方向，5个高度层？
                // TODO: 先通过完整的圆球来测试，然后再测试有随机遮蔽的情况

                #region P-雷达范围

                // 改为通过添加实体的方式来测试？
                m_RadarLayer = globe3DControl1.World.AddLayer("radarLayer");
                Entity3D radarEntity = m_RadarLayer.AddEntity("radar01");
                // radarEntity.Visible = false;

                radarEntity.Color = Vector4.One;

                GeographicCoordinateTransform transformCmp = new GeographicCoordinateTransform();
                transformCmp.AlwaysFaceGeoCenter = true;
                transformCmp.LocalPose.Scale = new Vector3d(100, 100, 100);
                transformCmp.Longitude = 120;
                transformCmp.Latitude = 30;
                transformCmp.Height = 0;
                 
                radarEntity.AddComponent(transformCmp);
               
                DetectRangeComponent detectRange = new DetectRangeComponent();

                // DetectRange detectRange = new DetectRange();

                // 创建一个变换矩阵
                // 缩小后放在北极？
                Matrix4d matScale = Matrix4d.Scale(0.0001);
                Matrix4d matTran = Matrix4d.CreateTranslation(0, 1, 0);

                detectRange.UseSimpleNormal = true;
                // detectRange.Transform = matScale * matTran;
                detectRange.Color = new Vector4(0.8f, 1, 0.8f, 0.9f);
                detectRange.ScanColor = new Vector4(1.0f, 0.6f, 0.2f, 0.5f);

                // TODO: 创建随机的360度的范围的遮蔽数据
                // 通过Perlin噪声来模拟
                double[] occlusion = new double[360];
                PerlinNoise noise = new PerlinNoise(99);

                for (int i = 0; i < 360; i++)
                {
                    double val =
                        (noise.Noise(2 * i / 180.0, 2 * i / 180.0, -0.5)) * 0.7
                        + (noise.Noise(4 * i / 180.0, 4 * i / 180.0, 0)) * 0.2
                        + (noise.Noise(8 * i / 180.0, 8 * i / 180.0, 0.5)) * 0.1;

                    val = Math.Max(val * 3, 0);

                    occlusion[i] = val * 6000 + 800;
                }

                detectRange.Ranges.Add(CreateHorizRange(300, 1000, occlusion));
                detectRange.Ranges.Add(CreateHorizRange(600, 2500, occlusion));
                detectRange.Ranges.Add(CreateHorizRange(800, 3500, occlusion));
                detectRange.Ranges.Add(CreateHorizRange(1500, 4500, occlusion));
                detectRange.Ranges.Add(CreateHorizRange(2000, 4400, occlusion));
                detectRange.Ranges.Add(CreateHorizRange(2500, 4000, occlusion));
                detectRange.Ranges.Add(CreateHorizRange(2300, 800, occlusion));

                detectRange.RefreshGeometry();
                
                radarEntity.AddComponent(detectRange);

                m_DetectRange = detectRange;

                #endregion

                // 创建雷达范围自定义图元

                // 添加到场景中
                // globe3DControl1.World.World.RenderScene.CustomRenderPrimitives.Add(detectRange);

                #region 扫描范围锥体

                bool useSenceVolumePrim = false;

                if (useSenceVolumePrim)
                {
                    // 直接用自定义图元测试

                    RectSenseVolume senceVolume = new RectSenseVolume();
                    senceVolume.Initialize(globe3DControl1.World.ContentManager);

                    // 属性：变换矩阵
                    Matrix4d matRot = Matrix4d.CreateRotationX(Math.PI * 0.5);

                    senceVolume.Transform = matRot * Matrix4d.CreateTranslation(0, -5.6, 0);
                    senceVolume.HorizHalfAngle = 5.5;
                    senceVolume.VertiHalfAngle = 10.5;
                    senceVolume.Color = Vector4.One;

                    globe3DControl1.World.World.RenderScene.CustomRenderPrimitives.Add(senceVolume);

                }
                else
                {
                    // 用实体和组件测试
                    m_SatelliteLayer = globe3DControl1.World.AddLayer("satelliteLayer");
                    Entity3D satelliteEntity = m_SatelliteLayer.AddEntity("satellite01");

                    GeographicCoordinateTransform satTran = new GeographicCoordinateTransform();
                    satTran.AlwaysFaceGeoCenter = true;
                    satTran.Longitude = 120;
                    satTran.Latitude = 0;
                    satTran.Height = 35800e3;
                    satelliteEntity.AddComponent(satTran);

                    Vector3d satellitePos = Globe3DCoordHelper.GraphicToCentric(satTran.Longitude, satTran.Latitude, satTran.Height);

                    // 注意：卫星模型的视觉放大需要通过再加一个卫星模型子实体来实现，否则会影响传感器子实体
                    
                    #region 大视场传感器

                    
                    Entity3D sensorEntity = m_SatelliteLayer.AddEntity("sensor01");
                    sensorEntity.Parent = satelliteEntity;

                    SRTTransformComponent sensorTran = new SRTTransformComponent();
                    // sensorTran.RotationX = 10;
                    sensorEntity.AddComponent(sensorTran);

                    CustomController.SensorAutoScanController sensorCtrl = new CustomController.SensorAutoScanController();
                    sensorEntity.AddComponent(sensorCtrl);

                    RectSenseVolumeComponent sensorRange = new RectSenseVolumeComponent(globe3DControl1.World.ContentManager);
                    sensorRange.HorizHalfAngle = 5.5;
                    sensorRange.VertiHalfAngle = 10.5;
                    sensorRange.Pickable = false;
                    sensorRange.Color = new Vector4(1, 0.4f, 0, 0.2f);// Vector4.One;

                    sensorRange.ScanPlanes.Add(new RectSenseVolume.ScanPlane()
                    {
                        CurrAngle = 0,
                        Orientation = RectSenseVolume.ScanPlane.ScanOrientations.Vertical,
                        Visible = true,
                        Color = new Vector4(1, 0.2f, 0, 0.25f)
                    });

                    //sensorRange.ScanPlanes.Add(new RectSenseVolume.ScanPlane()
                    //{
                    //    CurrAngle = 0,
                    //    Orientation = RectSenseVolume.ScanPlane.ScanOrientations.Horizontal,
                    //    Visible = true,
                    //    Color = new Vector4(0, 1, 0, 0.4f)
                    //});

                    sensorRange.InvalidateScanPlanes();

                    sensorEntity.AddComponent(sensorRange);

                    #endregion

                    #region 小视场传感器

                    Entity3D smallSensorEntity = m_SatelliteLayer.AddEntity("sensor02");
                    smallSensorEntity.Parent = satelliteEntity;

                    CustomController.SensorGridScanController sensorGridCtrl = new CustomController.SensorGridScanController();
                    sensorGridCtrl.MinHorizAngle = 2;
                    sensorGridCtrl.MaxHorizAngle = 4;

                    sensorGridCtrl.MinVertiAngle = 2;
                    sensorGridCtrl.MaxVertiAngle = 4;

                    sensorGridCtrl.NumHorizGrids = 2;
                    sensorGridCtrl.NumVertiGrids = 2;

                    sensorGridCtrl.MoveInterval = 0.75;
                    sensorGridCtrl.StayInterval = 0.25;

                    sensorGridCtrl.ScanSequence.Add(new System.Drawing.Point(0, 0));
                    sensorGridCtrl.ScanSequence.Add(new System.Drawing.Point(0, 1));
                    sensorGridCtrl.ScanSequence.Add(new System.Drawing.Point(1, 1));
                    sensorGridCtrl.ScanSequence.Add(new System.Drawing.Point(1, 0));

                    smallSensorEntity.AddComponent(sensorGridCtrl);

                    SRTTransformComponent smallSensorTran = new SRTTransformComponent();
                    smallSensorEntity.AddComponent(smallSensorTran);

                    RectSenseVolumeComponent smallSensorRange = new RectSenseVolumeComponent(globe3DControl1.World.ContentManager);
                    smallSensorRange.HorizHalfAngle = 0.5;
                    smallSensorRange.VertiHalfAngle = 0.5;
                    smallSensorRange.Pickable = false;
                    smallSensorRange.Color = new Vector4(0, 1, 0, 0.4f);

                    smallSensorEntity.AddComponent(smallSensorRange);

                    #endregion

                    //////////////////////////////////////

                    uint precision = 30;
                    double theta = 6.5;
                    double range = 11;       
            
                    List<Vector3d> points = RadarIntersect((Vector3d)satellitePos, theta, range, precision);

                    m_PackedMarkStyle = new PackedBillboardMaterialStyle(globe3DControl1.World.ContentManager);
                    m_PackedMarkStyle.Texture = globe3DControl1.World.ContentManager.LoadTexture(@".\Resources\Textures\PackedIcons.png");

                    using (FileStream fs = new FileStream(@".\Resources\Textures\PackedIcons.xml", FileMode.Open, FileAccess.Read))
                    {
                        XmlSerializer xs = new XmlSerializer(typeof(List<PackedImage>));
                        m_PackedMarkStyle.PackedImages = xs.Deserialize(fs) as List<PackedImage>;

                        fs.Close();

                        // 如果加载失败怎么办
                        if (m_PackedMarkStyle.PackedImages == null)
                        {
                            throw new InvalidDataException("Failed loading packed image definition");
                        }
                    }
                    for (int i = 0; i < points.Count; i++)
                    {         
                        Vector3d p = points[i];
                        Vector3d pos = Globe3DCoordHelper.CentricToGraphic(p.X, p.Y,p.Z);

                        Entity3D markEntity = m_SatelliteLayer.AddEntity(i.ToString());

                        GeographicCoordinateTransform geoTf = new GeographicCoordinateTransform();
                        geoTf.Longitude = pos.X;
                        geoTf.Latitude = pos.Y;
                        geoTf.Height = pos.Z;

                        markEntity.AddComponent(geoTf);

                        BillboardComponent bgIcon = new BillboardComponent();
                        bgIcon.Pickable = true;
                        bgIcon.Color = Vector4.One;
                        bgIcon.Visible = true;
                        bgIcon.Width = 20;
                        bgIcon.Height = 20;
                        // Fix me: 本billboard的偏移量在哪里进行比较好？
                        bgIcon.Offset = new Vector2d(0, 0);
                        bgIcon.MaterialStyle = m_PackedMarkStyle;
                        // Fix me: 通过名称获得packed image index的操作在哪里进行比较好？
                        bgIcon.PackID = m_PackedMarkStyle.GetPackIndexByName("bg.fw.png");
                        // 所有的组成部分使用统一的组ID
                        bgIcon.GroupID = markEntity.ID;
                        // 背景最先绘制
                        bgIcon.RenderOrder = 0;
                        markEntity.AddComponent(bgIcon); 
                    }
                    
                }

                #endregion

                // 摄像机操作
                // 设置摄像机跟随预警机实体
                globe3DControl1.CameraController.ObserveMode = Digihail.AVE.Controls.GIS3D.Core.EntityComponent.Controller.GlobeCameraControllerComponent.CameraObserveMode.Free;

                // 修改摄像机的oriantationmode为surface
                globe3DControl1.CameraController.OrientationMode = GlobeCameraControllerComponent.CameraOrientationMode.NorthPole;

                globe3DControl1.CameraController.MinDistanceMeter = 200;// 30000;
                globe3DControl1.CameraController.CurViewDistance = 11400000;
                //this.globe3DControl1.World.GlobeCameraController.LowRange = 200000;
                this.globe3DControl1.World.GlobeCameraController.LowRange = 20000;
                this.globe3DControl1.World.GlobeCameraController.LowRangeLeanFactor = 0.2;
                //this.globe3DControl1.World.GlobeCameraController.LowRangeLeanFactor = 4;
                this.globe3DControl1.World.GlobeCameraController.AutoRotate = false;

                m_Simulator = new GlobeSimulator();

                m_Simulator.Updated += new GlobeSimulator.UpdateEventHandler(m_Simulator_Updated);
                m_Simulator.StartSimulation();

                m_SimTimer.Interval = 40;
                m_SimTimer.Tick += new EventHandler(m_SimTimer_Tick);
                m_SimTimer.Start();                   
              
                       
            }
            catch(Exception ex)
            {
                MessageBox.Show(ex.ToString());
                Debug.WriteLine(ex.ToString());
                throw;
            }
        }
        

        void m_SimTimer_Tick(object sender, EventArgs e)
        {
            m_Simulator.Update();
        }

        void m_Simulator_Updated(object sender, double curTime, double elapsedTime, ref DateTime curSimDateTime)
        {
            base.Invoke(new SimulatorUpdatedEventHandler(OnSimulatorUpdated), new object[] { curTime, elapsedTime, curSimDateTime });
        }

        private void OnSimulatorUpdated(double curTime, double elapsedTime, ref DateTime simDateTime)
        {
            if (m_DetectRange != null)
            {
                m_DetectRange.CurScanAngle++;
                if (m_DetectRange.CurScanAngle >= 360)
                {
                    m_DetectRange.CurScanAngle = 0;
                }
                globe3DControl1.Refresh3DView(curTime, elapsedTime);
            }
        }
        
        private static DetectRange.HorizRangeData CreateHorizRange(double height, double dist, double[] occulusion)
        {
            DetectRange.HorizRangeData range = new DetectRange.HorizRangeData();
            range.Height = height;
            for (int i = 0; i < range.Distance.Length; i++)
            {
                if (height < 1500)
                {
                    range.Distance[i] = Math.Min(dist, occulusion[i]);
                }
                else
                {
                    range.Distance[i] = dist;
                }
            }

            // TODO: 根据一个假定的遮蔽角数据计算实际距离？

            //range.Distance[85] = dist / 1.5;
            //range.Distance[90] = dist / 2;
            //range.Distance[95] = dist / 1.5;

            return range;
        }

        private void Form1_FormClosed(object sender, FormClosedEventArgs e)
        {
            DetectRange.DestroyGeometries(globe3DControl1.World.World.ContentManager.Device);
        }
    }
                  
}
