#define USING_CAM

using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading;
using OpenCvSharp;
using System.Drawing;
using System.Drawing.Imaging;
using System.Windows.Forms;
using System.IO;
using System.Runtime.InteropServices;
using System.Numerics;
using System.Diagnostics;
using static bendodatasrv.NDI;

namespace bendodatasrv
{
    public sealed class FrameCapture
    {
        [DllImport("kernel32")]
        private static extern long WritePrivateProfileString(string section, string key, string val, string filePath);

        [DllImport("kernel32")]
        private static extern int GetPrivateProfileString(string section, string key, string def, StringBuilder retVal, int size, string filePath);

        //private string m_confFilePath = @"C:\\Knu\\BendoDataServer\\conf.ini"; 
        private string m_confFilePath = AppDomain.CurrentDomain.BaseDirectory + "conf.ini";
        private string m_testFilePath = @"C:\\Knu\\ustest\\output";

        private static readonly Lazy<FrameCapture> lazy = new Lazy<FrameCapture>(() => new FrameCapture());

        public static FrameCapture Instance { get { return lazy.Value; } }

        public string deviceName;
        private int devId = 1;

        public int LoggerFlag = 0;
        public int LinkPortN = 1;
        public int LinkPortSpeed = 57600;

        private Mat m_cvFrm;
        private int dist = 0;
        private VideoCapture m_cvVideoStream;
        delegate void FrameCaptureDelegate();

        public const byte NONE = 0;
        public const byte CALIBRATION = 1;
        public const byte SCAN_US = 2;
        public const byte PORT_POSITION = 3;
        public const byte SCAN_ES = 4;

        public const byte ES_MODE_SCAN = 1;
        public const byte ES_MODE_SIM = 2;

        private byte m_bCaptureMode;

        private int m_iWorldHeight;
        private int m_iUSRoIStartX;
        private int m_iUSRoIStartY;
        private int m_iUSRoIWidth;
        private int m_iUSRoIHeight;
        private int m_iNoPixelPerFrame;
        private double m_dPixelScaleX;
        private double m_dPixelScaleY;
        private bool m_isCapture;

        private string mUSPath;
        private string mESPath;
        private UInt64 m_ImgSeq;

        private byte m_ESScanMode;

        private Logger objLogger;
        private USSocketServer objUSSocket;

        private Rectangle m_RoiRect;
        private Bitmap m_BMP;
        private Graphics m_gG;

        private Mat m_cvUSFrm;
        public Rect m_cvRoiRect;
        private bool m_isRoISet;

        private byte m_bFrameType;

        private float m_fPhyRofRG = 180.0f; //190.0          // Radius of R Guide = 190mm
        private float m_fPhyDofDR = 130.5f;                  // Distance(Y axis) of Depth Rail
        private float m_fPhyDofCH = 9.0f;//37.0    // Distance(z axis) of cable holder
        private float m_fPhyDofRG2JC = 92.0f;//89.0        // Distance(z axis) of R Guide to Ball joint center = 63mm
        private float m_fPhyDofJC2HW = 23.6f;        // Distance(z axis) of Ball joint center to Probe Holder's wall = 23.6mm
        private float m_fPhyDofJC2HC = 105.17f;//38.4f;        // Distance of Ball joint center to Probe Holder's center
        private float m_fPhyDofDO2JC = 111.0f;//60.5f;        // Distance(y axis) of Depth Origin to Ball joint center = 60.5mm
        private float m_fPhyDofJC2OA = 160.0f;       // Distance of Ball joint center to Virtual Origin Axis = 160mm

        private float m_fScaleFactor = 0.01f;                // 10mm per 1 unity unit

        private float m_fScaledRofRG;                 //Radius of R Guide = 190mm
        private float m_fScaledDofDR;                 //Distance(y axis) of Depth Rail
        private float m_fScaledDofCH;                 //Distance(z axis) of cable holder
        private float m_fScaledDofRG2JC;              //Distance(z axis) of R Guide to Ball joint center = 63mm
        private float m_fScaledDofJC2HW;              //Distance(z axis) of Ball joint center to Probe Holder's wall = 23.6mm
        private float m_fScaledDofJC2HC;              //Distance of Probe holder's wall to holder's center = 
        private float m_fScaledDofDO2JC;              //Distance(y axis) of Depth Origin to Ball joint center = 60.5mm
        private float m_fScaledDofJC2OA;              //Distance of Ball joint center to Virtual Origin Axis = 160mm

        private Vector3 m_distVecJC2HC;

        private bool m_bIsRotCalib;
        private bool m_bIsPosCalib;

        private float m_fYawOffset;

        private Stopwatch stopwatchAck;

        private System.Timers.Timer mTimer;

        private Quaternion m_qPHoffsetQuat;
        private float m_fLenOffset;

        // rotProbeHolder, posProbeHolder의 내용을 클라이언트로 전송함
        private Quaternion rotProbeHolder;
        private Vector3 posProbeHolder;

        private Matrix4x4 m_usFrameMat;

        private StringBuilder m_mat2str;

        private FileInfo[] m_esSimImages;
        private int m_esSimImgIdx;

        private byte m_noDummyRead = 3;

        private bool isTestMode = false;

        private Thread m_pThdMaker;
        private static bool m_bMakerLooping = false;
        private static bool MakerPause = true;

#if OTS
        private string tspPath = AppDomain.CurrentDomain.BaseDirectory + "Tsp.txt";
        private string[] tspLine;
        private Mat t_SensorToProbe;
#endif

        private FrameCapture()
        {
            mUSPath = "";
            mESPath = "";
            m_ImgSeq = 0;

#if OTS
            tspLine = File.ReadAllLines(tspPath);
            t_SensorToProbe = new Mat(4, 4, MatType.CV_64FC1);
            for (int i = 0 ; i < 4 ; i++)
            {
                var row = tspLine[i + 2].Split('\t');
                for (int j = 0 ; j < 4 ; j++)
                {
                    t_SensorToProbe.Set(i, j, double.Parse(row[j]));
                }
            }
#endif

            m_esSimImgIdx = 0;

            objLogger = Logger.Instance;
            //objUSSocket = USSocketServer.Instance;
            mTimer = new System.Timers.Timer();
            
            StringBuilder tmp = new StringBuilder(10);
            //Get sampling period from configuration file
            GetPrivateProfileString("Timer", "Interval", "33", tmp, 10, m_confFilePath);
            mTimer.Interval = int.Parse(tmp.ToString());

            //Get ES scan mode from configuration file
            GetPrivateProfileString("ES Scan", "Mode", "1", tmp, 10, m_confFilePath);
            m_ESScanMode = byte.Parse(tmp.ToString());
            m_ESScanMode = 3;

            //Get Scale factor from configuration file
            GetPrivateProfileString("Scaling", "Factor", "0.01", tmp, 10, m_confFilePath);
            m_fScaleFactor = float.Parse(tmp.ToString());

            if (m_ESScanMode == ES_MODE_SIM) {
                GetPrivateProfileString("ES SCAN", "Sim Data Folder", "", tmp, 1024, m_confFilePath);
                DirectoryInfo d = new DirectoryInfo(tmp.ToString());
                m_esSimImages = d.GetFiles("*.jpg");
            }

            GetPrivateProfileString("Mode", "Test", "0", tmp, 1024, m_confFilePath);
            if (tmp.ToString() == "1")
            {
                isTestMode = true;
            }
            else
            {
                isTestMode = false;
            }

            mTimer.Elapsed += TimerFired;
            m_isCapture = false;
            m_isRoISet = false;
            stopwatchAck = new Stopwatch(); //객체 선언;

            m_cvVideoStream = new VideoCapture();
            m_cvVideoStream.Open(0);

            m_fScaledRofRG = m_fPhyRofRG * m_fScaleFactor;
            m_fScaledDofDR = m_fPhyDofDR * m_fScaleFactor;
            m_fScaledDofCH = m_fPhyDofCH * m_fScaleFactor;
            m_fScaledDofRG2JC = m_fPhyDofRG2JC * m_fScaleFactor;
            //m_fScaledDofJC2HW = m_fPhyDofJC2HW * m_fScaleFactor;
            m_fScaledDofJC2HC = m_fPhyDofJC2HC * m_fScaleFactor;
            m_fScaledDofDO2JC = m_fPhyDofDO2JC * m_fScaleFactor;
            m_fScaledDofJC2OA = m_fPhyDofJC2OA * m_fScaleFactor;

            m_bIsRotCalib = false;
            m_bIsPosCalib = false;
            m_fYawOffset = 0.0f;

            m_qPHoffsetQuat = Quaternion.Identity;

            //m_distVecJC2HC = (Vector3.UnitZ * (m_fScaledDofJC2HC * -1.0f));
            m_distVecJC2HC = (Vector3.UnitZ * m_fScaledDofJC2HC);

            SetCaptureMode(NONE);

            /*
            m_usFrameMat = new Matrix4x4(11.1f, 12.1f, 13.1f, 14.1f, 21.2f, 22.2f, 23.2f, 24.2f, 31.3f, 32.3f, 33.3f, 34.3f, 41.4f, 42.4f, 43.4f, 44.4f);
            m_usFrameMat.M14 = 11.1f;
            m_usFrameMat.M24 = 22.2f;
            m_usFrameMat.M34 = 33.3f;
            m_usFrameMat.M44 = 1.0f;*/
            m_mat2str = new StringBuilder();
            
        }

        /* Functions to write and read with the INI file
        public static void WriteConfig(string file, string section, string key, string val)
        {
            WritePrivateProfileString(section, key, val, GetFile(file));
        }
        public static string ReadConfig(string file, string section, string key)
        {
            // C#에서는 포인터를 명시적으로 표현할 수 없기 때문 StringBuilder로 가져옵니다.
            StringBuilder temp = new StringBuilder(255);
            int ret = GetPrivateProfileString(section, key, null, temp, 255, GetFile(file));
            return temp.ToString();
        }
        */

        public void FCinit(string FilePath)
        {
            mTimer = new System.Timers.Timer();
            mTimer.Interval = ReadConfigSet(FilePath);

            m_isCapture = false;

            stopwatchAck = new Stopwatch();

            m_mat2str = new StringBuilder();

        }

        private double ReadConfigSet(string path)
        {
            int res = 0;
            double intv = 50;
            m_confFilePath = path + "confs.ini";

            StringBuilder tmp = new StringBuilder(10);

            res = GetPrivateProfileString("Logger", "Record", "0", tmp, 10, m_confFilePath);
            LoggerFlag = int.Parse(tmp.ToString());
            if (LoggerFlag == 1)
                objLogger = Logger.Instance;

            //Get sampling period from configuration file
            GetPrivateProfileString("Timer", "Interval", "33", tmp, 10, m_confFilePath);//Confs.ini에서 설정읽기
            intv = 50;


            GetPrivateProfileString("Scaling", "Factor", "0.01", tmp, 10, m_confFilePath);
            m_fScaleFactor = float.Parse(tmp.ToString());

            GetPrivateProfileString("Mode", "Test", "0", tmp, 1024, m_confFilePath);
            if (tmp.ToString() == "1")
            {
                isTestMode = true;
            }
            else
            {
                isTestMode = false;
            }

            GetPrivateProfileString("GSLink", "COM", "0", tmp, 10, m_confFilePath);
            LinkPortN = int.Parse(tmp.ToString());
            GetPrivateProfileString("GSLink", "Speed", "0", tmp, 10, m_confFilePath);
            LinkPortSpeed = int.Parse(tmp.ToString());
            Console.WriteLine("COM{0}, Speed={1}", LinkPortN, LinkPortSpeed);

            return intv;

        }

        public void SetCaptureMode(byte type)
        {
            lock (this)
            {
                m_bCaptureMode = type;
            }
        }

        public byte GetCaptureMode()
        {
            lock (this)
            {
                return m_bCaptureMode;
            }
        }

        public void SetUSStorePath(string path)
        {
            try
            {
                if (Directory.Exists(path))
                {
                    Console.WriteLine("That path exists already.");
                    mUSPath = path;
                    return;
                }

                // Try to create the directory.
                DirectoryInfo di = Directory.CreateDirectory(path);
                mUSPath = path;
                Console.WriteLine("The directory was created successfully at {0}.", Directory.GetCreationTime(path));
            }
            catch (NotSupportedException e)
            {
                Console.WriteLine("\n\n지원하지 않는 예외: {0}\n\n", e.ToString());
                return;
            }
            catch (DriveNotFoundException e)
            {
                Console.WriteLine("\n\n드라이브를 찾을 수 없음: {0}\n\n", e.ToString());
                return;
            }
            finally { }

        }

        public void StartMaker()
        {
            objUSSocket = USSocketServer.Instance;

            lock (this)
            {
                m_bMakerLooping = true;
            }

            MakerPause = true;

            m_pThdMaker = new Thread(MakerThreadLoop);
            stopwatchAck.Start();
            m_pThdMaker.Start();

        }

        public void StopMaker()
        {
            SetCaptureMode(NONE);

            stopwatchAck.Stop();
            lock (this)
            {
                m_bMakerLooping = false;
            }

            if (MakerPause == false)
            {
                Thread.Sleep(200);
            }
            MakerPause = true;
            m_ImgSeq = 0;
        }

        private bool IsMakerLooping()
        {
            lock (this)
            {
                return m_bMakerLooping;
            }
        }

        public void MakerThreadLoop()
        {
            while (IsMakerLooping())
            {
                if ((stopwatchAck.ElapsedMilliseconds) >= mTimer.Interval)
                {
                    stopwatchAck.Restart();

                    switch (GetCaptureMode())
                    {

                    }
                    MakerPause = false;
                }
            }
        }

        public void SetESStorePath(string path)
        {
            System.IO.Directory.CreateDirectory(path);
            mESPath = path;
        }

        public void StartCatpure()
        {
            m_isCapture = true;

            objUSSocket = USSocketServer.Instance;

            if (GetCaptureMode() != SCAN_ES)
            {
#if !OTS
                // IMU, 위치센서 사용 시 활성화
                objIMU.StartThread();
                objPos.StartThread(); 
#endif
            }
            mTimer.Start();
        }

        public void StopCatpure()
        {
            //m_cvVideoStream.Release();
            m_isCapture = false;
            SetCaptureMode(NONE);
            mTimer.Stop();

            m_ImgSeq = 0;
        }


        private void TimerFired(object sender, System.Timers.ElapsedEventArgs e)
        {
            switch (GetCaptureMode()){
                case CALIBRATION:
                    GetCalibration();
                    break;
                case SCAN_US:
                    GetUSFrame();
                    break;
                case PORT_POSITION:
                    GetPortPosition();
                    break;
                case SCAN_ES:
                    GetESFrame();
                    break;
            }
        }

        public void GetCalibration()
        {
            if (isTestMode)
            {
                string fName = "";

                m_ImgSeq++;
                fName = String.Format("{0,4:D4}", m_ImgSeq);
                var dataFile = m_testFilePath + "\\" + fName + ".txt";

                if (!File.Exists(dataFile))
                {
                    m_ImgSeq = 1;
                    fName = String.Format("{0,4:D4}", m_ImgSeq);
                    dataFile = m_testFilePath + "\\" + fName + ".txt";
                }

                try
                {
                    var allData = File.ReadAllText(m_testFilePath + "\\" + fName + ".txt");
                    var data = allData.Split('\t');

                    rotProbeHolder = new Quaternion(float.Parse(data[1]), float.Parse(data[2]), float.Parse(data[3]), float.Parse(data[0]));
                    posProbeHolder = new Vector3(float.Parse(data[4]) * m_fScaleFactor, float.Parse(data[5]) * m_fScaleFactor, float.Parse(data[6]) * m_fScaleFactor);
                }
                catch (Exception)
                {
                    rotProbeHolder = new Quaternion(0, 0, 0, 0);
                    posProbeHolder = new Vector3(0, 0, 0);
                }
            }
            else
            {
            }

#if OTS
            try
            {
                UpdateTransforms();
                var matrix1 = GetMatrixTransform(0);
                var matrix2 = GetMatrixTransform(1);

                var t_WorldToSensor = ConvertTMarix(matrix1);

                Mat t_WorldToProbe = t_WorldToSensor * t_SensorToProbe;

                rotProbeHolder = MatrixToQuaternion(t_WorldToProbe);
                posProbeHolder = new Vector3((float)t_WorldToProbe.Get<double>(0, 3) * m_fScaleFactor, (float)t_WorldToProbe.Get<double>(1, 3) * m_fScaleFactor, (float)t_WorldToProbe.Get<double>(2, 3) * m_fScaleFactor);
            }
            catch (Exception)
            {
            }
#endif

            if (!posProbeHolder.X.Equals(float.NaN) && !posProbeHolder.Y.Equals(float.NaN) && !posProbeHolder.Z.Equals(float.NaN))
            {
                objUSSocket.ResponsePosCalib(rotProbeHolder, posProbeHolder);
            }
        }

        public void GetPortPosition()
        {
            objUSSocket.ResponsePortPosition(rotProbeHolder, posProbeHolder);
        }

        public void GetUSFrame()
        {
            String fName = "";
            if (m_cvVideoStream.IsOpened() && m_isCapture && !isTestMode)
            {
                //stopwatch.Start();
                m_ImgSeq++;
                fName = String.Format("{0,4:D4}", m_ImgSeq);
                m_cvVideoStream.Read(m_cvFrm);
                

                m_cvUSFrm = new Mat(m_cvFrm, m_cvRoiRect);// WARNING!!! ORIGIN SIZE & ROI SIZE  
                m_cvUSFrm.SaveImage(mUSPath + "\\" + fName+".jpg");
                
                m_usFrameMat = Matrix4x4.CreateFromQuaternion(rotProbeHolder);
                m_usFrameMat.M14 = posProbeHolder.X;
                m_usFrameMat.M24 = posProbeHolder.Y;
                m_usFrameMat.M34 = posProbeHolder.Z;
                m_usFrameMat.M44 = 1.0f;

#if OTS
                // OTS 사용 시 rotProbeHolder, posProbeHolder를 OTS 데이터로 대체
                try
                {
                    UpdateTransforms();
                    var matrix1 = GetMatrixTransform(0);
                    var matrix2 = GetMatrixTransform(1);

                    var t_WorldToSensor = ConvertTMarix(matrix1);

                    Mat t_WorldToProbe = t_WorldToSensor * t_SensorToProbe;

                    rotProbeHolder = MatrixToQuaternion(t_WorldToProbe);
                    posProbeHolder = new Vector3((float)t_WorldToProbe.Get<double>(0, 3) * m_fScaleFactor, (float)t_WorldToProbe.Get<double>(1, 3) * m_fScaleFactor, (float)t_WorldToProbe.Get<double>(2, 3) * m_fScaleFactor);

                    // matrix로 저장
                    //m_mat2str.Clear();
                    //m_mat2str.Append(t_WorldToProbe.Get<double>(0, 0) + " " + t_WorldToProbe.Get<double>(0, 1) + " " + t_WorldToProbe.Get<double>(0, 2) + " " + t_WorldToProbe.Get<double>(0, 3) + ", ");
                    //m_mat2str.Append(t_WorldToProbe.Get<double>(1, 0) + " " + t_WorldToProbe.Get<double>(1, 1) + " " + t_WorldToProbe.Get<double>(1, 2) + " " + t_WorldToProbe.Get<double>(1, 3) + ", ");
                    //m_mat2str.Append(t_WorldToProbe.Get<double>(2, 0) + " " + t_WorldToProbe.Get<double>(2, 1) + " " + t_WorldToProbe.Get<double>(2, 2) + " " + t_WorldToProbe.Get<double>(2, 3) + ", ");
                    //m_mat2str.Append(t_WorldToProbe.Get<double>(3, 0) + " " + t_WorldToProbe.Get<double>(3, 1) + " " + t_WorldToProbe.Get<double>(3, 2) + " " + t_WorldToProbe.Get<double>(3, 3));


                    // quaternion으로 저장
                    m_mat2str.Clear();
                    m_mat2str.Append(rotProbeHolder.W + "\t" + rotProbeHolder.X + "\t" + rotProbeHolder.Y + "\t" + rotProbeHolder.Z + "\t");
                    m_mat2str.Append(posProbeHolder.X + "\t" + posProbeHolder.Y + "\t" + posProbeHolder.Z);

                    File.WriteAllText(mUSPath + "\\" + fName + ".txt", m_mat2str.ToString());
                }
                catch (Exception)
                {

                }
#else
                m_mat2str.Clear();
                m_mat2str.Append(m_usFrameMat.M11 + " " + m_usFrameMat.M12 + " " + m_usFrameMat.M13 + " " + m_usFrameMat.M14 + ", ");
                m_mat2str.Append(m_usFrameMat.M21 + " " + m_usFrameMat.M22 + " " + m_usFrameMat.M23 + " " + m_usFrameMat.M24 + ", ");
                m_mat2str.Append(m_usFrameMat.M31 + " " + m_usFrameMat.M32 + " " + m_usFrameMat.M33 + " " + m_usFrameMat.M34 + ", ");
                m_mat2str.Append(m_usFrameMat.M41 + " " + m_usFrameMat.M42 + " " + m_usFrameMat.M43 + " " + m_usFrameMat.M44);
                File.WriteAllText(mUSPath + "\\" + fName + ".txt", m_mat2str.ToString());
#endif
                // ResponseUSScan함수를 이용해 초음파 이미지 파일명과 rotation, position 정보를 전달
                objUSSocket.ResponseUSScan(fName+".jpg", rotProbeHolder, posProbeHolder);

                //stopwatch.Stop();
                //Console.WriteLine("time : " + stopwatch.ElapsedMilliseconds + "ms");
                //stopwatch.Reset();
                //Console.WriteLine(m_ImgSeq + " img saved.");
            }
            // 테스트 모드는 미리 저장해놓은 데이터를 이용해 서버를 사용하는 모드(m_testFilePath 폴더 내의 데이터를 반복 전송함)
            // conf.ini 파일에서 [Mode] 아래의 Test를 1로 설정하면 테스트 모드, 0으로 설정하면 일반 모드로 동작
            else if (isTestMode)
            {
                m_ImgSeq++;
                fName = String.Format("{0,4:D4}", m_ImgSeq);
                var dataFile = m_testFilePath + "\\" + fName + ".txt";
                var imageFile = m_testFilePath + "\\" + fName + ".jpg";

                if (!File.Exists(imageFile))
                {
                    m_ImgSeq = 1;
                    fName = String.Format("{0,4:D4}", m_ImgSeq);
                    dataFile = m_testFilePath + "\\" + fName + ".txt";
                    imageFile = m_testFilePath + "\\" + fName + ".jpg";
                }

                try
                {
                    var allData = File.ReadAllText(m_testFilePath + "\\" + fName + ".txt");
                    var data = allData.Split('\t');

                    rotProbeHolder = new Quaternion(float.Parse(data[1]), float.Parse(data[2]), float.Parse(data[3]), float.Parse(data[0]));
                    posProbeHolder = new Vector3(float.Parse(data[4]) * m_fScaleFactor, float.Parse(data[5]) * m_fScaleFactor, float.Parse(data[6]) * m_fScaleFactor);

                    File.Copy(imageFile, mUSPath + "\\" + fName + ".jpg", true);
                    File.Copy(dataFile, mUSPath + "\\" + fName + ".txt", true);
                }
                catch (Exception)
                {
                    rotProbeHolder = new Quaternion(0, 0, 0, 0);
                    posProbeHolder = new Vector3(0, 0, 0);
                }

                objUSSocket.ResponseUSScan(fName + ".jpg", rotProbeHolder, posProbeHolder);
            }
        }

        public void GetUSFrameForRoi()
        {
            objUSSocket = USSocketServer.Instance;
            if (m_cvVideoStream.IsOpened())
            {
                string fileName = "sample";
                m_cvFrm = new Mat();
                m_cvVideoStream.Read(m_cvFrm);
                m_cvFrm.SaveImage(mUSPath + "\\" + fileName + ".jpg");

                objUSSocket.ResponseUSScanForRoi(fileName + ".jpg");
            }
            else
            {
                objUSSocket.ResponseUSScanForRoi("");
            }
        }

        public void GetESFrame()
        {
            String fName = "";
            m_ImgSeq++;
            fName = String.Format("{0,4:D4}.jpg", m_ImgSeq);

            if (m_ESScanMode == ES_MODE_SCAN)
            {
                if (m_cvVideoStream.IsOpened() && m_isCapture)
                {
                    m_cvVideoStream.Read(m_cvFrm);
                    m_cvUSFrm = new Mat(m_cvFrm, m_cvRoiRect);// WARNING!!! ORIGIN SIZE & ROI SIZE  
                    m_cvUSFrm.SaveImage(mESPath + "\\" + fName);
                }
            }
            else
            {
                Mat tmp = new Mat(m_esSimImages[m_esSimImgIdx].FullName, ImreadModes.Color);
                tmp.SaveImage(mESPath + "\\" + fName);
                m_esSimImgIdx++;
            }
        }

        // 위치 센서, IMU센서 사용 시 아래 부분 내용 사용됨
 

        private bool AssetValues(Quaternion rg, Quaternion ph, float depth, float length) {
            bool retVal = false;
            if (!(rg.W.Equals(float.NaN) && rg.X.Equals(float.NaN) && rg.Y.Equals(float.NaN) && rg.Z.Equals(float.NaN) && depth.Equals(float.NaN) && length.Equals(float.NaN)))
            {
                // All values are normal
                retVal = true;
            }
            else
            {
                // Some values are Not a Number
                retVal = false;
            }
            return retVal;
        }

        public void DoScreenCapture(string filename)
        {
            // Determine the size of the "virtual screen", including all monitors.
            int screenLeft = SystemInformation.VirtualScreen.Left;
            int screenTop = SystemInformation.VirtualScreen.Top;
            int screenWidth = SystemInformation.VirtualScreen.Width;
            int screenHeight = SystemInformation.VirtualScreen.Height;

            // Create a bitmap of the appropriate size to receive the screenshot.
            using (Bitmap bmp = new Bitmap(screenWidth, screenHeight))
            {
                // Draw the screenshot into our bitmap.
                using (Graphics g = Graphics.FromImage(bmp))
                {
                    g.CopyFromScreen(screenLeft, screenTop, 0, 0, bmp.Size);
                }

                // Stuff the bitmap into a file
                bmp.Save(filename, System.Drawing.Imaging.ImageFormat.Jpeg);
            }
        }

        public void SetUSRoI(int x, int y, int w, int h)
        {
            Mat aFrm = new Mat();
            m_cvVideoStream.Read(aFrm);
            //Cv2.ImShow("test", aFrm);

            m_iUSRoIStartX = x;
            m_iUSRoIStartY = y;
            m_iUSRoIWidth = w;
            m_iUSRoIHeight = h;

            m_RoiRect = new Rectangle(m_iUSRoIStartX, m_iUSRoIStartY, m_iUSRoIWidth, m_iUSRoIHeight);
            m_BMP = new Bitmap(m_RoiRect.Width, m_RoiRect.Height, System.Drawing.Imaging.PixelFormat.Format24bppRgb);
            m_gG = Graphics.FromImage(m_BMP);

            //m_cvFrm = Mat.Zeros(480, 640, MatType.CV_16UC3);
            m_cvFrm = new Mat();
            m_cvUSFrm = Mat.Zeros(m_iUSRoIHeight, m_iUSRoIWidth, MatType.CV_8UC3);
            m_cvRoiRect = new Rect(m_iUSRoIStartX, m_iUSRoIStartY, m_iUSRoIWidth, m_iUSRoIHeight);

            m_isRoISet = true;
        }

        public Vector3 ToEulerAngles(Quaternion q)
        {
            // Store the Euler angles in radians
            Vector3 pitchYawRoll = new Vector3();

            double sqw = q.W * q.W;
            double sqx = q.X * q.X;
            double sqy = q.Y * q.Y;
            double sqz = q.Z * q.Z;
            //Console.WriteLine("q.w={0}|sqw={1}, q.x={2}|sqx={3}, q.y={4}|sqy={5}, q.z={6}|sqz={7}", q.W, sqw, q.Y, sqx, q.Y, sqy, q.Z, sqz);

            // If quaternion is normalised the unit is one, otherwise it is the correction factor
            double unit = sqx + sqy + sqz + sqw;
            double test = q.X * q.Y + q.Z * q.W;

            if (test > 0.4999f * unit)                              // 0.4999f OR 0.5f - EPSILON
            {
                // Singularity at north pole
                pitchYawRoll.Y = 2f * (float)Math.Atan2(q.X, q.W);  // Yaw
                pitchYawRoll.X = (float)Math.PI * 0.5f;             // Pitch
                pitchYawRoll.Z = 0f;                                // Roll
                return pitchYawRoll;
            }
            else if (test < -0.4999f * unit)                        // -0.4999f OR -0.5f + EPSILON
            {
                // Singularity at south pole
                pitchYawRoll.Y = -2f * (float)Math.Atan2(q.X, q.W); // Yaw
                pitchYawRoll.X = (float)(-Math.PI) * 0.5f;          // Pitch
                pitchYawRoll.Z = 0f;                                // Roll
                return pitchYawRoll;
            }
            else
            {
                pitchYawRoll.Y = (float)Math.Atan2(2f * q.Y * q.W - 2f * q.X * q.Z, sqx - sqy - sqz + sqw);       // Yaw
                pitchYawRoll.X = (float)Math.Asin(2f * test / unit);                                             // Pitch
                pitchYawRoll.Z = (float)Math.Atan2(2f * q.X * q.W - 2f * q.Y * q.Z, -sqx + sqy - sqz + sqw);      // Roll
            }

            // convert radians to degrees
            
            /*
            pitchYawRoll.Y *= (180.0f / (float)Math.PI);
            pitchYawRoll.X *= (180.0f / (float)Math.PI);
            pitchYawRoll.Z *= (180.0f / (float)Math.PI);
            */

            return pitchYawRoll;
        }

        public Quaternion MatrixToQuaternion(Mat matrix)
        {
            var result = new Quaternion();

            var w = Math.Sqrt(1 + matrix.Get<double>(0, 0) + matrix.Get<double>(1, 1) + matrix.Get<double>(2, 2)) / 2;
            result.W = (float)w;
            result.X = (float)((matrix.Get<double>(2, 1) - matrix.Get<double>(1, 2)) / (4 * w));
            result.Y = (float)((matrix.Get<double>(0, 2) - matrix.Get<double>(2, 0)) / (4 * w));
            result.Z = (float)((matrix.Get<double>(1, 0) - matrix.Get<double>(0, 1)) / (4 * w));

            return result;
        }

        public Vector3 GetTranslation(Mat matrix)
        {
            var result = new Vector3();

            result.X = (float)matrix.Get<double>(0, 3);
            result.Y = (float)matrix.Get<double>(1, 3);
            result.Z = (float)matrix.Get<double>(2, 3);

            return result;
        }

        private Mat ConvertTMarix(MatrixTransformStruct matrix)
        {
            var result = new Mat(4, 4, MatType.CV_64FC1);

            result.Set(0, 0, matrix.m11);
            result.Set(0, 1, matrix.m12);
            result.Set(0, 2, matrix.m13);
            result.Set(0, 3, matrix.m14);

            result.Set(1, 0, matrix.m21);
            result.Set(1, 1, matrix.m22);
            result.Set(1, 2, matrix.m23);
            result.Set(1, 3, matrix.m24);

            result.Set(2, 0, matrix.m31);
            result.Set(2, 1, matrix.m32);
            result.Set(2, 2, matrix.m33);
            result.Set(2, 3, matrix.m34);

            result.Set(3, 0, matrix.m41);
            result.Set(3, 1, matrix.m42);
            result.Set(3, 2, matrix.m43);
            result.Set(3, 3, matrix.m44);

            return result;
        }

        public Mat QuaternionToMatrix(Quaternion q)
        {
            var result = new Mat(4, 4, MatType.CV_64FC1);

            var w = q.W;
            var x = q.X;
            var y = q.Y;
            var z = q.Z;
            var w2 = Math.Pow(q.W, 2);
            var x2 = Math.Pow(q.X, 2);
            var y2 = Math.Pow(q.Y, 2);
            var z2 = Math.Pow(q.Z, 2);

            result.Set<double>(0, 0, w2 + x2 - y2 - z2);
            result.Set<double>(0, 1, 2 * (x * y - w * z));
            result.Set<double>(0, 2, 2 * (x * z + w*y));
            result.Set<double>(0, 3, 0.0);

            result.Set<double>(1, 0, 2 * (x * y + w * z));
            result.Set<double>(1, 1, w2 - x2 + y2 - z2);
            result.Set<double>(1, 2, 2 * (y * z - w * x));
            result.Set<double>(1, 3, 0.0);

            result.Set<double>(2, 0, 2 * (x * z - w * y));
            result.Set<double>(2, 1, 2 * (y * z + w * x));
            result.Set<double>(2, 2, w2 - x2 - y2 + z2);
            result.Set<double>(2, 3, 0.0);

            result.Set<double>(3, 0, 0.0);
            result.Set<double>(3, 1, 0.0);
            result.Set<double>(3, 2, 0.0);
            result.Set<double>(3, 3, 1.0);


            return result;
        }
    }
}