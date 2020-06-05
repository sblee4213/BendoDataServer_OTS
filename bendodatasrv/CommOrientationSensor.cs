#define USING_EULERANGLE
#define YKPARK

using System;
using System.Collections;
using System.Text;
using System.IO.Ports;
using System.Threading;
using System.Numerics;
using System.IO;

namespace bendodatasrv
{
    public struct SenVal
    {
#if USING_EULERANGLE           
        public Quaternion q;
        public float roll;
        public float pitch;
        public float yaw;
        public string batt;

        public SenVal(Quaternion tmp, float r, float p, float y, string bat)
        {
            this.q = tmp;
            this.roll = r;
            this.pitch = p;
            this.yaw = y;
            this.batt = bat;
        }
#else
        public Quaternion q;
        public Matrix4x4 mat;
        public string batt;

        public SenVal(Quaternion paramQ, Matrix4x4 paramMat, string bat)
        {
            this.q = paramQ;
            this.mat = paramMat;
            this.batt = bat;
        }
#endif
    };


    public sealed class CommOrientationSensor
    {
        private static readonly Lazy<CommOrientationSensor> lazy =
            new Lazy<CommOrientationSensor>(() => new CommOrientationSensor());

        public static CommOrientationSensor Instance { get { return lazy.Value; } }

        private string m_IMUPortName = "COM3";
        private int m_IMUBaudrate = 9600;

        private SerialPort m_IMUComm;
        private Thread m_pThdIMU;
        private Queue m_qIMUOut; // From Unity to IMU Receiver
        private Queue m_qIMUPosIn; // From IMU Receiver to Unity
        private Queue m_qIMURotIn; // From IMU Receiver to Unity
        private static bool m_bIMULooping = false;
        private static bool m_bIMUCommIsOpen = false;
        private static bool m_bIMUThdRunning = false;
        private static bool m_bEnableQueue = false;
        private int m_iImuID;
        private bool m_bCalcProbeOffset = false;
        private bool m_bIsProbeOffsetSet = false;
        private SenVal m_qt1stRGuide = new SenVal(Quaternion.Identity, 0,0,0,"0");
        private SenVal m_qt1stProbeHolder = new SenVal(Quaternion.Identity, 0, 0, 0, "0");
        private SenVal m_qt2ndRGuide = new SenVal(Quaternion.Identity, 0, 0, 0, "0");
        private SenVal m_qt2ndProbeHolder = new SenVal(Quaternion.Identity, 0, 0, 0, "0");
        private Logger objLogger;

        private CommOrientationSensor()
        {
            objLogger = Logger.Instance;
        }

        public int SetPort(string pName, int baud)
        {
            int retVal = 0;
            m_IMUPortName = pName;
            m_IMUBaudrate = baud;

            retVal = PortOpen();

            return retVal;

            try
            {
                m_IMUComm = new SerialPort(m_IMUPortName, m_IMUBaudrate, Parity.None, 8, StopBits.One);
                m_IMUComm.ReadTimeout = 200;
                m_IMUComm.Open();

                Console.WriteLine("Configured serialport for IMU sensor.(" + pName + ", " + baud + ")");
                objLogger.LogWrite("IMU sensor configured(" + pName + ", " + baud + ")");
                return 1;
            }
            catch (Exception ex)
            {
                Console.WriteLine("Configuration failed(IMU sensor)(" + pName + ", " + baud + "): error="+ex.Message);
                objLogger.LogWrite("IMU connection failed.(" + pName + ", " + baud + ")");
                return 0;
            }
        }

        private int PortOpen()
        {
            try
            {
                m_IMUComm = new SerialPort(m_IMUPortName, m_IMUBaudrate, Parity.None, 8, StopBits.One);
                m_IMUComm.ReadTimeout = 200;
                m_IMUComm.Open();

                Console.WriteLine("Configured serialport for IMU sensor.(" + m_IMUPortName + ", " + m_IMUBaudrate + ")");
                objLogger.LogWrite("IMU sensor configured(" + m_IMUPortName + ", " + m_IMUBaudrate + ")");
                return 1;
            }
            catch (Exception ex)
            {
                Console.WriteLine("Configuration failed(IMU sensor)(" + m_IMUPortName + ", " + m_IMUBaudrate + "): error=" + ex.Message);
                objLogger.LogWrite("IMU connection failed.(" + m_IMUPortName + ", " + m_IMUBaudrate + ")");
                return 0;
            }
        }

        public void StopThread()
        {
            lock (this)
            {
                m_bIMUCommIsOpen = false;
                m_bIMULooping = false;
                m_bIMUThdRunning = false;
                m_bEnableQueue = false;
            }
        }

        public bool GetThreadStatus()
        {
            lock (this)
            {
                return m_bIMUThdRunning;
            }
        }

        public Quaternion GetQuat1stRGuide()
        {
            lock (this)
            {
                return m_qt1stRGuide.q;
            }
        }

        public Quaternion GetQuat1stProbeHolder()
        {
            lock (this)
            {
                return m_qt1stProbeHolder.q;
            }
        }

        public Quaternion GetQuat2ndRGuide()
        {
            lock (this)
            {
                return m_qt2ndRGuide.q;
            }
        }

        public Quaternion GetQuat2ndProbeHolder()
        {
            lock (this)
            {
                return m_qt2ndProbeHolder.q;
            }
        }

        public void EnableQueue()
        {
            //Debug.Log("Eable Q");
            lock (this)
            {
                m_bEnableQueue = true;
            }
        }

        private bool IsQueueEnabled()
        {
            lock (this)
            {
                return m_bEnableQueue;
            }
        }

        public void StartThread()
        {
            m_qIMUOut = Queue.Synchronized(new Queue());
            m_qIMUPosIn = Queue.Synchronized(new Queue());
            m_qIMURotIn = Queue.Synchronized(new Queue());

            /*
            m_IMUComm = new SerialPort(m_IMUPortName, m_IMUBaudrate, Parity.None, 8, StopBits.One);
            m_IMUComm.ReadTimeout = 200;
            m_IMUComm.Open();
            */

            // Creates and starts the thread
            m_pThdIMU = new Thread(IMUThreadLoop);
            m_pThdIMU.Start();

            lock (this)
            {
                m_bIMUCommIsOpen = true;
                m_bIMULooping = true;
                m_bIMUThdRunning = true;
                m_bEnableQueue = false;
            }
        }

        private void pushQueue(string command)
        {
            m_qIMUOut.Enqueue(command);
        }

        public void FlushInQueues()
        {
            m_qIMURotIn.Clear();
            m_qIMUPosIn.Clear();
        }

        public SenVal? PopPosQueue()    //? = to allow return 'null';
        {
            if (m_qIMUPosIn.Count == 0)
            {
                return null;
            }
            return (SenVal)m_qIMUPosIn.Dequeue();
        }

        public SenVal? PopRotQueue()
        {
            if (m_qIMURotIn.Count == 0)
            {
                return null;
            }

            return (SenVal)m_qIMURotIn.Dequeue();
        }

        public bool IsIMULooping()
        {
            lock (this)
            {
                return m_bIMULooping;
            }
        }

        public void WriteToIMU(string message)
        {
            // Send the request
            m_IMUComm.WriteLine(message);
            m_IMUComm.BaseStream.Flush();
        }


        public string ReadFromIMU(int timeout = 0)
        {
            m_IMUComm.ReadTimeout = timeout;
            try
            {
                return m_IMUComm.ReadLine();
            }
            catch (System.TimeoutException)
            {
                return null;
            }
        }

        private string ReadSerialByteData()
        {
            m_IMUComm.ReadTimeout = 100;
            byte[] bytesBuffer = new byte[m_IMUComm.BytesToRead];
            int bufferOffset = 0;
            int bytesToRead = m_IMUComm.BytesToRead;

            Console.WriteLine(m_IMUComm.BytesToRead);

            while (bytesToRead > 0)
            {
                try
                {
                    int readBytes = m_IMUComm.Read(bytesBuffer, bufferOffset, bytesToRead - bufferOffset);
                    bytesToRead -= readBytes;
                    bufferOffset += readBytes;
                }
                catch (TimeoutException ex)
                {
                    Console.WriteLine(ex.ToString());
                }
            }
            
            return bytesBuffer.ToString();
        }

        public void IMUThreadLoop()
        {
#if YKPARK
            // Opens the connection on the serial port

            if (!m_IMUComm.IsOpen)
            {
                PortOpen();
            }

            if (m_IMUComm.IsOpen)
            {
                lock (this)
                {
                    m_bIMUCommIsOpen = true;
                    m_bIMUThdRunning = true;
                }
                m_IMUComm.DiscardOutBuffer();
                m_IMUComm.DiscardInBuffer();
                //string sdata = m_IMUComm.ReadLine(); // dummy read         
            }

            while (IsIMULooping())
            {
                // Read from Receiver
                string result = ReadFromIMU(m_IMUComm.ReadTimeout);
                //string result = ReadSerialByteData();
                //Debug.Log(result);
                
                if (result != null)
                {
                    
                    // payload shoud be start with "100-"
                    if (result.StartsWith("100-"))
                    {
                        string[] vals = result.Split(',');
                        m_iImuID = Convert.ToByte(vals[0].Replace("100-", ""));

                        SenVal sensorData = new SenVal();

                        //Console.WriteLine(result);
                        //objLogger.LogWrite(result);

                        //if (IsQueueEnabled())
                        //{
#if USING_EULERANGLE
                            sensorData.batt = vals[5];
#else
                            sensorData.q = new Quaternion(Convert.ToSingle(vals[1]), Convert.ToSingle(vals[2]), Convert.ToSingle(vals[3]), Convert.ToSingle(vals[4]));
                            sensorData.mat = Matrix4x4.CreateFromQuaternion(sensorData.q);
                            sensorData.batt = vals[5];

                            
#endif
                            switch (m_iImuID)
                            {
                                // 1st R Guide angle
                                case 0:
                                    lock (this) { 
                                        m_qt1stRGuide.q.X = Convert.ToSingle(vals[1]);
                                        m_qt1stRGuide.q.Y = Convert.ToSingle(vals[2]);
                                        m_qt1stRGuide.q.Z = Convert.ToSingle(vals[3]);
                                        m_qt1stRGuide.q.W = Convert.ToSingle(vals[4]);
                                    }
                                    break;

                                // 1st Probe orientation
                                case 1:
                                    lock (this)
                                    {
                                        m_qt1stProbeHolder.q.Y = Convert.ToSingle(vals[1]);
                                        m_qt1stProbeHolder.q.X = Convert.ToSingle(vals[2]);
                                        m_qt1stProbeHolder.q.Z = Convert.ToSingle(vals[3]);
                                        m_qt1stProbeHolder.q.W = Convert.ToSingle(vals[4]);
                                    }
                                    break;
                                // 2nd R Guide angle
                                case 2:
                                    lock (this)
                                    {
                                        m_qt2ndRGuide.q.X = Convert.ToSingle(vals[1]);
                                        m_qt2ndRGuide.q.Y = Convert.ToSingle(vals[2]);
                                        m_qt2ndRGuide.q.Z = Convert.ToSingle(vals[3]);
                                        m_qt2ndRGuide.q.W = Convert.ToSingle(vals[4]);
                                    }
                                    break;

                                // 2nd Probe orientation
                                case 3:
                                    lock (this)
                                    {
                                        m_qt2ndProbeHolder.q.Y = Convert.ToSingle(vals[1]);
                                        m_qt2ndProbeHolder.q.X = Convert.ToSingle(vals[2]);
                                        m_qt2ndProbeHolder.q.Z = Convert.ToSingle(vals[3]);
                                        m_qt2ndProbeHolder.q.W = Convert.ToSingle(vals[4]);
                                    }
                                    break;
                        }
                        //m_IMUComm.DiscardInBuffer();
                        //}
                    }
                    // if payload not start with "100-" then flush receive buffer
                    else
                    {
                        //m_IMUComm.DiscardInBuffer();
                    }

                }
                Thread.Sleep(5);
            }

            m_IMUComm.Close();
#endif
            /*Quaternion q = new Quaternion(0.1f, 0.2f, 0.3f, 0.4f);
            Matrix4x4 mat = Matrix4x4.CreateFromQuaternion(q);
            Quaternion q2 = MatrixToQuaternion(mat);*/

        }

        /*public static Quaternion MatrixToQuaternion(Matrix4x4 m)
        {
            float tr = m.m00 + m.m11 + m.m22;
            float w, x, y, z;
            if (tr > 0.0f)
            {
                float s = Mathf.Sqrt(1f + tr) * 2f;
                w = 0.25f * s;
                x = (m.m21 - m.m12) / s;
                y = (m.m02 - m.m20) / s;
                z = (m.m10 - m.m01) / s;
            }
            else if ((m.m00 > m.m11) && (m.m00 > m.m22))
            {
                float s = Mathf.Sqrt(1f + m.m00 - m.m11 - m.m22) * 2f;
                w = (m.m21 - m.m12) / s;
                x = 0.25f * s;
                y = (m.m01 + m.m10) / s;
                z = (m.m02 + m.m20) / s;
            }
            else if (m.m11 > m.m22)
            {
                float s = Mathf.Sqrt(1f + m.m11 - m.m00 - m.m22) * 2f;
                w = (m.m02 - m.m20) / s;
                x = (m.m01 + m.m10) / s;
                y = 0.25f * s;
                z = (m.m12 + m.m21) / s;
            }
            else
            {
                float s = Mathf.Sqrt(1f + m.m22 - m.m00 - m.m11) * 2f;
                w = (m.m10 - m.m01) / s;
                x = (m.m02 + m.m20) / s;
                y = (m.m12 + m.m21) / s;
                z = 0.25f * s;
            }

            Quaternion quat = new Quaternion(x, y, z, w);
            //Debug.Log("Quat is " + quat.ToString() );
            return quat;
        }*/

        public static Quaternion MatrixToQuaternion(Matrix4x4 m)
        {
            float tr = m.M11 + m.M22 + m.M33;
            float w, x, y, z;
            if (tr > 0.0f)
            {
                float s = (float)Math.Sqrt(1f + tr) * 2f;
                w = 0.25f * s;
                x = (m.M21 - m.M12) / s;
                y = (m.M13 - m.M31) / s;
                z = (m.M21 - m.M12) / s;
            }
            else if ((m.M11 > m.M22) && (m.M11 > m.M33))
            {
                float s = (float)Math.Sqrt(1f + m.M11 - m.M22 - m.M33) * 2f;
                w = (m.M21 - m.M12) / s;
                x = 0.25f * s;
                y = (m.M12 + m.M21) / s;
                z = (m.M13 + m.M31) / s;
            }
            else if (m.M22 > m.M33)
            {
                float s = (float)Math.Sqrt(1f + m.M22 - m.M11 - m.M33) * 2f;
                w = (m.M13 - m.M31) / s;
                x = (m.M12 + m.M21) / s;
                y = 0.25f * s;
                z = (m.M12 + m.M21) / s;
            }
            else
            {
                float s = (float)Math.Sqrt(1f + m.M33 - m.M11 - m.M22) * 2f;
                w = (m.M21 - m.M12) / s;
                x = (m.M13 + m.M31) / s;
                y = (m.M12 + m.M21) / s;
                z = 0.25f * s;
            }

            Quaternion quat = new Quaternion(x, y, z, w);
            //Debug.Log("Quat is " + quat.ToString() );
            return quat;
        }
    }
}
