using System;
using System.Collections;
using System.Text;
using System.IO.Ports;
using System.Threading;
using System.Numerics;

namespace bendodatasrv
{
    public sealed class CommPositionSensor
    {
        private static readonly Lazy<CommPositionSensor> lazy = new Lazy<CommPositionSensor>(() => new CommPositionSensor());

        public static CommPositionSensor Instance { get { return lazy.Value; } }

        private string m_LinearPortName = "COM8";
        private int m_LinearBaudrate = 9600;

        private SerialPort m_LinearComm;
        private Thread m_pThdLinear;
        private Queue m_qLinearOut; // From Unity to Arduino
        private Queue m_qLinearIn; // From Arduino to Unity
        public bool m_bSocketIsOpen = false;
        private static bool m_bLinearLooping = false;
        private static bool m_bLinearCommIsOpen = false;
        private static bool m_bLinearThdRunning = false;
        private static bool m_bEnableQueue = false;
        private float m_fLengthVScale = 3.0f / 1024.0f; // 3V : 10bit AD Converter
        private float m_fLengthDScale = 1000.0f / 3.0f; // 1000mm : 3V
        private float m_fDepthVScale = 3.0f / 1024.0f; // 3V : 10bit AD 
        private float m_fDepthDScale = 500.0f / 3.0f; // 500mm : 3V 
        private float m_fLength = 0.0f;
        private float m_fDepth = 0.0f;
        private bool m_bOffsetIsInit = false;
        private float m_fLengthOffset = 0.0f;
        private float m_fDepthOffset = 0.0f;
        public int m_seq = 0;
        private Logger objLogger;
        private byte[] REQ_FIRST_ENCODER;
        private byte[] REQ_SECOND_ENCODER;
        private byte[] RESET_FIRST_ENCODER;
        private byte[] RESET_SECOND_ENCODER;
        private bool m_bCounterInit = false;
        private byte m_targetAddr=1;

        private CommPositionSensor()
        {
            objLogger = Logger.Instance;

            REQ_FIRST_ENCODER = new byte[] {0x01, 0x04, 0x03, 0xEB, 0x00, 0x02, 0x01, 0xBB};
            REQ_SECOND_ENCODER = new byte[] { 0x02, 0x04, 0x03, 0xEB, 0x00, 0x02, 0x01, 0x88};
            RESET_FIRST_ENCODER = new byte[] { 0x01, 0x05, 0x00, 0x00, 0xff, 0x00, 0x8c, 0x3a};
            RESET_SECOND_ENCODER = new byte[] { 0x02, 0x05, 0x00, 0x00, 0xff, 0x00, 0x8c, 0x09};
        }

        public int SetPort(string pName, int baud)
        {
            int retVal = 0;
            m_LinearPortName = pName;
            m_LinearBaudrate = baud;

            retVal = PortOpen();

            return retVal;
        }

        private int PortOpen()
        {
            try
            {
                m_LinearComm = new SerialPort(m_LinearPortName, m_LinearBaudrate, Parity.None, 8, StopBits.Two);
                m_LinearComm.ReadTimeout = 10;
                m_LinearComm.Open();

                Console.WriteLine("Configured serialport for position sensor.(" + m_LinearPortName + ", " + m_LinearBaudrate + ")");
                objLogger.LogWrite("Position sensor configured(" + m_LinearPortName + ", " + m_LinearBaudrate + ")");
                return 1;
            }
            catch (Exception ex)
            {
                Console.WriteLine("Configuration failed(position sensor)(" + m_LinearPortName + ", " + m_LinearBaudrate + "): error=" + ex.Message);
                objLogger.LogWrite("position sensor connection failed.(" + m_LinearPortName + ", " + m_LinearBaudrate + ")");
                return 0;
            }
        }

        public void StopThread()
        {
            lock (this)
            {
                m_bLinearCommIsOpen = false;
                m_bLinearLooping = false;
                m_bLinearThdRunning = false;
            }
        }

        public float GetDepth()
        {
            lock (this) {
                return m_fDepth;
            }
        }

        public float GetLehgth()
        {
            lock (this)
            {
                return m_fLength;
            }
        }

        public bool GetThreadStatus()
        {
            lock (this)
            {
                return m_bLinearThdRunning;
            }
        }

        public void EnableQueue()
        {
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
            m_qLinearOut = Queue.Synchronized(new Queue());
            m_qLinearIn = Queue.Synchronized(new Queue());

            m_bLinearLooping = true;
            // Creates and starts the m_pThdLinear
            m_pThdLinear = new Thread(LinearThreadLoop);
            //Debug.Log("Started.");
            m_pThdLinear.Start();

        }

        private void pushQueue(string command)
        {
            m_qLinearOut.Enqueue(command);
            //Debug.Log("added cmd["+outputQueue.Count+"]");
        }

        public void FlushInQueue()
        {
            m_qLinearIn.Clear();
        }

        public void FlushOutQueue()
        {
            m_qLinearOut.Clear();
        }

        public string PopQueue()
        {
            if (m_qLinearIn.Count == 0)
            {
                return null;
            }

            return (string)m_qLinearIn.Dequeue();
        }

        public bool IsLinearLooping()
        {
            lock (this)
            {
                return m_bLinearLooping;
            }
        }

        private void WriteToLinearSenor(byte[] message)
        {
            m_LinearComm.Write(message, 0, message.Length);
            //m_LinearComm.DiscardOutBuffer();
            m_LinearComm.BaseStream.Flush();
        }
        /*
        private void WriteToLinearSenor(string message)
        {
            m_LinearComm.WriteLine(message);
            m_LinearComm.BaseStream.Flush();
        }
        */
        private byte[] ReadSerialByteData()
        {
            m_LinearComm.ReadTimeout = 100;
            int bytesToRead = m_LinearComm.BytesToRead;
            byte[] bytesBuffer = new byte[bytesToRead];
            int bufferOffset = 0;

            while (bytesToRead > 0)
            {
                try
                {
                    int readBytes = m_LinearComm.Read(bytesBuffer, bufferOffset, bytesToRead - bufferOffset);
                    bytesToRead -= readBytes;
                    bufferOffset += readBytes;
                    
                }
                catch (TimeoutException ex)
                {
                    Console.WriteLine(ex.ToString());
                }
            }

            return bytesBuffer;
        }

        private void ReadFromLinearSensor(ref byte[] rcvBuf, int timeout = 40)
        {
            m_LinearComm.ReadTimeout = 100;
            int bytesToRead = m_LinearComm.BytesToRead;
            byte[] bytesBuffer = new byte[m_LinearComm.BytesToRead];
            int bufferOffset = 0;

            if ((bytesToRead >= 5)&& (bytesToRead < 10))
            {
                try
                {
                    int readBytes = m_LinearComm.Read(bytesBuffer, bufferOffset, bytesToRead - bufferOffset);
                    bytesToRead -= readBytes;
                    bufferOffset += readBytes;
                    m_LinearComm.DiscardOutBuffer();
                    m_LinearComm.DiscardInBuffer();
                    
                }
                catch (TimeoutException ex)
                {
                    Console.WriteLine(ex.ToString());
                }
                //Array.Copy(bytesBuffer, rcvBuf, bytesToRead);
                bytesBuffer.CopyTo(rcvBuf, 0);
            }
            //Console.WriteLine("bytesToRead = " + bytesToRead + "\t|\trcv buf len=" + bytesBuffer.Length);
            
            //
            /*
            int i = 0;

            ReadSerialByteData();

            m_LinearComm.ReadTimeout = timeout;

            try
            {
                for(i = 0; i < rcvBuf.Length; i++)
                { 
                    rcvBuf[i] = (byte)m_LinearComm.ReadByte();
                }
                //m_LinearComm.ReadExisting();
            }
            catch (System.TimeoutException)
            {
                //Debug.Log("Read Timeout");
                Console.WriteLine("Serial read timeout!");
            }
            */
        }


        private void LinearThreadLoop()
        {
            //string length;
            //string depth;
            //string lnd;
            float tmpLen;
            float tmpDep;
            StringBuilder lnd = new StringBuilder();
            // Opens the connection on the serial m_LinearPortName
            if (!m_LinearComm.IsOpen) {
                PortOpen();
            }

            if (m_LinearComm.IsOpen)
            {
                lock (this)
                {
                    m_bLinearCommIsOpen = true;
                    m_bLinearThdRunning = true;
                }
                m_LinearComm.DiscardOutBuffer();
                m_LinearComm.DiscardInBuffer();
                //m_LinearComm.ReadLine(); // dummy read
            }
            
            while (IsLinearLooping())
            {
                byte[] response = new byte[9];

                WriteToLinearSenor(REQ_FIRST_ENCODER);
                Delay(50);
                ReadFromLinearSensor(ref response, 0);
                m_fLength = ParsingData(ref response);
                //Console.WriteLine(response[0].ToString() + "]L = " + m_fLength);
                Delay(10);

                WriteToLinearSenor(REQ_SECOND_ENCODER);
                Delay(50);
                ReadFromLinearSensor(ref response, 0);
                m_fDepth = ParsingData(ref response);
                //Console.WriteLine(response[0].ToString() + "]D = " + m_fDepth);
                Delay(10);
                /*
                if (m_targetAddr == 1)
                {
                    WriteToLinearSenor(REQ_FIRST_ENCODER);
                }
                else
                {
                    WriteToLinearSenor(REQ_SECOND_ENCODER);
                }

                Delay(4);
                ReadFromLinearSensor(ref response, 30);
                
                if (response[0] == 1)
                {
                    m_fLength = ParsingData(ref response);
                    Console.WriteLine("["+m_targetAddr+"|"+response[0].ToString()+"]Length = " + m_fLength + "\t|\tDepth = " + m_fDepth);
                    m_targetAddr = 2;
                }
                else if (response[0] == 2)
                {
                    m_fDepth = ParsingData(ref response);
                    Console.WriteLine("[" + m_targetAddr + "|" + response[0].ToString() + "]Length = " + m_fLength + "\t|\tDepth = " + m_fDepth);
                    m_targetAddr = 1;
                }
                */



            }

            m_LinearComm.Close();
            
            /*
            while (true)
            {
                if (m_bSocketIsOpen)
                {
                    Console.Write(m_seq + "aaa\r\n");
                    m_seq++;
                }
                Thread.Sleep(500);
            }
            */
        }

        private static DateTime Delay(int MS)
        {
            DateTime ThisMoment = DateTime.Now;
            TimeSpan duration = new TimeSpan(0, 0, 0, 0, MS);
            DateTime AfterWards = ThisMoment.Add(duration);

            while (AfterWards >= ThisMoment)
            {
                System.Windows.Forms.Application.DoEvents();
                ThisMoment = DateTime.Now;
            }

            return DateTime.Now;
        }

        public void CounterInit() {
            string res_1st = string.Empty;
            string res_2nd = string.Empty;
            string command = string.Empty;
            byte[] tmp = new byte[8];
            
            WriteToLinearSenor(RESET_FIRST_ENCODER);
            Delay(5);
            ReadFromLinearSensor(ref tmp, 30);
            Delay(100);
            
            WriteToLinearSenor(RESET_SECOND_ENCODER);
            Delay(5);
            ReadFromLinearSensor(ref tmp, 30);
            Delay(100);
            
            Console.WriteLine("Reset Counter");
        }

        private float ParsingData(ref byte[] res)
        {
            float retVal = 0.0f;
            byte[] data = new byte[4];

            data[0] = res[4];
            data[1] = res[3];
            data[2] = res[6];
            data[3] = res[5];

            if (data[3] != 0xFF) //positive value
            {
                retVal = (BitConverter.ToInt32(data, 0)/1000.0f);
                //Console.WriteLine(retVal);
            }
            else if (data[3] == 0xff) //negative value
            {
                byte[] toConv = new byte[] { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff };
                retVal = ((BitConverter.ToInt32(data, 0) - BitConverter.ToInt64(toConv,0) + 1)/1000.0f);
            }

            return retVal;
        }

        // Response to request message for configuration of sensor connection
        private string RequestMsg(byte addr)
        {
            byte[] reqmsg = new byte[4];
            int cmd = 1;
            int msgIdx = 0;

            reqmsg[msgIdx] = addr; 
            reqmsg[++msgIdx] = 0x04;
            reqmsg[++msgIdx] = 0x03;
            reqmsg[++msgIdx] = 0xEB;
            reqmsg[++msgIdx] = 0x00;
            reqmsg[++msgIdx] = 0x02;

            return Encoding.Default.GetString(reqmsg);
        }

        public static ushort CalcCRC(byte[] strPacket)
        {
            ushort[] CRC16_TABLE = { 0x0000, 0xCC01, 0xD801, 0x1400, 0xF001, 0x3C00, 0x2800, 0xE401, 0xA001, 0x6C00, 0x7800, 0xB401, 0x5000, 0x9C01, 0x8801, 0x4400 };
            ushort usCRC = 0xFFFF;
            ushort usTemp = 0;

            foreach (char cCurrent in strPacket)
            {
                byte bytCurrent = Convert.ToByte(cCurrent);// lower 4 bits
                usTemp = CRC16_TABLE[usCRC & 0x000F];
                usCRC = (ushort)((usCRC >> 4) & 0x0FFF);
                usCRC = (ushort)(usCRC ^ usTemp ^ CRC16_TABLE[bytCurrent & 0x000F]); // Upper 4 Bits
                usTemp = CRC16_TABLE[usCRC & 0x000F];
                usCRC = (ushort)((usCRC >> 4) & 0x0FFF);
                usCRC = (ushort)(usCRC ^ usTemp ^ CRC16_TABLE[(bytCurrent >> 4) & 0x000F]);
            }
            return usCRC;
        }
    }
}


