using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Net;
using System.Numerics;
using System.IO.Ports;



namespace bendodatasrv
{
    public sealed class USSocketServer
    {
        private static readonly Lazy<USSocketServer> lazy = new Lazy<USSocketServer>(() => new USSocketServer());

        public static USSocketServer Instance { get { return lazy.Value; } }

        public string mUSPath;
        private static SerialPort m_LinkComSv;


        private byte FIXEDLen = 8; //STX(1) + DIR(1) + CMD(1) + DATALEN(4) + ETX(1)

        private const byte STX = 0x02;

        private const byte DIR_REQ = 0x51;
        private const byte DIR_RES = 0x52;

        private const byte CMD_SENSOR_CONF_REQ = 0x43;
        private const byte CMD_SENSOR_CONF_RES = 0x43;

        private const byte CMD_US_SCAN_ROI_REQ = 0x6F;
        private const byte CMD_US_SCAN_ROI_RES = 0x6F;

        private const byte CMD_US_POS_START_REQ = 0x64;
        private const byte CMD_US_POS_STOP_REQ = 0x74;

        private const byte CMD_US_SCAN_START_REQ = 0x73;
        private const byte CMD_US_SCAN_RES = 0x73;
        private const byte CMD_US_SCAN_STOP_REQ = 0x65;

        private const byte CMD_POINT_START_REQ = 0x70;
        private const byte CMD_POINT_RES = 0x70;
        private const byte CMD_POINT_STOP_REQ = 0x71;

        private const byte ETX = 0x03;

        private string m_LinkPortName = "COM3";
        private int m_LinkBaudrate = 9600;
        private static bool m_bLinkComIsOpen = false;

        private static byte[] link_rbuffer = new byte[250];
        private static int link_rbuffer_pos = 0;

        public byte m_bTargetPort;

        private Logger objLogger;
        private FrameCapture objFrame;

        private USSocketServer()
        {
            // 비동기 작업에 사용될 대리자를 초기화합니다.
            objLogger = Logger.Instance;
            objFrame = FrameCapture.Instance;


        }

        public int SetPort(string pName, int baud)
        {
            int retVal = 0;
            m_LinkPortName = pName;
            m_LinkBaudrate = baud;

            retVal = PortOpen();
            return retVal;
        }

        private int PortOpen()
        {
            try
            {
                m_LinkComSv = new SerialPort(m_LinkPortName, m_LinkBaudrate, Parity.None, 8, StopBits.One);
                m_LinkComSv.DataReceived += new SerialDataReceivedEventHandler(DataReceivedHandler);

                m_LinkComSv.ReadTimeout = 200;
                m_LinkComSv.Open();
                m_bLinkComIsOpen = true;

                Console.WriteLine("Serial port configuration is successful. (" + m_LinkPortName + ", " + m_LinkBaudrate + ")");
                objLogger.LogWrite("Serial port configuration is successful. (" + m_LinkPortName + ", " + m_LinkBaudrate + ")");
                return 1;
            }

            catch (Exception ex)
            {
                Console.WriteLine("Serial port configuration is failed. (" + m_LinkPortName + ", " + m_LinkBaudrate + ")");
                objLogger.LogWrite("Serial port configuration is failed. (" + m_LinkPortName + ", " + m_LinkBaudrate + ")");
                return 0;
            }
        }

        private void DataReceivedHandler(object client, SerialDataReceivedEventArgs e)
        {
            SerialPort server = (SerialPort)client;
            int sv;

            while (server.BytesToRead > 0)
            {
                sv = server.ReadByte();

                if (sv == STX) // 
                {
                    link_rbuffer_pos = 0;
                }

                link_rbuffer[link_rbuffer_pos] = (byte)sv;
                link_rbuffer_pos++;

                if (sv == ETX)
                {
                    link_rbuffer[link_rbuffer_pos] = 0x00;
                    this.putToBuff(ByteToString(link_rbuffer));

                    link_rbuffer_pos = 0;
                }

            }
        }


        public void SendMessage(String message)
        {
            // 추가 정보를 넘기기 위한 변수 선언
            // 크기를 설정하는게 의미가 없습니다.
            // 왜냐하면 바로 밑의 코드에서 문자열을 유니코드 형으로 변환한 바이트 배열을 반환하기 때문에
            // 최소한의 크기르 배열을 초기화합니다.
            
            string emsg = string.Empty;

            // 문자열을 바이트 배열으로 변환
            

            // 사용된 소켓을 저장
            //ao.WorkingSocket = m_ServerSocket;
            

            // 전송 시작!
            try
            {
                
            }
            catch (Exception ex)
            {
                emsg = "Error on sending. " +ex.Message;
                Console.WriteLine(emsg);
                objLogger.LogWrite(emsg);
                objFrame.StopCatpure();
                
            }
        }

        public void MessageParsing(byte[] msg)
        {
            Byte stx = msg[0];
            Byte direction = msg[1];
            Byte cmd = msg[2];
            int dataLen = GetDataLength(ref msg);

            if (stx == STX && direction == DIR_REQ)
            {
                switch (cmd)
                {
                    case CMD_US_POS_START_REQ: //통신연결 후 초기 위치 확인용 '초음파 진단부 위치 전송 시작요청'
                        Console.WriteLine("<< CMD_US_POS_START_REQ");
                        objLogger.LogWrite("[RX]CMD_US_POS_START_REQ");
                        objFrame.SetCaptureMode(FrameCapture.CALIBRATION);
                        objFrame.StartCatpure();
                        //ResponseInitPos();
                        break;

                    case CMD_US_POS_STOP_REQ: //통신연결 후 초기 위치 확인용 '초음파 진단부 위치 전송 시작요청'
                        Console.WriteLine("<< CMD_US_POS_STOP_REQ");
                        objLogger.LogWrite("[RX]CMD_US_POS_STOP_REQ");
                        objFrame.StopCatpure();
                        break;

                    case CMD_US_SCAN_START_REQ: // '초음파 스캔 시작(RoI 및 저장경로 설정)'
                        Console.WriteLine("<< CMD_US_SCAN_START_REQ");
                        objLogger.LogWrite("[RX]CMD_US_SCAN_START_REQ");
                        ParsingUSRoIRect(ref msg); // parsing RoI Rectangle
                        //Console.WriteLine("(" + mRoIOriginX + ", " + mRoIOriginY + "), w = " + mRoIWidth + ", h = " + mRoIHeight);
                        ParsingUSSaveDir(ref msg, dataLen);
                        objFrame.SetCaptureMode(FrameCapture.SCAN_US);
                        objFrame.StartCatpure();
                        break;

                    case CMD_US_SCAN_STOP_REQ: // '초음파 스캔 종료'
                        Console.WriteLine("<< CMD_US_SCAN_STOP_REQ");
                        objLogger.LogWrite("[RX]CMD_US_SCAN_START_REQ");
                        objFrame.StopCatpure();
                        break;

                    case CMD_POINT_START_REQ: // '포트 위치 전송 시작 요청'
                        Console.WriteLine("<< CMD_POINT_START_REQ");
                        objLogger.LogWrite("[RX]CMD_POINT_START_REQ");
                        objFrame.SetCaptureMode(FrameCapture.PORT_POSITION);
                        ParsingTargetPortNum(ref msg, dataLen);
                        objFrame.StartCatpure();
                        break;

                    case CMD_POINT_STOP_REQ: // '포트 위치 전송 종료 요청'
                        Console.WriteLine("<< CMD_POINT_STOP_REQ");
                        objLogger.LogWrite("[RX]CMD_POINT_STOP_REQ");
                        objFrame.StopCatpure();
                        break;

                    case CMD_US_SCAN_ROI_REQ: // 'ROI 세팅용 이미지 1 Frame 요청'
                        Console.WriteLine("<< CMD_US_SCAN_ROI_REQ");
                        objLogger.LogWrite("[RX]CMD_US_SCAN_ROI_REQ");
                        ParsingUSRoiSaveDir(ref msg, dataLen); // 초음파 스캔 시작 시 지정하는 경로를 변경하도록 우선 설정(추후 분리 필요)
                        objFrame.GetUSFrameForRoi();
                        break;
                }
            }
        }

        private int GetDataLength(ref byte[] msg)
        {
            return ((msg[3] - 0x30) * 1000) + ((msg[4] - 0x30) * 100) + ((msg[5] - 0x30) * 10) + ((msg[6] - 0x30));
        }

        //-------------------------------------------------------------------------------
        //  위치/자세 센서수신기 연결용 COM PORT 설정
        //
        //  input: 수신 메시지(msg)
        //  
        //  수신 메시지에서 두 쌍의 COM PORT번호, BAUDRATE 값을 추출하여 IMU센서와 위치센서 연결
        //-------------------------------------------------------------------------------


        //-------------------------------------------------------------------------------
        //  초음파 RoI 설정
        //
        //  input: 수신 메시지(msg)
        //  
        //  수신 메시지에서 초음파 영상의 RoI 영역 추출하여 SRV의 RoI 변수에 저장
        //  (시작 픽셀위치(x, y), crop할 width, height)
        //-------------------------------------------------------------------------------
        private void ParsingUSRoIRect(ref byte[] msg)
        {
            int x;
            int y;
            int w;
            int h;
            x = ((msg[7] - 0x30) * 1000) + ((msg[8] - 0x30) * 100) + ((msg[9] - 0x30) * 10) + ((msg[10] - 0x30));
            y = ((msg[11] - 0x30) * 1000) + ((msg[12] - 0x30) * 100) + ((msg[13] - 0x30) * 10) + ((msg[14] - 0x30));
            w = ((msg[15] - 0x30) * 1000) + ((msg[16] - 0x30) * 100) + ((msg[17] - 0x30) * 10) + ((msg[18] - 0x30));
            h = ((msg[19] - 0x30) * 1000) + ((msg[20] - 0x30) * 100) + ((msg[21] - 0x30) * 10) + ((msg[22] - 0x30));

            objFrame.SetUSRoI(x, y, w, h);
            objLogger.LogWrite("RoI = ("+x+", "+y+") w:"+w+", h:"+h);
        }

        //-------------------------------------------------------------------------------
        //  초음파 프레임 저장 경로 설정
        //
        //  input: 수신 메시지(msg), 저장경로 문자열 길이(len)
        //  
        //  수신 메시지에서 저장경로 문자열을 추출하여 SRV의 초음파 영상 저장 경로 전역변수에 저장
        //-------------------------------------------------------------------------------
        private void ParsingUSSaveDir(ref byte[] msg, int len)
        {
            byte[] savePath = new byte[len - 16];
            Buffer.BlockCopy(msg, 23, savePath, 0, (len - 16));
            string path = ByteToString(savePath);
            objFrame.SetUSStorePath(path);
            objLogger.LogWrite("US store path = "+path);
        }

        //-------------------------------------------------------------------------------
        //  초음파 ROI 설정용 저장 경로 설정
        //
        //  input: 수신 메시지(msg), 저장경로 문자열 길이(len)
        //  
        //  수신 메시지에서 저장경로 문자열을 추출하여 SRV의 초음파 영상 저장 경로 전역변수에 저장
        //-------------------------------------------------------------------------------
        private void ParsingUSRoiSaveDir(ref byte[] msg, int len)
        {
            byte[] savePath = new byte[len];
            Buffer.BlockCopy(msg, 7, savePath, 0, len);
            string path = ByteToString(savePath);
            objFrame.SetUSStorePath(path);
            objLogger.LogWrite("US ROI store path = " + path);
        }

        //-------------------------------------------------------------------------------
        //  내시경 프레임 저장 경로 설정
        //
        //  input: 수신 메시지(msg), 저장경로 문자열 길이(len)
        //  
        //  수신 메시지에서 저장경로 문자열을 추출하여 SRV의 내시경 영상 저장 경로 전역변수에 저장
        //-------------------------------------------------------------------------------
        private void ParsingESSaveDir(ref byte[] msg, int len)
        {
            byte[] savePath = new byte[len];
            Buffer.BlockCopy(msg, 7, savePath, 0, len);
            string path = ByteToString(savePath);
            objFrame.SetESStorePath(path);
            objLogger.LogWrite("ES store path = " + path);
        }

        //-------------------------------------------------------------------------------
        //  위치확인 대상 포트 번호 설정
        //
        //  input: 수신 메시지(msg), 저장경로 문자열 길이(len)
        //  
        //  수신 메시지에서 포트번호 추출
        //-------------------------------------------------------------------------------
        private void ParsingTargetPortNum(ref byte[] msg, int len)
        {
            byte[] savePath = new byte[len];
            Buffer.BlockCopy(msg, 7, savePath, 0, len);
            m_bTargetPort = msg[7];
            m_bTargetPort -= 0x30;
            objLogger.LogWrite("Target port = " + m_bTargetPort);
        }

        private byte[] StringToByte(string str)
        {
            byte[] StrByte = Encoding.ASCII.GetBytes(str);
            return StrByte;
        }

        public string ByteToString(byte[] strByte)
        {
            string str = Encoding.Default.GetString(strByte);
            return str;
        }

        // Response to request message for configuration of sensor connection
        private void ResponseSensorConf(int confResult)
        {
            byte[] resmsg = new byte[9];
            int dataLen = 1;
            int msgIdx = 0;

            resmsg[msgIdx] = STX;
            resmsg[++msgIdx] = DIR_RES;
            resmsg[++msgIdx] = CMD_SENSOR_CONF_RES;
            AddDataLen(ref resmsg, dataLen, ref msgIdx);
            AddCommConfResult(ref resmsg, ref msgIdx, confResult);
            resmsg[msgIdx] = ETX;
            SendMessage(Encoding.Default.GetString(resmsg));
            objLogger.LogWrite("[TX]CMD_SENSOR_CONF_RES|" + Encoding.ASCII.GetString(resmsg));
            Console.WriteLine(">> CMD_SENSOR_CONF_RES");
        }

        // Response to request message for the initial position of the probe
        public void ResponsePosCalib(Quaternion q, Vector3 pos)
        {
            byte[] resmsg = new byte[71];
            int dataLen = 63;
            int msgIdx = 0;

            resmsg[msgIdx] = STX;
            resmsg[++msgIdx] = DIR_RES;
            resmsg[++msgIdx] = CMD_US_POS_START_REQ;
            AddDataLen(ref resmsg, dataLen, ref msgIdx);
            AddQuaternion(ref resmsg, ref msgIdx, q.X, q.Y, q.Z, q.W);
            AddPosition(ref resmsg, ref msgIdx, pos.X, pos.Y, pos.Z);
            resmsg[msgIdx] = ETX;
            SendMessage(Encoding.Default.GetString(resmsg));
            objLogger.LogWrite("[TX]CMD_US_POS_RES|" + Encoding.ASCII.GetString(resmsg));
            Console.WriteLine(">> CMD_US_POS_RES");
        }

        // Response to request message for the US scan
        public void ResponseUSScan(string imgName, Quaternion q, Vector3 position)
        {
            int dataLen = 63 + imgName.Length;
            byte[] resmsg = new byte[(dataLen + FIXEDLen)];            
            int msgIdx = 0;

            resmsg[msgIdx] = STX;
            resmsg[++msgIdx] = DIR_RES;
            resmsg[++msgIdx] = CMD_US_SCAN_RES;
            AddDataLen(ref resmsg, dataLen, ref msgIdx);
            AddQuaternion(ref resmsg, ref msgIdx, q.X, q.Y, q.Z, q.W);
            AddPosition(ref resmsg, ref msgIdx, position.X, position.Y, position.Z);
            AddFileName(ref resmsg, ref msgIdx, imgName);
            resmsg[msgIdx] = ETX;
            SendMessage(Encoding.Default.GetString(resmsg));
            objLogger.LogWrite("[TX]CMD_US_SCAN_RES|" + Encoding.ASCII.GetString(resmsg));
            Console.WriteLine(">> CMD_US_SCAN_RES|"+ imgName);
        }

        // Response to request message for the initial position of the probe
        public void ResponsePortPosition(Quaternion q, Vector3 pos)
        {
            int dataLen = 64;
            byte[] resmsg = new byte[(dataLen + FIXEDLen)];
            int msgIdx = 0;

            resmsg[msgIdx] = STX;
            resmsg[++msgIdx] = DIR_RES;
            resmsg[++msgIdx] = CMD_POINT_RES;
            AddDataLen(ref resmsg, dataLen, ref msgIdx);
            AddPortNum(ref resmsg, ref msgIdx, m_bTargetPort);
            AddQuaternion(ref resmsg, ref msgIdx, q.X, q.Y, q.Z, q.W);
            AddPosition(ref resmsg, ref msgIdx, pos.X, pos.Y, pos.Z);
            resmsg[msgIdx] = ETX;
            SendMessage(Encoding.Default.GetString(resmsg));
            objLogger.LogWrite("[TX]CMD_POINT_RES|" + Encoding.ASCII.GetString(resmsg));
            Console.WriteLine(">> CMD_POINT_RES");
        }

        // Response to request message for the ES scan
        public void ResponseESScan(string imgName)
        {
            int dataLen = imgName.Length;
            byte[] resmsg = new byte[(dataLen + FIXEDLen)];
            int msgIdx = 0;

            resmsg[msgIdx] = STX;
            resmsg[++msgIdx] = DIR_RES;
            resmsg[++msgIdx] = CMD_US_SCAN_RES;
            AddDataLen(ref resmsg, dataLen, ref msgIdx);
            AddFileName(ref resmsg, ref msgIdx, imgName);
            resmsg[msgIdx] = ETX;
            SendMessage(Encoding.Default.GetString(resmsg));
            //objLogger.LogWrite("[TX]CMD_US_SCAN_RES|" + Encoding.ASCII.GetString(resmsg));
            //Console.WriteLine(">> CMD_US_SCAN_RES|"+ imgName);
        }

        // Response to request message for the US scan for ROI
        public void ResponseUSScanForRoi(string imgName)
        {
            int dataLen = imgName.Length;
            byte[] resmsg = new byte[(dataLen + FIXEDLen)];
            int msgIdx = 0;

            resmsg[msgIdx] = STX;
            resmsg[++msgIdx] = DIR_RES;
            resmsg[++msgIdx] = CMD_US_SCAN_ROI_RES;
            AddDataLen(ref resmsg, dataLen, ref msgIdx);
            AddFileName(ref resmsg, ref msgIdx, imgName);
            resmsg[msgIdx] = ETX;
            SendMessage(Encoding.Default.GetString(resmsg));
        }

        private void AddPortNum(ref byte[] msgBuf, ref int msgIdx, byte portNum)
        {
            int size = 1;
            byte[] portIdx = new byte[size];

            portIdx[0] = portNum;
            Buffer.BlockCopy(portIdx, 0, msgBuf, msgIdx, size);
            msgIdx += size;
        }

        // Add data length field to message buffer
        private void AddDataLen(ref byte[] msgBuf, int dataLen, ref int msgIdx)
        {
            byte[] lenArr = new byte[4];
            lenArr = Encoding.ASCII.GetBytes(dataLen.ToString("D4"));
            Buffer.BlockCopy(lenArr, 0, msgBuf, ++msgIdx, 4);
            msgIdx += 4;
        }

        // Add quaternion field to message buffer
        private void AddCommConfResult(ref byte[] msgBuf, ref int msgIdx, int result)
        {
            int size = 1;
            byte[] res = new byte[size];

            res = Encoding.ASCII.GetBytes(result.ToString());
            Buffer.BlockCopy(res, 0, msgBuf, msgIdx, size);
            msgIdx += size;
        }

        // Add quaternion field to message buffer
        private void AddQuaternion(ref byte[] msgBuf, ref int msgIdx, float qX, float qY, float qZ, float qW)
        {
            int size = 9;
            byte[] qi = new byte[size];

            qi = Encoding.ASCII.GetBytes(String.Format("{0,9:F3}", qX));
            Buffer.BlockCopy(qi, 0, msgBuf, msgIdx, size);
            msgIdx += size;

            qi = Encoding.ASCII.GetBytes(String.Format("{0,9:F3}", qY));
            Buffer.BlockCopy(qi, 0, msgBuf, msgIdx, size);
            msgIdx += size;

            qi = Encoding.ASCII.GetBytes(String.Format("{0,9:F3}", qZ));
            Buffer.BlockCopy(qi, 0, msgBuf, msgIdx, size);
            msgIdx += size;

            qi = Encoding.ASCII.GetBytes(String.Format("{0,9:F3}", qW));
            Buffer.BlockCopy(qi, 0, msgBuf, msgIdx, size);
            msgIdx += size;
        }

        // Add position field to message buffer
        private void AddPosition(ref byte[] msgBuf, ref int msgIdx, float posX, float posY, float posZ)
        {
            int size = 9;
            byte[] posi = new byte[size];
            posi = Encoding.ASCII.GetBytes(String.Format("{0,9:F3}", posX));
            Buffer.BlockCopy(posi, 0, msgBuf, msgIdx, size);
            msgIdx += size;

            posi = Encoding.ASCII.GetBytes(String.Format("{0,9:F3}", posY));
            Buffer.BlockCopy(posi, 0, msgBuf, msgIdx, size);
            msgIdx += size;

            posi = Encoding.ASCII.GetBytes(String.Format("{0,9:F3}", posZ));
            Buffer.BlockCopy(posi, 0, msgBuf, msgIdx, size);
            msgIdx += size;
        }

        private void AddFileName(ref byte[] msgBuf, ref int msgIdx, string fName)
        {
            int size = fName.Length;
            Buffer.BlockCopy(StringToByte(fName), 0, msgBuf, msgIdx, size);
            msgIdx += size;
        }

        private static byte[] MakeLenArray(int len)
        {
            byte[] lenArr = new byte[4];
            lenArr = Encoding.ASCII.GetBytes(len.ToString("D4"));
            return lenArr;
        }
    }

    
}
