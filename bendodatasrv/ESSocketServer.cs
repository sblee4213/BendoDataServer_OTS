using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Net;
using System.Net.Sockets;
using System.Numerics;



namespace bendodatasrv
{
    public sealed class ESSocketServer
    {
        private static readonly Lazy<ESSocketServer> lazy = new Lazy<ESSocketServer>(() => new ESSocketServer());

        public static ESSocketServer Instance { get { return lazy.Value; } }

        public string mUSPath;
        public string mESPath;
        public bool mIsSocketConnected;

        private byte FIXEDLen = 8; //STX(1) + DIR(1) + CMD(1) + DATALEN(4) + ETX(1)

        private const byte STX = 0x02;

        private const byte DIR_REQ = 0x51;
        private const byte DIR_RES = 0x52;

        private const byte CMD_SENSOR_CONF_REQ = 0x43;
        private const byte CMD_SENSOR_CONF_RES = 0x43;

        private const byte CMD_US_POS_START_REQ = 0x64;
        private const byte CMD_US_POS_RES = 0x64;
        private const byte CMD_US_POS_STOP_REQ = 0x74;

        private const byte CMD_US_SCAN_START_REQ = 0x73;
        private const byte CMD_US_SCAN_RES = 0x73;
        private const byte CMD_US_SCAN_STOP_REQ = 0x65;

        private const byte CMD_POINT_START_REQ = 0x70;
        private const byte CMD_POINT_RES = 0x70;
        private const byte CMD_POINT_STOP_REQ = 0x71;

        private const byte CMD_ES_START_REQ = 0x69;
        private const byte CMD_ES_RES = 0x69;
        private const byte CMD_ES_STOP_REQ = 0x72;

        private const byte ETX = 0x03;

        public class AsyncObject
        {
            public Byte[] Buffer;
            public Socket WorkingSocket;
            public AsyncObject(Int32 bufferSize)
            {
                this.Buffer = new Byte[bufferSize];
            }
        }

        private Socket m_ConnectedClient = null;
        private Socket m_ServerSocket = null;
        private AsyncCallback m_fnReceiveHandler;
        private AsyncCallback m_fnSendHandler;
        private AsyncCallback m_fnAcceptHandler;
        private Logger objLogger;
        private FrameCapture objFrame;

        private ESSocketServer()
        {
            // 비동기 작업에 사용될 대리자를 초기화합니다.
            objLogger = Logger.Instance;
            objFrame = FrameCapture.Instance;

            m_fnReceiveHandler = new AsyncCallback(handleDataReceive);
            m_fnSendHandler = new AsyncCallback(handleDataSend);
            m_fnAcceptHandler = new AsyncCallback(handleClientConnectionRequest);
            mIsSocketConnected = false;
        }

        public void StartServer(int port)
        {
            // TCP 통신을 위한 소켓을 생성합니다.
            m_ServerSocket = new Socket(AddressFamily.InterNetwork, SocketType.Stream, ProtocolType.IP);
            Console.WriteLine("ES Socket created");
            objLogger.LogWrite("ES Socket created");

            // 특정 포트에서 모든 주소로부터 들어오는 연결을 받기 위해 포트를 바인딩합니다.
            // 사용한 포트: 5678
            m_ServerSocket.Bind(new IPEndPoint(IPAddress.Any, port));
            Console.WriteLine("Port bind with " + port);
            objLogger.LogWrite("Port bined");

            // 연결 요청을 받기 시작합니다.
            m_ServerSocket.Listen(5);
            Console.WriteLine("ES Socket listening ... ");
            objLogger.LogWrite("ES Socket listening ... ");

            // BeginAccept 메서드를 이용해 들어오는 연결 요청을 비동기적으로 처리합니다.
            // 연결 요청을 처리하는 함수는 handleClientConnectionRequest 입니다.
            m_ServerSocket.BeginAccept(m_fnAcceptHandler, null);
        }

        public void StopServer()
        {
            // 가차없이 서버 소켓을 닫습니다.
            m_ServerSocket.Close();
        }

        public void SendMessage(String message)
        {
            // 추가 정보를 넘기기 위한 변수 선언
            // 크기를 설정하는게 의미가 없습니다.
            // 왜냐하면 바로 밑의 코드에서 문자열을 유니코드 형으로 변환한 바이트 배열을 반환하기 때문에
            // 최소한의 크기르 배열을 초기화합니다.
            AsyncObject ao = new AsyncObject(1);
            string emsg = string.Empty;

            // 문자열을 바이트 배열으로 변환
            ao.Buffer = Encoding.ASCII.GetBytes(message);

            // 사용된 소켓을 저장
            //ao.WorkingSocket = m_ServerSocket;
            ao.WorkingSocket = m_ConnectedClient;

            // 전송 시작!
            try
            {
                m_ConnectedClient.BeginSend(ao.Buffer, 0, ao.Buffer.Length, SocketFlags.None, m_fnSendHandler, ao);
            }
            catch (Exception ex)
            {
                emsg = "Error on sending. " + ex.Message;
                Console.WriteLine(emsg);
                objLogger.LogWrite(emsg);
                objFrame.StopCatpure();
                m_ServerSocket.Close();
                this.StartServer(2345);
            }
        }

        private void handleClientConnectionRequest(IAsyncResult ar)
        {
            Socket sockClient;
            try
            {
                // 클라이언트의 연결 요청을 수락합니다.
                sockClient = m_ServerSocket.EndAccept(ar);
                Console.WriteLine("ES Socket connected.");
                objLogger.LogWrite("ES Socket connected");
                mIsSocketConnected = true;

            }
            catch (Exception ex)
            {
                Console.WriteLine("Connection failed. msg: {0}", ex.Message);
                return;
            }

            // 4096 바이트의 크기를 갖는 바이트 배열을 가진 AsyncObject 클래스 생성
            AsyncObject ao = new AsyncObject(4096);

            // 작업 중인 소켓을 저장하기 위해 sockClient 할당
            ao.WorkingSocket = sockClient;

            // 클라이언트 소켓 저장
            m_ConnectedClient = sockClient;

            try
            {
                // 비동기적으로 들어오는 자료를 수신하기 위해 BeginReceive 메서드 사용!
                sockClient.BeginReceive(ao.Buffer, 0, ao.Buffer.Length, SocketFlags.None, m_fnReceiveHandler, ao);
            }
            catch (Exception ex)
            {
                // 예외가 발생하면 예외 정보 출력 후 함수를 종료한다.
                Console.WriteLine("자료 수신 대기 도중 오류 발생! 메세지: {0}", ex.Message);
                return;
            }
        }

        private void handleDataReceive(IAsyncResult ar)
        {

            // 넘겨진 추가 정보를 가져옵니다.
            // AsyncState 속성의 자료형은 Object 형식이기 때문에 형 변환이 필요합니다~!
            AsyncObject ao = (AsyncObject)ar.AsyncState;

            // 받은 바이트 수 저장할 변수 선언
            Int32 recvBytes;

            try
            {
                // 자료를 수신하고, 수신받은 바이트를 가져옵니다.
                recvBytes = ao.WorkingSocket.EndReceive(ar);
            }
            catch
            {
                // 예외가 발생하면 함수 종료!
                return;
            }

            // 수신받은 자료의 크기가 1 이상일 때에만 자료 처리
            if (recvBytes > 0)
            {
                // 공백 문자들이 많이 발생할 수 있으므로, 받은 바이트 수 만큼 배열을 선언하고 복사한다.
                Byte[] msgByte = new Byte[recvBytes];
                //Console.WriteLine(recvBytes);
                Array.Copy(ao.Buffer, msgByte, recvBytes);

                // 받은 메세지를 출력
                //Console.WriteLine("메세지 받음: {0}", Encoding.ASCII.GetString(msgByte));
                MessageParsing(msgByte);
            }

            try
            {
                // 비동기적으로 들어오는 자료를 수신하기 위해 BeginReceive 메서드 사용!
                ao.WorkingSocket.BeginReceive(ao.Buffer, 0, ao.Buffer.Length, SocketFlags.None, m_fnReceiveHandler, ao);
            }
            catch (Exception ex)
            {
                // 예외가 발생하면 예외 정보 출력 후 함수를 종료한다.
                Console.WriteLine("자료 수신 대기 도중 오류 발생! 메세지: {0}", ex.Message);
                return;
            }
        }
        private void handleDataSend(IAsyncResult ar)
        {

            // 넘겨진 추가 정보를 가져옵니다.
            AsyncObject ao = (AsyncObject)ar.AsyncState;

            // 보낸 바이트 수를 저장할 변수 선언
            Int32 sentBytes;

            try
            {
                // 자료를 전송하고, 전송한 바이트를 가져옵니다.
                sentBytes = ao.WorkingSocket.EndSend(ar);
            }
            catch (Exception ex)
            {
                // 예외가 발생하면 예외 정보 출력 후 함수를 종료한다.
                Console.WriteLine("Error on sending. err: {0}", ex.Message);
                return;
            }

            if (sentBytes > 0)
            {
                // 여기도 마찬가지로 보낸 바이트 수 만큼 배열 선언 후 복사한다.
                Byte[] msgByte = new Byte[sentBytes];
                Array.Copy(ao.Buffer, msgByte, sentBytes);

                //Console.WriteLine("MSG sent: {0}", Encoding.ASCII.GetString(msgByte));
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
                    case CMD_ES_START_REQ: // '내시경 영상 전송 시작 요청(저장경로 설정)'
                        Console.WriteLine("<< CMD_ES_START_REQ");
                        objLogger.LogWrite("[RX]CMD_ES_START_REQ");
                        objFrame.SetCaptureMode(FrameCapture.SCAN_ES);
                        ParsingESSaveDir(ref msg, dataLen);
                        objFrame.StartCatpure();
                        break;

                    case CMD_ES_STOP_REQ: // '내시경 영상 전송 중지 요청'
                        Console.WriteLine("<< CMD_ES_STOP_REQ");
                        objLogger.LogWrite("[RX]CMD_ES_STOP_REQ");
                        objFrame.StopCatpure();
                        break;
                }
            }
        }

        private int GetDataLength(ref byte[] msg)
        {
            return ((msg[3] - 0x30) * 1000) + ((msg[4] - 0x30) * 100) + ((msg[5] - 0x30) * 10) + ((msg[6] - 0x30));
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

        // Response to request message for the US scan
        public void ResponseESScan(string imgName)
        {
            int dataLen = imgName.Length;
            byte[] resmsg = new byte[(dataLen + FIXEDLen)];
            int msgIdx = 0;

            resmsg[msgIdx] = STX;
            resmsg[++msgIdx] = DIR_RES;
            resmsg[++msgIdx] = CMD_ES_RES;
            AddDataLen(ref resmsg, dataLen, ref msgIdx);
            AddFileName(ref resmsg, ref msgIdx, imgName);
            resmsg[msgIdx] = ETX;
            SendMessage(Encoding.Default.GetString(resmsg));
            objLogger.LogWrite("[TX]CMD_ES_SCAN_RES|" + Encoding.ASCII.GetString(resmsg));
            Console.WriteLine(">> CMD_ES_SCAN_RES|"+ imgName);
        }

        // Add data length field to message buffer
        private void AddDataLen(ref byte[] msgBuf, int dataLen, ref int msgIdx)
        {
            byte[] lenArr = new byte[4];
            lenArr = Encoding.ASCII.GetBytes(dataLen.ToString("D4"));
            Buffer.BlockCopy(lenArr, 0, msgBuf, ++msgIdx, 4);
            msgIdx += 4;
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
