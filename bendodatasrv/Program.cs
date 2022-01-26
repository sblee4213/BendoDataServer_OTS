using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using OpenCvSharp;
using System.IO.Ports;
using System.IO;
using static bendodatasrv.NDI;

namespace bendodatasrv
{
    class Program
    {
        static void Main(string[] args)
        {
            PrintIntro();
            Logger objLogger = Logger.Instance;

#if OTS
            var portList = SerialPort.GetPortNames();

            var port = "COM8";

            if (Array.Exists(portList, element => element == port))
            {
                if (OpenOTS(port))
                {
                    if (StartTrackingMarker())
                    {
                        objLogger.LogWrite("OTS is connected.");
                        Console.WriteLine("OTS is connected.");
                    }
                    else
                    {
                        objLogger.LogWrite("OTS is not connected.");
                        Console.WriteLine("OTS is not connected.");
                    }
                }
                else
                {
                    objLogger.LogWrite("OTS is not connected.");
                    Console.WriteLine("OTS is not connected.");
                } 
            }
            else
            {
                objLogger.LogWrite("OTS is not connected.");
                Console.WriteLine("OTS is not connected.");
            }
#endif
            CommPositionSensor objPos = CommPositionSensor.Instance;

            USSocketServer objUSSocketServer = USSocketServer.Instance;
            objLogger.LogWrite("US socket server is started.");

            ESSocketServer objESSocketServer = ESSocketServer.Instance;
            objLogger.LogWrite("ES socket server is started.");

            objUSSocketServer.StartServer(1234);
            objESSocketServer.StartServer(2345);

            /*if (objPos.SetPort("COM8", 9600) == 1) {
                objPos.CounterInit();
                objPos.StartThread();
            }*/
            

            while (true)
            {
                string msg;
                Console.Write("");
                msg = Console.ReadLine().Trim();

                // 입력받은 문자열이 null 인 경우, 다시 반복문의 처음으로 돌아간다.
                if (string.IsNullOrEmpty(msg))
                    continue;

                // 입력받은 문자열이 X 인 경우, 프로그램을 종료한다.
                if (msg.Equals("X"))
                {
                    if (objUSSocketServer.mIsSocketConnected)
                    {
                        objUSSocketServer.StopServer();
                    }
                    objLogger.LogWrite("US socket server is stoped.");

                    if (objESSocketServer.mIsSocketConnected)
                    {
                        objESSocketServer.StopServer();
                    }
                    objLogger.LogWrite("ES socket server is stoped.");
                    objLogger.CloseLog();
                    return;
                }
            }
        }

        private static bool OpenOTS(string port)
        {
            try
            {
                Probe();
                Open(port, false);
                return true;
            }
            catch (Exception)
            {
                System.Windows.Forms.MessageBox.Show("NDI is not working.");
            }
            return false;
        }

        private static bool StartTrackingMarker()
        {
            var tool1FilePath = AppDomain.CurrentDomain.BaseDirectory + "8700449.rom";
            var tool2FilePath = AppDomain.CurrentDomain.BaseDirectory + "8700339.rom";

            var tool1 = LoadToolFromFile(0, tool1FilePath);
            var tool2 = LoadToolFromFile(1, tool2FilePath);
            var tracking = StartTracking(true);

            if (!tool1 || !tool2 || !tracking)
            {
                System.Windows.Forms.MessageBox.Show("Tool1 : " + tool1.ToString() + "\nTool2 : " + tool2.ToString() + "\nTracking : " + tracking.ToString());
                return false;
            }
            else
            {
                return true;
            }
        }

        public static void PrintIntro()
        {
            Console.WriteLine("┏━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┓");
            Console.WriteLine("┃          BENDO DATA SERVER          ┃");
            Console.WriteLine("┃                                     ┃");
            Console.WriteLine("┃              KNU MDRIP              ┃");
            Console.WriteLine("┃              2020.08.07             ┃");
            Console.WriteLine("┗━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┛");
        }
    }
}
