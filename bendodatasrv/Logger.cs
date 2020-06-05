using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.IO;

namespace bendodatasrv
{
    public sealed class Logger
    {
        private static readonly Lazy<Logger> lazy = new Lazy<Logger>(() => new Logger());

        public static Logger Instance { get { return lazy.Value; } }
        private StreamWriter gLogFile;
        private StringBuilder gLogMsg;

        private Logger()
        {
            DateTime now;
            gLogMsg = new StringBuilder();
            string fName;
            now = DateTime.Now;
            fName = "LOG_" + now.Year.ToString("D4") + now.Month.ToString("D2") + now.Day.ToString("D2") + "_" + now.Hour.ToString("D2") + now.Minute.ToString("D2") + now.Second.ToString("D2") + ".txt";
            gLogFile = new StreamWriter(fName);
        }

        public void LogWrite(string msg) {
            gLogMsg.Clear();
            gLogMsg.Append(GetTimeString());
            gLogMsg.Append(msg);
            gLogFile.WriteLine(gLogMsg);
            gLogFile.Flush();
        }

        private string GetTimeString() {
            DateTime now = DateTime.Now;
            return now.Hour.ToString("D2") +":"+ now.Minute.ToString("D2") + ":" + now.Second.ToString("D2") + "." + now.Millisecond.ToString("D3")+"\t";
        }

        public void CloseLog() {
            gLogFile.Close();
        }
    }
}
