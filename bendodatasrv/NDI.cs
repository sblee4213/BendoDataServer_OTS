using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading.Tasks;

namespace bendodatasrv
{
    /// <summary>
    /// 코드 사용 순서
    /// 
    /// 연결, 트래킹 시작
    /// Probe();
    /// Open(port, false);
    /// var tool1FilePath = @"C:/파일경로/8700449.rom";
    /// var tool2FilePath = @"C:/파일경로/8700339.rom";
    /// var tool1 = LoadToolFromFile(0, tool1FilePath);
    /// var tool2 = LoadToolFromFile(1, tool2FilePath);
    /// var tracking = StartTracking(true);
    /// 
    /// 데이터 수집
    /// UpdateTransforms();
    /// var matrix1 = GetMatrixTransform(0);
    /// var matrix2 = GetMatrixTransform(1);
    /// </summary>
    class NDI
    {
        public struct QuaternionTransformStruct
        {
            public double x;
            public double y;
            public double z;
            public double q0;
            public double q1;
            public double q2;
            public double q3;
            public double rmsError;
            public int frameNumber;
            public int status;
        };

        public struct MatrixTransformStruct
        {
            public double m11;
            public double m12;
            public double m13;
            public double m14;
            public double m21;
            public double m22;
            public double m23;
            public double m24;
            public double m31;
            public double m32;
            public double m33;
            public double m34;
            public double m41;
            public double m42;
            public double m43;
            public double m44;
            public double rmsError;
            public int frameNumber;
            public int status;
        };

        public struct ToolStatusStruct
        {
            public int portNumber;
            public bool bToolInPort;
            public bool bGPIO1;
            public bool bGPIO2;
            public bool bGPIO3;
            public bool bInitialized;
            public bool bEnabled;
            public bool bOutOfVolume;
            public bool bPartiallyOutOfVolume;
            public bool bDisturbanceDet;
            public bool bSignalTooSmall;
            public bool bSignalTooBig;
            public bool bProcessingException;
            public bool bHardwareFailure;
        };

        public struct DeviceStatusStruct
        {
            public bool bCommunicationSyncError;
            public bool bTooMuchInterference;
            public bool bSystemCRCError;
            public bool bRecoverableException;
            public bool bHardwareFailure;
            public bool bHardwareChange;
            public bool bPortOccupied;
            public bool bPortUnoccupied;
            public bool bDiagnosticsPending;
            public bool bTemperatureOutOfRange;
        };

        #region imports

        [DllImport("NDIAPI.DLL", CallingConvention = CallingConvention.Cdecl, EntryPoint = "GetConnectedDevicePortName")]
        private static extern IntPtr ndiapiGetConnectedDevicePortName();
        public static string GetConnectedDevicePortName()
        {
            return Marshal.PtrToStringAnsi(ndiapiGetConnectedDevicePortName());
        }

        [DllImport("NDIAPI.DLL", CallingConvention = CallingConvention.Cdecl, EntryPoint = "GetConnectedDeviceName")]
        private static extern IntPtr ndiapiGetConnectedDeviceName();
        public static string GetConnectedDeviceName()
        {
            return Marshal.PtrToStringAnsi(ndiapiGetConnectedDeviceName());
        }

        [DllImport("NDIAPI.DLL", CallingConvention = CallingConvention.Cdecl, EntryPoint = "GetConnectedDeviceVersion")]
        private static extern IntPtr ndiapiGetConnectedDeviceVersion();
        public static string GetConnectedDeviceVersion()
        {
            return Marshal.PtrToStringAnsi(ndiapiGetConnectedDeviceVersion());
        }

        [DllImport("NDIAPI.DLL", CallingConvention = CallingConvention.Cdecl, EntryPoint = "GetConnectedDeviceSerialNumber")]
        private static extern IntPtr ndiapiGetConnectedDeviceSerialNumber();
        public static string GetConnectedDeviceSerialNumber()
        {
            return Marshal.PtrToStringAnsi(ndiapiGetConnectedDeviceSerialNumber());
        }

        [DllImport("NDIAPI.DLL", CallingConvention = CallingConvention.Cdecl)]
        public static extern bool GetDeviceTracking();

        [DllImport("NDIAPI.DLL", CallingConvention = CallingConvention.Cdecl)]
        public static extern int Probe();

        [DllImport("NDIAPI.DLL", CallingConvention = CallingConvention.Cdecl)]
        public static extern int Open(string portName, bool enableCommandLogging);

        [DllImport("NDIAPI.DLL", CallingConvention = CallingConvention.Cdecl)]
        public static extern bool LoadToolFromFile(uint toolNumber, [MarshalAs(UnmanagedType.LPStr)] String filepath);

        [DllImport("NDIAPI.DLL", CallingConvention = CallingConvention.Cdecl)]
        public static extern bool StartTracking(bool resetFrameCount);

        [DllImport("NDIAPI.DLL", CallingConvention = CallingConvention.Cdecl)]
        public static extern bool UpdateTransforms();

        [DllImport("NDIAPI.DLL", CallingConvention = CallingConvention.Cdecl, EntryPoint = "GetMatrixTransform")]
        private static extern IntPtr ndiapiGetMatrixTransform(uint toolNumber);
        public static MatrixTransformStruct GetMatrixTransform(uint toolNumber)
        {
            return (MatrixTransformStruct)Marshal.PtrToStructure(ndiapiGetMatrixTransform(toolNumber), typeof(MatrixTransformStruct));
        }

        [DllImport("NDIAPI.DLL", CallingConvention = CallingConvention.Cdecl, EntryPoint = "GetQuaternionTransform")]
        private static extern IntPtr ndiapiGetQuaternionTransform(uint toolNumber);
        public static QuaternionTransformStruct GetQuaternionTransform(uint toolNumber)
        {
            return (QuaternionTransformStruct)Marshal.PtrToStructure(ndiapiGetQuaternionTransform(toolNumber), typeof(QuaternionTransformStruct));
        }

        [DllImport("NDIAPI.DLL", CallingConvention = CallingConvention.Cdecl, EntryPoint = "GetToolStatus")]
        private static extern IntPtr ndiapiGetToolStatus(uint toolNumber);
        public static ToolStatusStruct GetToolStatus(uint toolNumber)
        {
            return (ToolStatusStruct)Marshal.PtrToStructure(ndiapiGetToolStatus(toolNumber), typeof(ToolStatusStruct));
        }

        [DllImport("NDIAPI.DLL", CallingConvention = CallingConvention.Cdecl, EntryPoint = "GetDeviceStatus")]
        private static extern IntPtr ndiapiGetDeviceStatus();
        public static DeviceStatusStruct GetDeviceStatus()
        {
            return (DeviceStatusStruct)Marshal.PtrToStructure(ndiapiGetDeviceStatus(), typeof(DeviceStatusStruct));
        }

        [DllImport("NDIAPI.DLL", CallingConvention = CallingConvention.Cdecl)]
        public static extern bool StopTracking();

        [DllImport("NDIAPI.DLL", CallingConvention = CallingConvention.Cdecl)]
        public static extern bool RemoveTool(uint toolNumber);

        #endregion
    }
}
