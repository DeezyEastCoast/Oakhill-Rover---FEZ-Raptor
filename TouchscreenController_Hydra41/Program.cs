using System;
using System.Collections;
using System.Threading;
using Microsoft.SPOT;
using Microsoft.SPOT.Presentation;
using Microsoft.SPOT.Presentation.Controls;
using Microsoft.SPOT.Presentation.Media;
using Microsoft.SPOT.Touch;

//using Microsoft.SPOT.Hardware;
//using GHIElectronics.NETMF.Hardware;

using Microsoft.SPOT.IO;
//using GHIElectronics.NETMF.IO;
using GHI.Glide;
using GHI.Glide.Display;
using GHI.Glide.UI;
using GW = GHI.Glide.Display;

using System.IO.Ports;
using System.Text;
using System.IO;

using Gadgeteer.Networking;
using GT = Gadgeteer;
using GTM = Gadgeteer.Modules;
using Gadgeteer.Modules.GHIElectronics;

using Power_Uart_Mux;
using Adafruit10DOFIMU;
using Skewworks.Drivers.GPS;

namespace Oakhill_Rover
{
    public partial class Program
    {
        static string SD_ROOT_DIRECTORY;
        //static FileStream FILE_HANDLE;
        static Load roverLoadBrd;

        enum DRIVE_MODE { MANUAL = 0, ASSIST, AUTO, COMPASS };
        static DRIVE_MODE DRIVE_MODE_ROVER = DRIVE_MODE.MANUAL;

        const int ROVER_TIMER_INTERVAL = 50;
        static int MANUAL_MODE_TICK_INTERVAL = 20; // time interval in units of ROVER_TIMER_INTERVAL to run this mode --> 20 * 50 mSec = 500 mSec interval
        static int AUTO_MODE_TICK_INTERVAL = 20; // time interval in units of ROVER_TIMER_INTERVAL to run this mode --> 20 * 50 mSec = 1000 mSec interval
        static int COMPASS_MODE_TICK_INTERVAL = 20; // time interval in units of ROVER_TIMER_INTERVAL to run this mode --> 20 * 50 mSec = 1000 mSec interval

        static GT.Timer ROVER_TIMER;
        static GT.Timer GroundEfxTimer; //Timer to run ground efx sho

        static bool DEBUG_LED_STATE = true;
        //static bool READCOMPLETE_IMU = false;
        static uint ROVER_LOOP_CNT = 0;
        static bool BNO_SETTINGS_SAVED = false;
        static string BNO_SETTINGS_FILE = "roverBNO.txt";

        static int MA_RANGE_INDEX = 0;
        static int MAX_RANGE = 100;  //max range in inches to care about
        static int MA_RANGE_PERIOD = 5;
        static double[] MA_RANGE_ARRAY = new double[MA_RANGE_PERIOD];
        static int SONAR_DISTANCE = 0;

        static int SIGNAL_QUAL_INDEX = 0;
        static int SIGNAL_QUAL_PERIOD = 20;
        static double[] SIGNAL_QUALITY_ARRAY = new double[SIGNAL_QUAL_PERIOD];
        static double SIGNAL_QUALITY = 0;

        static int picCounter = 0;
        static string ERR_STATE = "NONE";
        //static bool roverHeadLights = false;
        //static bool roverTakePicture = false;

        static SDCard roverSDCARD;

        static GT.SocketInterfaces.AnalogInput roverDistanceSensor;
        static GT.SocketInterfaces.AnalogInput roverBatteryVoltage;
        static GT.SocketInterfaces.AnalogInput roverBatteryCurrent;

        static SerialPort motor_steering_ComPort;
        static OHS_UartMux muxComPort;
        //static EMIC2 roverVoice;
        static MTK3339 roverGPS;

        //static DateTime start_time;
        //static DateTime end_time;
        //static TimeSpan ts;

        #region AUTONOMOUS NAVIGATION

        //Compass Navigation
        static int TARGET_HEADING;  // where we want to go to reach current waypoint
        static double CURRENT_HEADING; // where we are actually facing now
        static int HEADING_ERROR;  // signed (+/-) difference between targetHeading and currentHeading
        static readonly int HEADING_TOLERANCE = 10;  // tolerance +/- (in degrees) within which we don't attempt to turn to intercept targetHeading

        //GPS Navigation
        static WAYPOINT homeWaypoint;
        static WAYPOINT currentWaypoint; //realtime position of rover
        static WAYPOINT targetWaypoint;  //position of target waypoint
        static int distanceToTarget;  // current distance to target (current waypoint)
        static int originalDistanceToTarget;  // distance to original waypoint when we started navigating to it

        //GPS Time 
        struct GPSTIME
        {
            public GPSTIME(int h, int m, int s)
            {
                hour = h;
                min = m;
                sec = s;
            }

            public int hour;
            public int min;
            public int sec;
        }

        struct GPSDATE
        {
            public int month;
            public int day;
            public int year;

            public GPSDATE(int m, int d, int y)
            {
                month = m;
                day = d;
                year = y;
            }
        }

        //Waypoints
        struct WAYPOINT
        {
            ////gps coords must be in decimal degrees
            //WAYPOINT(string inLat, string inLon, sbyte inNum = -1)
            //{
            //    lat = inLat;
            //    lon = inLon;
            //    number = inNum;
            //}

            //gps coords must be in decimal degrees
            public WAYPOINT(double inLat, double inLon, sbyte inNum = -1)
            {
                dMaplat = inLat;
                dMaplon = inLon;
                number = inNum;
            }

            //public string lat, lon;
            double dMaplat, dMaplon;

            public double dMapLatitude
            {
                get
                {
                    return dMaplat;
                    //return Convert.ToDouble(lat);
                }
                set
                {
                    dMaplat = value;
                }
            }

            public double dMapLongitude
            {
                get
                {
                    return dMaplon;
                    //return Convert.ToDouble(lon);
                }
                set
                {
                    dMaplon = value;
                }
            }

            public int number;
        }

        static readonly int WAYPOINT_DISTANCE_TOLERANCE = 5; // tolerance in meters to waypoint; once within this tolerance, will advance to the next waypoint
        static readonly int NUMBER_WAYPOINTS = 4; // enter the numebr of way points here (will run from 0 to (n-1))
        static int targetWaypointNum = 0;  // current waypoint number; will run from 0 to (NUMBER_WAYPOINTS -1); start at -1 and gets initialized during setup()
        static WAYPOINT[] targetWaypointList = new WAYPOINT[NUMBER_WAYPOINTS];
        static WAYPOINT[] gsTargetWaypointList = new WAYPOINT[NUMBER_WAYPOINTS];
        static GPSTIME curGPSTIME = new GPSTIME(0, 0, 0);
        static GPSDATE curGPSDATE = new GPSDATE(0, 0, 0);

        //Steering/Turning
        //enum TURN { LEFT = 200, RIGHT = 56, STRAIGHT = 128 };
        enum TURN { RIGHT = 164, LEFT = 92, STRAIGHT = 128 };
        static TURN turnDirection_DESIRED = TURN.STRAIGHT;
        static TURN turnDirection_CURRENT = TURN.STRAIGHT;

        //Object avoidance distances (inches)
        static byte SAFE_DISTANCE = 60;
        static byte TURN_DISTANCE = 36;
        static byte STOP_DISTANCE = 12;

        //Speeds 
        static long MOTORTIMEOUT_CNT = 0;
        const byte STOP_SPEED = 135;
        const byte FAST_SPEED = 160;
        const byte NORMAL_SPEED = 155;
        const byte TURN_SPEED = 150;
        const byte SLOW_SPEED = 150;
        const byte REVERSE_SPEED = 0;
        static byte CURRENT_NAV_SPEED = STOP_SPEED;
        static byte NEW_NAV_SPEED = STOP_SPEED;

        #endregion

        #region COMMUNICATION ARRAYS

        static string roverData; //string sent from rover
        static string ps2DataLine;  //string received from controller
        static byte STATUS_DEVICES = 0; //byte to hold status of SDCARD, ALCAM, VOICE, etc.

        #endregion

        #region LAIRD WIRELESS XCVR

        static SerialPort lairdComPort;
        static SerialBuffer lairdWirelessBuffer;
        Microsoft.SPOT.Hardware.Cpu.Pin lairdResetPin;
        Microsoft.SPOT.Hardware.OutputPort lairdReset;
        Microsoft.SPOT.Hardware.Cpu.Pin lairdCtsPin;
        Microsoft.SPOT.Hardware.OutputPort lairdCts;
        //Microsoft.SPOT.Hardware.Cpu.Pin lairdRtsPin;
        //Microsoft.SPOT.Hardware.InputPort lairdRts;

        #endregion

        #region Adafruit 10DOF

        //static Unified IMU_Adafruit = Unified.Instance;

        //bool IMU_PRESS_ALIVE = IMU_Adafruit.Bmp180.IsAlive();
        //bool IMU_MAG_ALIVE = IMU_Adafruit.Magnetometer.IsAlive();
        //bool IMU_ACCEL_ALIVE = IMU_Adafruit.Accelerometer.IsAlive();
        //bool IMU_GYRO_ALIVE = IMU_Adafruit.Gyroscope.IsAlive();


        static double DECLINATION_ANGLE = 0.0;
        static BNO055 IMU_BNO = new BNO055();

        byte[] sysStatus = new byte[] { 0, 0, 0 };
        static BNO055.Vector ATTITUDE_VECTOR;

        #endregion

        #region ALCAM

        //UART SETUP
        //Microsoft.SPOT.Hardware.Cpu.Pin GPIO_Pin_Port6_Pin3; //pin 3
        //Microsoft.SPOT.Hardware.Cpu.Pin RTS_Pin_Port6;  //pin 6
        //Microsoft.SPOT.Hardware.Cpu.Pin CTS_Pin_Port6;  //pin 7
        //string comPortNameAlcam;

        //TX is pin 4 on mainboard gadgeteer port
        //RX is pin 5 on mainboard gadgeteer port
        //RTS is pin 6 on mainboard gadgeteer port
        //CTS is pin 7 on mainboard gadgeteer port
        //use pin 3 as reset

        //**********SPI PORT SETUP*********//
        //MOSI is pin 7 on mainboard gadgeteer port
        //MISO is pin 8 on mainboard gadgeteer port
        //SCKL is pin 9 on mainboard gadgeteer port
        //use pin 3 as alcam reset on mainboard gadgeteer port
        //use pin 5 as SSEL
        //use pin 6 as BUSYpin

        //Microsoft.SPOT.Hardware.SPI.SPI_module SPI1 = Microsoft.SPOT.Hardware.SPI.SPI_module.SPI1;
        //Microsoft.SPOT.Hardware.Cpu.Pin _BUSYpin;
        //Microsoft.SPOT.Hardware.Cpu.Pin _SSEL;
        //private static Zigbee2B zigbeeWireless = new Zigbee2B(SPI1, _chipSelectPin, _chipEnablePin, _interruptPin);

        #endregion

        #region JPEG CAMERA

        //static LinkspriteCamera LINKSPRITE_CAMERA;
        //static readonly string JPEG_FILENAME = "roverPic";
        //static uint JPEG_FILENUMBER = 0;
        //static string[] sdCardJpgFileNames;

        //static string linkSpriteCamComPortName;
        //static SerialPort linkSpriteCamComPort;

        #endregion

        #region ECO2 VIDEO CAMERA

        //static bool ECOCAMBOOL = false;

        //enum videoCamMode: byte
        //{
        //    Unknown,
        //    Stopped,
        //    SinglePhoto,
        //    SerialPhoto,
        //    Video,
        //    Flip180,
        //    Flip0,
        //}

        //static GT.Socket.SocketInterfaces.AnalogInput videoCamRedLed;
        //static GT.Socket.SocketInterfaces.AnalogInput videoCamGreenLed;

        //static Microsoft.SPOT.Hardware.Cpu.Pin vidCamSelectBtnPin;
        //static Microsoft.SPOT.Hardware.TristatePort vidCamSelectBtn;
        //static videoCamMode EcoCamMode = videoCamMode.Unknown;

        #endregion


        // This method is run when the mainboard is powered up or reset.   
        void ProgramStarted()
        {
            // Use Debug.Print to show messages in Visual Studio's "Output" window during debugging.
            Debug.Print("Program Started");

            #region GENERAL SETUP
            //Attelboro, MA: Capron park test mission  
            //targetWaypointList[0].dMapLatitude = 41.9378456849981376;
            //targetWaypointList[0].dMapLongitude = -71.2962967157363892;
            //targetWaypointList[1].dMapLatitude = 41.9380132828811867;
            //targetWaypointList[1].dMapLongitude = -71.2957066297531128;
            //targetWaypointList[2].dMapLatitude = 41.937618230023773;
            //targetWaypointList[2].dMapLongitude = -71.295427680015564;
            //targetWaypointList[3].dMapLatitude = 41.9374466406466198;
            //targetWaypointList[3].dMapLongitude = -71.2960553169250488;

            ////Charlotte, NC: Veterans Memorial Park Test Mission
            //targetWaypointList[0].dMapLatitude = 35.217762;
            //targetWaypointList[0].dMapLongitude = -80.802774;
            //targetWaypointList[1].dMapLatitude = 35.218061; 
            //targetWaypointList[1].dMapLongitude = -80.802755;
            //targetWaypointList[2].dMapLatitude = 35.218058;
            //targetWaypointList[2].dMapLongitude = -80.802328;
            //targetWaypointList[3].dMapLatitude = 35.217770;
            //targetWaypointList[3].dMapLongitude = -80.802347;

            //Charlotte, NC: Romany Rd Park Test Mission
            //targetWaypointList[0].dMapLatitude = 35.206633;
            //targetWaypointList[0].dMapLongitude = -80.844797;
            //targetWaypointList[1].dMapLatitude = 35.206347;
            //targetWaypointList[1].dMapLongitude = -80.844144;
            //targetWaypointList[2].dMapLatitude = 35.206080;
            //targetWaypointList[2].dMapLongitude = -80.844510;
            //targetWaypointList[3].dMapLatitude = 35.206483;
            //targetWaypointList[3].dMapLongitude = -80.845065;

            //Charlotte, NC: harlotteville Rd Park Test Mission
            targetWaypointList[0].dMapLatitude = 35.2167000;
            targetWaypointList[0].dMapLongitude = -80.8270400;
            targetWaypointList[1].dMapLatitude = 35.2169233;
            targetWaypointList[1].dMapLongitude = -80.8266567;
            targetWaypointList[2].dMapLatitude = 35.2171633;
            targetWaypointList[2].dMapLongitude = -80.8267283;
            targetWaypointList[3].dMapLatitude = 35.2169683;
            targetWaypointList[3].dMapLongitude = -80.8271650;

            //pin 3 port 13 is ultrasonic distance sensor Vcc/512 per inch --> 6.4mV per inch
            //pin 6 port 13 is ultrasonic distance sensor RX pin, High = enabled, Low = Disabled
            //pin 4 port 13 is battery voltage monitor  63.69 mV/Volt
            //pin 5 port 13 is battery current monitor  36.60 mV/Volt
            //pin 8 port 13 is unconnected but wired to 2X3 connector
            roverBatteryCurrent = extender_Battery_Input.CreateAnalogInput(GT.Socket.Pin.Five);
            roverBatteryVoltage = extender_Battery_Input.CreateAnalogInput(GT.Socket.Pin.Four);
            roverDistanceSensor = extender_Battery_Input.CreateAnalogInput(GT.Socket.Pin.Three);

            roverLoadBrd = loadBrd;
            Program.roverSDCARD = sdCard;

            //try
            //{
            //    //roverSDCARD.Mount();
            //    //SD_ROOT_DIRECTORY = VolumeInfo.GetVolumes()[0].RootDirectory;
            //    //Thread.Sleep(500);
            //    //WriteFileLines("PoeticJustice.txt", new string[] { "Police", "Aparteid", "We gon be alright" });
            //    STATUS_DEVICES += 1;
            //}
            //catch (Exception)
            //{
            //    picCounter = -1;  
            //}

            //Block to test using CTS as digital output
            //Microsoft.SPOT.Hardware.Cpu.Pin testCTSPin;
            //testCTSPin = GT.Socket.GetSocket(6, true, null, null).CpuPins[7];
            //Microsoft.SPOT.Hardware.OutputPort testCTS;
            //testCTS = new Microsoft.SPOT.Hardware.OutputPort(testCTSPin, true);
            //testCTS.Write(false);

            #endregion

            #region LAIRD WIRELESS XCVR Setup

            //lairdWirelss.Configure(9600, GT.Interfaces.Serial.SerialParity.None, GT.Interfaces.Serial.SerialStopBits.One, 7);
            string lairdComName = GT.Socket.GetSocket(11, true, null, null).SerialPortName;
            lairdComPort = new SerialPort(lairdComName, 115200);
            lairdCtsPin = GT.Socket.GetSocket(11, true, null, null).CpuPins[6];
            //lairdRtsPin = GT.Socket.GetSocket(11, true, null, null).CpuPins[7];
            lairdResetPin = GT.Socket.GetSocket(11, true, null, null).CpuPins[3];

            lairdReset = new Microsoft.SPOT.Hardware.OutputPort(lairdResetPin, true);
            lairdCts = new Microsoft.SPOT.Hardware.OutputPort(lairdCtsPin, false);
            //lairdRts = new Microsoft.SPOT.Hardware.InputPort(lairdRtsPin, false, Microsoft.SPOT.Hardware.Port.ResistorMode.PullUp);

            lairdWirelessBuffer = new SerialBuffer(72);

            lairdComPort.Open();

            #endregion

            #region MOTOR & SERVO COM PORT

            string steering_motor_ComName = GT.Socket.GetSocket(12, true, null, null).SerialPortName;
            motor_steering_ComPort = new SerialPort(steering_motor_ComName, 9600);
            motor_steering_ComPort.Open();

            #endregion

            #region Adafruit10DOF Setup

            //if (IMU_GYRO_ALIVE && IMU_MAG_ALIVE && IMU_PRESS_ALIVE && IMU_ACCEL_ALIVE)
            //{
            //    //IMU_Adafruit.ReadCompleted += (t) =>
            //    //{
            //    //    IMU_Adafruit.ContinuousRead = false;
            //    //    READCOMPLETE_IMU = true;
            //    //};

            //    IMU_Adafruit.ReadCompleted += (t) =>
            //        {
            //            //Debug.Print("Current Temperature: " + IMU_Adafruit.Bmp180.Temperature + " \u00B0C");
            //            Debug.Print("Current Temperature: " + ((IMU_Adafruit.Bmp180.Temperature * 1.8000) + 32) + " \u00B0F");
            //            Debug.Print("Accelerometer X: " + IMU_Adafruit.Accelerometer.X + " Y: " + IMU_Adafruit.Accelerometer.Y + " Z: " + IMU_Adafruit.Accelerometer.Z);
            //            Debug.Print("Magnetometer X: " + IMU_Adafruit.Magnetometer.X + " Y: " + IMU_Adafruit.Magnetometer.Y + " Z: " + IMU_Adafruit.Magnetometer.Z);
            //            Debug.Print("Gyroscope X: " + IMU_Adafruit.Gyroscope.X + " Y: " + IMU_Adafruit.Gyroscope.Y + " Z: " + IMU_Adafruit.Gyroscope.Z);
            //            Debug.Print("Heading: " + IMU_Adafruit.Heading);
            //        };

            //    IMU_Adafruit.ContinuousRead = true;
            //    IMU_Adafruit.BeginAsync();
            //}
            //else
            //{
            //    while (true)
            //    {
            //        if (!IMU_Adafruit.Accelerometer.IsAlive()) Debug.Print("ACCEL not working.\r\n");
            //        else Debug.Print("ACCEL OK.\r\n");
            //        if (!IMU_Adafruit.Gyroscope.IsAlive()) Debug.Print("GYRO not working.\r\n");
            //        else Debug.Print("GYRO OK.\r\n");
            //        if (!IMU_Adafruit.Magnetometer.IsAlive()) Debug.Print("MAG not working.\r\n");
            //        Debug.Print("MAG OK.\r\n");
            //        if (!IMU_Adafruit.Bmp180.IsAlive()) Debug.Print("PRESSURE not working.\r\n");
            //        Debug.Print("PRESS OK.\r\n");

            //        Mainboard.SetDebugLED(true);
            //        Thread.Sleep(1000);
            //        Mainboard.SetDebugLED(false);
            //        Thread.Sleep(1000);

            //    }
            //}

            bool BNO_IS_ALIVE = IMU_BNO.IsAlive();

            if (BNO_IS_ALIVE) //read saved calibration settings
            {
                if (!roverSDCARD.IsCardMounted)
                {
                    roverSDCARD.Mount();
                    Thread.Sleep(250);
                }

                if (roverSDCARD.IsCardMounted)
                    SD_ROOT_DIRECTORY = VolumeInfo.GetVolumes()[0].RootDirectory;

                if (File.Exists(SD_ROOT_DIRECTORY + @"\" + BNO_SETTINGS_FILE))
                {
                    string[] tempBNOSettings = ReadFileLines(BNO_SETTINGS_FILE, 11);

                    if (tempBNOSettings != null)
                    {
                        IMU_BNO.BNO_SENSOR_OFFSETS.accel_offset_x = Convert.ToUInt16(tempBNOSettings[0]);
                        IMU_BNO.BNO_SENSOR_OFFSETS.accel_offset_y = Convert.ToUInt16(tempBNOSettings[1]);
                        IMU_BNO.BNO_SENSOR_OFFSETS.accel_offset_z = Convert.ToUInt16(tempBNOSettings[2]);
                        IMU_BNO.BNO_SENSOR_OFFSETS.gyro_offset_x = Convert.ToUInt16(tempBNOSettings[3]);
                        IMU_BNO.BNO_SENSOR_OFFSETS.gyro_offset_y = Convert.ToUInt16(tempBNOSettings[4]);
                        IMU_BNO.BNO_SENSOR_OFFSETS.gyro_offset_z = Convert.ToUInt16(tempBNOSettings[5]);
                        IMU_BNO.BNO_SENSOR_OFFSETS.mag_offset_x = Convert.ToUInt16(tempBNOSettings[6]);
                        IMU_BNO.BNO_SENSOR_OFFSETS.mag_offset_y = Convert.ToUInt16(tempBNOSettings[7]);
                        IMU_BNO.BNO_SENSOR_OFFSETS.mag_offset_z = Convert.ToUInt16(tempBNOSettings[8]);
                        IMU_BNO.BNO_SENSOR_OFFSETS.accel_radius = Convert.ToUInt16(tempBNOSettings[9]);
                        IMU_BNO.BNO_SENSOR_OFFSETS.mag_radius = Convert.ToUInt16(tempBNOSettings[10]);

                        IMU_BNO.setSensorOffsets(IMU_BNO.BNO_SENSOR_OFFSETS);

                        Thread.Sleep(500);
                    }
                }

                roverSDCARD.Unmount();
            }

            //IMU_BNO.getSystemStatus(ref sysStatus[0], ref sysStatus[1], ref sysStatus[2]);
            //Debug.Print("SystemStatus: " + sysStatus[0].ToString() + " -- " + sysStatus[1].ToString() + " -- " + sysStatus[2].ToString() + "\n\r");
            //Thread.Sleep(2000);

            //while (!IMU_BNO.isFullyCalibrated())
            //{
            //    IMU_BNO.getSystemStatus(ref sysStatus[0], ref sysStatus[1], ref sysStatus[2]);
            //    Debug.Print("SystemStatus: " + sysStatus[0].ToString() + " -- " + sysStatus[1].ToString() + " -- " + sysStatus[2].ToString());

            //    Thread.Sleep(500);

            //    ATTITUDE_VECTOR = IMU_BNO.getVector(BNO055.adafruit_vector_type_t.VECTOR_EULER);
            //    Debug.Print("Euler: X:" + ATTITUDE_VECTOR.x + " Y:" + ATTITUDE_VECTOR.y + " Z:" + ATTITUDE_VECTOR.z);

            //    Thread.Sleep(500);

            //    Debug.Print("SYS Cal: " + IMU_BNO.Calibration_SYS);
            //    Debug.Print("Mag Cal: " + IMU_BNO.Calibration_MAG);
            //    Debug.Print("Accel Cal: " + IMU_BNO.Calibration_ACCEL);
            //    Debug.Print("Gyro Cal: " + IMU_BNO.Calibration_GYRO);

            //    Thread.Sleep(500);
            //}

            #endregion

            #region ALCAM STUFF
            /*
            CTS_Pin_Port6 = Gadgeteer.Socket.GetSocket(6, true, null, null).CpuPins[7];
            RTS_Pin_Port6 = Gadgeteer.Socket.GetSocket(6, true, null, null).CpuPins[6];
            GPIO_Pin_Port6 = Gadgeteer.Socket.GetSocket(6, true, null, null).CpuPins[3]; //used as reset pin

            var alcam = new ALCAM(ALCAM.CommunicationMode.Uart, GPIO_Pin_Port6);

            Thread.Sleep(1000);

            comPortNameAlcam = Gadgeteer.Socket.GetSocket(6, true, null, null).SerialPortName;
            alcam.SetInterface(new UartInterface(comPortNameAlcam, RTS_Pin_Port6, CTS_Pin_Port6));

            Thread.Sleep(1000);
            */

            //GPIO_Pin_Port6_Pin3 = Gadgeteer.Socket.GetSocket(3, true, null, null).CpuPins[3]; //used as reset pin
            //try
            //{
            //    var alcam = new ALCAM(ALCAM.CommunicationMode.Spi, GPIO_Pin_Port6_Pin3);

            //    Thread.Sleep(1000);

            //    _SSEL = Gadgeteer.Socket.GetSocket(3, true, null, null).CpuPins[5];
            //    _BUSYpin = Gadgeteer.Socket.GetSocket(3, true, null, null).CpuPins[6];
            //    alcam.SetInterface(new SpiInterface(SPI1, _SSEL, _BUSYpin));

            //    //If using I2C or SPI, make sure to read the banner.
            //    Debug.Print("Read banner:\n" + alcam.ReadBanner());
            //    Debug.Print("Get version:\n" + alcam.GetVersion());
            //}
            //catch
            //{
            //    Debug.Print("ALCAM camera not connected.\n");

            //}

            //Debug.Print("Change size:\n" + alcam.ChangeSize());
            //Debug.Print("Take picture to SD:\n" + alcam.TakePictureToSD());
            ////Debug.Print("Take picture to host:\n" + alcam.TakePictureToHost("Hello"));
            //Debug.Print("Take picture to host:\n" + alcam.TakePictureToScreen());
            //Debug.Print("Record movie:\n" + alcam.RecordMovie());
            //Debug.Print("Write file:\n" + alcam.WriteFile());
            //Debug.Print("Close file:\n" + alcam.CloseFile());
            //Debug.Print("Read file:\n" + alcam.ReadFile());

            //display_TE35.SimpleGraphics.DisplayImage(SD_ROOT_DIRECTORY + @"\Hello.jpg", Bitmap.BitmapImageType.Jpeg, 0, 0);
            //display_TE35.SimpleGraphics.DisplayImage(new Microsoft.SPOT.Bitmap(ALCAM.pictureData, Bitmap.BitmapImageType.Jpeg), 0, 0);

            #endregion

            #region ECO2 VIDEO CAMERA SETUP

            //vidCamSelectBtnPin = GT.Socket.GetSocket(13, true, null, null).CpuPins[3];
            //vidCamSelectBtn = new Microsoft.SPOT.Hardware.TristatePort(vidCamSelectBtnPin, false, false, Microsoft.SPOT.Hardware.Port.ResistorMode.Disabled);
            ////must leave vidCamSelectBtn initstate above as false so that when tristate pin is switched to output it is low.
            //MakePinInput(vidCamSelectBtn);

            //videoCamGreenLed = GT.Socket.GetSocket(13, true, null, null).AnalogInput4;
            //videoCamRedLed = GT.Socket.GetSocket(13, true, null, null).AnalogInput5;

            #endregion

            #region LINKSPRITE CAMWERA
            //camPortName = GT.Socket.GetSocket(6, false, null, null).SerialPortName;
            //camComPort = new SerialPort(camPortName, 38400);
            //LINKSPRITE_CAMERA = new LinkspriteCamera(camComPort);
            //LINKSPRITE_CAMERA = new LinkspriteCamera(multiComPort.MuxComPort);
            #endregion

            #region EMIC 2 VOICE

            //myVoice = new EMIC2(multiComPort.MuxComPort);

            ////set com port 0 for EMIC 2 at 9600
            ////set com port 1 for Linksprite Camera at 38400
            //multiComPort = new OHS_Power_Uart_Mux(4, 9600, 38400, 9600, 9600);        
            ////settings for EMIC, see if it affects other devices on mux port
            //multiComPort.MuxComPort.Close();
            //multiComPort.MuxComPort.WriteTimeout = 500;
            //multiComPort.MuxComPort.ReadTimeout = 500;
            //multiComPort.MuxComPort.Open();
            //multiComPort.writeMuxCom(new byte[] { 0x41, 0x42, 0x43, 0x0d, 0x0a }, 3); //test debug port 3

            /*
            //Talk to EMIC
            myVoice.changeVolume(13);
            myVoice.changeVolume(12);
            myVoice.changeSpeed(280);
            myVoice.Say("Hey Gus and everyone from Tiny C L R.  This is EMIC 2 coming right at yah from Fez Cerb er us. Adding EMIC 2 to a project is easy, all I need is a 5 volt power line, a ground connection, and a u art connection. Plug in headphones or a speaker and you're done. Even easier with a Gadgeteer adapter from Brett! Then, all you do is tell me what to say, and I say it. easy peas ee.");
            myVoice.Say("I have multiple voices too. This is Perfect Paul");
            myVoice.changeVoice(8);
            myVoice.Say("I'm Whispering Wendy");
            myVoice.changeVoice(0);
            myVoice.Say("And we're back to Paul");
            myVoice.Say("I can speak Spanish too");
            myVoice.changeLanguage(1);
            myVoice.changeSpeed(180);
            myVoice.Say("Hola a todo mi gente");
             */
            #endregion

            #region GPS

            muxComPort = new OHS_UartMux(4);
            Power_Uart_Mux.OHS_UartMux.myActiveComPort = 0;
            roverGPS = new MTK3339(Gadgeteer.Socket.GetSocket(4, true, null, null).SerialPortName, MTK3339.SubscriptionLevels.RMCGGA, MTK3339.UpdateRates.GPS_1HZ);
            //roverGPS = new MTK3339(Gadgeteer.Socket.GetSocket(4, true, null, null).SerialPortName, MTK3339.SubscriptionLevels.RMCGGA, MTK3339.UpdateRates.GPS_1HZ,true);
            //roverGPS.TimeChanged += gpsTimeChange;
            //roverGPS.DateChanged += gpsDateChange;
            //roverGPS.CoordinatesUpdated += GPS_CoordinatesUpdated;

            //void GPS_CoordinatesUpdated(MTK3339 sender)
            //{
            //    Debug.Print("Coordinates: " + roverGPS.Latitude + ", " + roverGPS.Longitude + " (" + roverGPS.MapLatitude + ", " + roverGPS.MapLongitude + ")");
            //}

            #endregion

            #region TIMERS

            ROVER_TIMER = new GT.Timer(ROVER_TIMER_INTERVAL);
            ROVER_TIMER.Tick += new GT.Timer.TickEventHandler(RoverTimer_Tick);
            ROVER_TIMER.Start();

            GroundEfxTimer = new GT.Timer(500);
            GroundEfxTimer.Tick += new GT.Timer.TickEventHandler(GroundEfxTimer_Tick);
            //GroundEfxTimer.Start();

            #endregion

            //display_T35.SimpleGraphics.DisplayImage(SD_ROOT_DIRECTORY + @"\roverPic17.jpg", Bitmap.BitmapImageType.Jpeg, 0, 0);
        }


        /// <summary>
        /// Switches between rover operational modes.
        /// </summary>
        static void switchMode(DRIVE_MODE newMode)
        {
            int i = 0;
            switch (newMode)
            {
                case DRIVE_MODE.MANUAL:

                    DRIVE_MODE_ROVER = newMode;
                    targetWaypointNum = 0;

                    break;

                case DRIVE_MODE.AUTO:

                    DRIVE_MODE_ROVER = newMode;

                    //if (ManualRoverTimer.IsRunning)
                    //    ManualRoverTimer.Stop();

                    if (!roverSDCARD.IsCardMounted)
                    {
                        roverSDCARD.Mount();
                        Thread.Sleep(250);
                    }

                    if (roverSDCARD.IsCardMounted)
                        SD_ROOT_DIRECTORY = VolumeInfo.GetVolumes()[0].RootDirectory;

                    break;

                case DRIVE_MODE.COMPASS:

                    DRIVE_MODE_ROVER = newMode;

                    Power_Uart_Mux.OHS_UartMux.myActiveComPort = 3;

                    break;

                case DRIVE_MODE.ASSIST:
                    break;

                default:
                    break;
            }

            if (newMode != DRIVE_MODE.AUTO)
            {
                if ((roverSDCARD.IsCardInserted) && (roverSDCARD.IsCardMounted))
                {
                    roverSDCARD.Unmount();
                    Thread.Sleep(250);
                }
            }

            if (newMode != DRIVE_MODE.COMPASS)
            {
                Power_Uart_Mux.OHS_UartMux.myActiveComPort = 0;
            }

        }

        /// <summary>
        /// Handle a gps time change.
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="h"></param>
        /// <param name="m"></param>
        /// <param name="s"></param>
        static void gpsTimeChange(MTK3339 sender, int h, int m, int s)
        {
            curGPSTIME.hour = h;
            curGPSTIME.min = m;
            curGPSTIME.sec = s;
        }

        /// <summary>
        /// Handle a gps date change
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="m"></param>
        /// <param name="d"></param>
        /// <param name="y"></param>
        static void gpsDateChange(MTK3339 sender, int m, int d, int y)
        {
            curGPSDATE.month = m;
            curGPSDATE.day = d;
            curGPSDATE.year = y;
        }

        #region TIMERS

        /// <summary>
        /// Timer handler to manually driver rover. Get commands from joystick.
        /// </summary>
        static void RoverTimer_Tick(GT.Timer timer)
        {
            //start_time = DateTime.Now;
            MOTORTIMEOUT_CNT++;
            ROVER_LOOP_CNT++;
            SIGNAL_QUALITY_ARRAY[SIGNAL_QUAL_INDEX] = 0;
            //end_time = DateTime.Now;

            lairdWirelessBuffer.LoadSerial(lairdComPort);
            if ((ps2DataLine = lairdWirelessBuffer.ReadLine()) != null)
            {
                try
                {
                    //verify checksum
                    if ((byte)(Convert.ToInt32(ps2DataLine.Substring(ps2DataLine.LastIndexOf("*") + 1, 2), 16)) == getChecksum(Encoding.UTF8.GetBytes(ps2DataLine)))
                    {
                        SIGNAL_QUALITY_ARRAY[SIGNAL_QUAL_INDEX] = 1.0 / SIGNAL_QUAL_PERIOD;

                        //ts = end_time - start_time;
                        //Debug.Print("Total time: " + ts.ToString());
                        //Debug.Print("Total time in seconds: " + ((float)ts.Ticks / TimeSpan.TicksPerSecond).ToString() + " seconds");
                        //start_time = DateTime.Now;

                        switch (ps2DataLine.Substring(0, 4))
                        {
                            case "$OJC": //Oakhill Joysick Control

                                servoAngle((byte)(Convert.ToInt32(ps2DataLine.Substring(5, 2), 16)));
                                motorSpeed((byte)(Convert.ToInt32(ps2DataLine.Substring(8, 2), 16)));

                                MOTORTIMEOUT_CNT = 0; //reset command timeout

                                if (DRIVE_MODE_ROVER != DRIVE_MODE.MANUAL)
                                    switchMode(DRIVE_MODE.MANUAL);

                                //'$','O','J','C',','  
                                //   ,'0','0',','  //bytes 5,6    get steering
                                //   ,'0','0',','  //bytes 8,9:   get motor speed
                                //   ,'0','0',     //bytes 11,12  get joystick clicks/expansion
                                //   ,'*'
                                //   ,'0','0'
                                //   ,0x0D,0x0A};

                                break;

                            case "$OAC": //Autonomous control

                                if (DRIVE_MODE_ROVER != DRIVE_MODE.AUTO)
                                    switchMode(DRIVE_MODE.AUTO);

                                //'$','O','A','C',','  
                                //   ,'0','0',        //bytes 5,6    get byte 1
                                //   ,'*'
                                //   ,'0','0'
                                //   ,0x0D,0x0A};

                                break;

                            case "$OCC": //Calibrate compass/IMU mode
                                if (DRIVE_MODE_ROVER != DRIVE_MODE.COMPASS)
                                    switchMode(DRIVE_MODE.COMPASS);

                                //'$','O','C','C',','  
                                //   ,'0','0',        //bytes 5,6    get byte 1
                                //   ,'*'
                                //   ,'0','0'
                                //   ,0x0D,0x0A};

                                break;

                            case "$OSS": //Set settings rover

                                try
                                {
                                    switchMode(DRIVE_MODE.MANUAL);

                                    string[] newSettings = ps2DataLine.Split(new char[] { ',', '*' });

                                    DECLINATION_ANGLE = Convert.ToDouble(newSettings[1]);

                                    string errData = "$ACK,OSS*";
                                    errData += (new string(byteToHex(getChecksum(Encoding.UTF8.GetBytes(errData)))) + "\r\n"); //add checksum, cr and lf
                                    lairdComPort.Write(Encoding.UTF8.GetBytes(errData), 0, errData.Length);
                                }
                                catch (Exception)
                                {
                                    string errData = "$NAK,OSS*";
                                    errData += (new string(byteToHex(getChecksum(Encoding.UTF8.GetBytes(errData)))) + "\r\n"); //add checksum, cr and lf
                                    Debug.Print("******$OSS Exception******\n\r");
                                }

                                break;

                            case"$OWP": //load a "waypoint" mission from the ground station

                                try
                                {
                                    switchMode(DRIVE_MODE.MANUAL);

                                    string[] waypoints = ps2DataLine.Split(new char[] { ',', '*' });
                                    targetWaypointList[0].dMapLatitude = Convert.ToDouble(waypoints[1]);
                                    targetWaypointList[0].dMapLongitude = Convert.ToDouble(waypoints[2]);
                                    targetWaypointList[1].dMapLatitude = Convert.ToDouble(waypoints[3]);
                                    targetWaypointList[1].dMapLongitude = Convert.ToDouble(waypoints[4]);
                                    targetWaypointList[2].dMapLatitude = Convert.ToDouble(waypoints[5]);
                                    targetWaypointList[2].dMapLongitude = Convert.ToDouble(waypoints[6]);
                                    targetWaypointList[3].dMapLatitude = Convert.ToDouble(waypoints[7]);
                                    targetWaypointList[3].dMapLongitude = Convert.ToDouble(waypoints[8]);

                                    string errData = "$ACK,OWP*";
                                    errData += (new string(byteToHex(getChecksum(Encoding.UTF8.GetBytes(errData)))) + "\r\n"); //add checksum, cr and lf
                                    lairdComPort.Write(Encoding.UTF8.GetBytes(errData), 0, errData.Length);
                                }
                                catch (Exception)
                                {
                                    string errData = "$NAK,OWP*";
                                    errData += (new string(byteToHex(getChecksum(Encoding.UTF8.GetBytes(errData)))) + "\r\n"); //add checksum, cr and lf
                                    Debug.Print("******$OWP Exception******\n\r");
                                }

                                break;

                            case "$GFX": //Ground Efx LED's

                                groundEfxLedControl((byte)Convert.ToInt32(ps2DataLine.Substring(5, 2), 16));

                                //'$','G','F','X',','  
                                //   ,'0','0',        //bytes 5,6    get byte 1
                                //   ,'*'
                                //   ,'0','0'
                                //   ,0x0D,0x0A};

                                break;

                            //case "$EFC": //Eco Fly Cam

                            //    //ecoFlyCam((byte)Convert.ToInt32(ps2DataLine.Substring(5, 2), 16));

                            //    //'$','E','F','C',','  
                            //    //   ,'0','0',       //bytes 5,6    eco cam byte 1
                            //    //   ,'*'
                            //    //   ,'0','0'
                            //    //   ,0x0D,0x0A};

                            //    break;

                            //case "$JPC": //Jpeg Camera

                            //    jpegCamControl( ps2DataLine.Substring(5, 2) );

                            //    //'$','J','P','C',','  
                            //    //   ,'0','0',       //bytes 5,6    
                            //    //   ,'*'
                            //    //   ,'0','0'
                            //    //   ,0x0D,0x0A};

                            //    break;
                        }

                        DEBUG_LED_STATE = !DEBUG_LED_STATE;
                        //led on shows connected
                        Mainboard.SetDebugLED(DEBUG_LED_STATE);
                    }

                }
                catch (Exception)
                { Debug.Print("******RoverTimer_Tick Cmd Exception********\n\r"); }
            }

            switch (DRIVE_MODE_ROVER)
            {
                case DRIVE_MODE.MANUAL:

                    //send rover state 
                    ManualMode_Tick(ROVER_LOOP_CNT);

                    //if timeout has occurred
                    if (MOTORTIMEOUT_CNT > 75)
                    {
                        //stop motor
                        motorSpeed(STOP_SPEED);
                        MOTORTIMEOUT_CNT = 76;
                        Mainboard.SetDebugLED(false);
                    }

                    break;

                case DRIVE_MODE.AUTO:

                    //send rover autonomous state data
                    AutonomousMode_Tick(ROVER_LOOP_CNT);

                    break;

                case DRIVE_MODE.COMPASS:

                    CompassMode_Tick(ROVER_LOOP_CNT);

                    break;
            }

            SIGNAL_QUALITY = 0;

            for (int i = 0; i < SIGNAL_QUAL_PERIOD; i++)
                SIGNAL_QUALITY += SIGNAL_QUALITY_ARRAY[i];

            SIGNAL_QUAL_INDEX = (SIGNAL_QUAL_INDEX + 1) % SIGNAL_QUAL_PERIOD;
            //Debug.Print("Signal Qual:" + SIGNAL_QUALITY);

            //end_time = DateTime.Now;
            //ts = end_time - start_time;
            ////Debug.Print("Total time: " + ts.ToString());
            //Debug.Print("Total time in seconds: " + ((float)ts.Ticks / TimeSpan.TicksPerSecond).ToString() + " seconds");
        }

        /// <summary>
        /// Timer to handle autonomous driving of rover.
        /// </summary>
        static void AutonomousMode_Tick(uint LOOP_TICK)
        {
            string tempDir = "?";
            string tempSpd = "?";
            
            //check sonar distance
            SONAR_DISTANCE = getSonarRangeAvg();

            //get current compass heading average
            ATTITUDE_VECTOR = IMU_BNO.getVector(BNO055.adafruit_vector_type_t.VECTOR_EULER);
            CURRENT_HEADING += ATTITUDE_VECTOR.x;

            if (LOOP_TICK % AUTO_MODE_TICK_INTERVAL == 0)
            {
                ERR_STATE = "$ACK,ORA";

                //update gps
                //roverGPS._gpsSP_DataPoll();

                targetWaypoint.dMapLatitude = targetWaypointList[targetWaypointNum].dMapLatitude;
                targetWaypoint.dMapLongitude = targetWaypointList[targetWaypointNum].dMapLongitude;

                //get current GPS position --------- PUT A VALID LOCK/FIX CHECK ON GPS
                if (roverGPS.FixAvailable)
                {
                    currentWaypoint.dMapLatitude = roverGPS.MapLatitude;
                    currentWaypoint.dMapLongitude = roverGPS.MapLongitude;

                    //currentWaypoint.dMapLatitude = 41.937618230023773; //test GPS points for Attleboro, MA
                    //currentWaypoint.dMapLongitude = -71.295427680015564; //test GPS points for Attleboro, MA

                    //currentWaypoint.dMapLatitude = 35.207644; //test GPS points for The Venue -- Charlotte, NC 
                    //currentWaypoint.dMapLongitude = -80.810116; //test GPS points for The Venue -- Charlotte, NC

                    try
                    {
                        // Process GPS --- update the course and distance to waypoint based on our new position
                        distanceToWaypoint();
                        courseToWaypoint();
                    }
                    catch (Exception)
                    {
                        ERR_STATE = "$NAK,ORA";
                        Debug.Print("******AutoMode_Tick Exception******\n\r");
                    }
                }
                else
                {
                    currentWaypoint.dMapLatitude = 0.0;
                    currentWaypoint.dMapLongitude = 0.0;
                }

                //get current compass heading
                //ATTITUDE_VECTOR = IMU_BNO.getVector(BNO055.adafruit_vector_type_t.VECTOR_EULER);
                //CURRENT_HEADING = (int)ATTITUDE_VECTOR.x;
                CURRENT_HEADING = (CURRENT_HEADING / AUTO_MODE_TICK_INTERVAL) + DECLINATION_ANGLE;

                calcDesiredTurn();

                //if (roverGPS.FixAvailable)
                moveAndAvoid();

                switch (turnDirection_DESIRED)
                {
                    case TURN.STRAIGHT:
                        tempDir = "STR";
                        break;
                    case TURN.LEFT:
                        tempDir = "LFT";
                        break;
                    case TURN.RIGHT:
                        tempDir = "RGT";
                        break;
                    default:
                        tempDir = "?";
                        break;
                }

                switch (CURRENT_NAV_SPEED)
                {
                    case STOP_SPEED:
                        tempSpd = "STP";
                        break;
                    case SLOW_SPEED: //slow and turn are same speed
                        tempSpd = "SLW";
                        break;
                    case NORMAL_SPEED:
                        tempSpd = "NRM";
                        break;
                    case FAST_SPEED:
                        tempSpd = "FST";
                        break;
                    case REVERSE_SPEED:
                        tempSpd = "REV";
                        break;
                    default:
                        tempSpd = "?";
                        break;
                }

                //output oakhill rover autonomous
                roverData = "$ORA,"
                                  + SONAR_DISTANCE
                            + "," + (int)CURRENT_HEADING
                            + "," + currentWaypoint.dMapLatitude
                            + "," + currentWaypoint.dMapLongitude
                            + "," + distanceToTarget
                            + "," + TARGET_HEADING
                            + "," + targetWaypointNum
                            + "," + tempDir//(byte)turnDirection_DESIRED
                            + "," + tempSpd//(byte)CURRENT_NAV_SPEED
                            + "," + ((byte)DRIVE_MODE_ROVER).ToString()
                            + "," + roverGPS.SatellitesUsed
                            + "," + (int)roverGPS.HorizontalDilution
                            + "," + ERR_STATE

                    //+ "," + (1.0 - ((IMU_Adafruit.Bmp180.Pressure/101910)^.19)) //convert pressure to altitude in meters, 1019.1hPa as sealevel in Pawtucket
                            + "*";

                roverData += (new string(byteToHex(getChecksum(Encoding.UTF8.GetBytes(roverData)))) + "\r\n"); //add checksum, cr and lf

                ////log auto data
                //WriteFileLines("roverLog.txt", new string[] {curGPSDATE.month.ToString() + "/" + curGPSDATE.day.ToString() + "/" + curGPSDATE.year.ToString() + "," +
                //                                             curGPSTIME.hour.ToString() + ":" + curGPSTIME.min.ToString() + ":" + curGPSTIME.sec.ToString() + "," + 
                //                                              roverGPS.SatellitesUsed.ToString() + "," + roverData.TrimEnd(new char[] { '\n', '\r' })  });

                //curGPSTIME.hour = 0; curGPSTIME.min = 0; curGPSTIME.sec = 0;

                CURRENT_HEADING = 0;

                Debug.Print(roverData);
                lairdComPort.Write(Encoding.UTF8.GetBytes(roverData), 0, roverData.Length);
            }
        }

        /// <summary>
        /// Timer handler to report Rover data to controller.
        /// </summary>
        static void ManualMode_Tick(uint LOOP_TICK)
        {
            //get current compass heading average
            ATTITUDE_VECTOR = IMU_BNO.getVector(BNO055.adafruit_vector_type_t.VECTOR_EULER);
            CURRENT_HEADING += ATTITUDE_VECTOR.x;

            if (LOOP_TICK % MANUAL_MODE_TICK_INTERVAL == 0)
            {
                int _temperature = 0;
                int _pressure = 0;
                //CURRENT_HEADING = 0;

                //update gps
                //roverGPS._gpsSP_DataPoll();

                //ATTITUDE_VECTOR = IMU_BNO.getVector(BNO055.adafruit_vector_type_t.VECTOR_EULER);
                //CURRENT_HEADING = ATTITUDE_VECTOR.x;
                CURRENT_HEADING = (CURRENT_HEADING / MANUAL_MODE_TICK_INTERVAL) + DECLINATION_ANGLE;

                _temperature = (int)(IMU_BNO.getTemperature() * 1.8000) + 32;

                if (roverGPS.FixAvailable)
                {
                    currentWaypoint.dMapLatitude = roverGPS.MapLatitude;
                    currentWaypoint.dMapLongitude = roverGPS.MapLongitude;
                }
                else
                {
                    currentWaypoint.dMapLatitude = 0.0;
                    currentWaypoint.dMapLongitude = 0.0;
                }

                try
                {
                    //output oakhill rover data
                    roverData = "$ORD,"
                                      + getBatteryVoltage()
                                + "," + getBatteryCurrent()
                                + "," + getSonarRangeAvg()
                                + "," + (int)CURRENT_HEADING
                                + "," + _temperature
                                + "," + _pressure
                                + "," + currentWaypoint.dMapLatitude
                                + "," + currentWaypoint.dMapLongitude
                                + "," + roverGPS.FixAvailable
                                + "," + ((byte)DRIVE_MODE_ROVER).ToString()
                                + "," + roverGPS.SatellitesUsed
                                + "," + (int)roverGPS.SpeedInMPH
                                + "," + (int)roverGPS.HorizontalDilution

                                //+ "," + (1.0 - ((IMU_Adafruit.Bmp180.Pressure/101910)^.19)) //convert pressure to altitude in meters, 1019.1hPa as sealevel in Pawtucket
                                + "*";

                    roverData += (new string(byteToHex(getChecksum(Encoding.UTF8.GetBytes(roverData)))) + "\r\n"); //add checksum, cr and lf

                    //WriteFileLines("roverLog.txt", new string[] { roverData });
                    Debug.Print(roverData);
                    lairdComPort.Write(Encoding.UTF8.GetBytes(roverData), 0, roverData.Length);
                }
                catch (Exception)
                {
                    roverData = "$NAK,ORD*";
                    roverData += (new string(byteToHex(getChecksum(Encoding.UTF8.GetBytes(roverData)))) + "\r\n"); //add checksum, cr and lf
                    Debug.Print("******ManualMode_Tick Exception******\n\r");
                }

                CURRENT_HEADING = 0;
            }
        }

        /// <summary>
        /// Timer handler to report Imu data to controller.
        /// </summary>
        static void CompassMode_Tick(uint LOOP_TICK)
        {
            if (LOOP_TICK % COMPASS_MODE_TICK_INTERVAL == 0)
            {
                byte[] sysStatus = new byte[3];

                IMU_BNO.getSystemStatus(ref sysStatus[0], ref sysStatus[1], ref sysStatus[2]);
                ATTITUDE_VECTOR = IMU_BNO.getVector(BNO055.adafruit_vector_type_t.VECTOR_EULER);

                roverData = "$OCC,"
                          + IMU_BNO.isFullyCalibrated()
                    + "," + sysStatus[0]
                    + "," + sysStatus[1]
                    + "," + sysStatus[2]
                    + "," + IMU_BNO.Calibration_SYS
                    + "," + IMU_BNO.Calibration_MAG
                    + "," + IMU_BNO.Calibration_ACCEL
                    + "," + IMU_BNO.Calibration_GYRO
                    + "," + (int)ATTITUDE_VECTOR.x
                    + "," + (int)ATTITUDE_VECTOR.y
                    + "," + (int)ATTITUDE_VECTOR.z
                    + "*";

                //saves sensor offsets if sensor is calibrated and settings have not been saved yet.
                if (IMU_BNO.getSensorOffsets() && !BNO_SETTINGS_SAVED)
                {
                    if (!roverSDCARD.IsCardMounted)
                    {
                        roverSDCARD.Mount();
                        Thread.Sleep(250);
                    }

                    if (roverSDCARD.IsCardMounted)
                        SD_ROOT_DIRECTORY = VolumeInfo.GetVolumes()[0].RootDirectory;

                    ////Save sensor offsets to file.
                    bool writeSuccess = WriteFileLines(BNO_SETTINGS_FILE, new string[] {
                    IMU_BNO.BNO_SENSOR_OFFSETS.accel_offset_x.ToString(),
                    IMU_BNO.BNO_SENSOR_OFFSETS.accel_offset_y.ToString(),
                    IMU_BNO.BNO_SENSOR_OFFSETS.accel_offset_z.ToString(),
                    IMU_BNO.BNO_SENSOR_OFFSETS.gyro_offset_x.ToString(), 
                    IMU_BNO.BNO_SENSOR_OFFSETS.gyro_offset_y.ToString(), 
                    IMU_BNO.BNO_SENSOR_OFFSETS.gyro_offset_z.ToString(), 
                    IMU_BNO.BNO_SENSOR_OFFSETS.mag_offset_x.ToString(),
                    IMU_BNO.BNO_SENSOR_OFFSETS.mag_offset_y.ToString(),
                    IMU_BNO.BNO_SENSOR_OFFSETS.mag_offset_z.ToString(),
                    IMU_BNO.BNO_SENSOR_OFFSETS.accel_radius.ToString(),
                    IMU_BNO.BNO_SENSOR_OFFSETS.mag_radius.ToString()
                    });

                    roverSDCARD.Unmount();

                    if (writeSuccess) { Debug.Print("BNO sensor offsets saved to file."); BNO_SETTINGS_SAVED = true;}
                    else Debug.Print("Could not save BNO sensor offsetsto file.");
                }

                //BNO055.Vector attitude_VEC;
                //BNO055.Vector attitude_MAG;
                //BNO055.Vector attitude_ACC;
                //BNO055.Vector attitude_GYRO;
                //attitude_VEC = IMU_BNO.getVector(BNO055.adafruit_vector_type_t.VECTOR_EULER);
                //attitude_MAG = IMU_BNO.getVector(BNO055.adafruit_vector_type_t.VECTOR_MAGNETOMETER);
                //attitude_ACC = IMU_BNO.getVector(BNO055.adafruit_vector_type_t.VECTOR_ACCELEROMETER);
                //attitude_GYRO = IMU_BNO.getVector(BNO055.adafruit_vector_type_t.VECTOR_GYROSCOPE);

                //output compass/IMU data
                //roverData = "Raw:"
                //             + attitude_VEC.x
                //       + "," + attitude_VEC.y
                //       + "," + attitude_VEC.z
                //       + "," + attitude_MAG.x
                //       + "," + attitude_MAG.y
                //       + "," + attitude_MAG.z
                //       + "," + attitude_ACC.x
                //       + "," + attitude_ACC.y
                //       + "," + attitude_ACC.z
                //       + "," + attitude_GYRO.x
                //       + "," + attitude_GYRO.y
                //       + "," + attitude_GYRO.z
                //       + "," + IMU_BNO.Calibration_SYS
                //       + "," + IMU_BNO.Calibration_MAG
                //       + "," + IMU_BNO.Calibration_ACCEL
                //       + "," + IMU_BNO.Calibration_GYRO;

                //while (!IMU_BNO.isFullyCalibrated())
                //{
                //    IMU_BNO.getSystemStatus(ref sysStatus[0], ref sysStatus[1], ref sysStatus[2]);
                //    Debug.Print("SystemStatus: " + sysStatus[0].ToString() + " -- " + sysStatus[1].ToString() + " -- " + sysStatus[2].ToString());

                //    Thread.Sleep(500);

                //    ATTITUDE_VECTOR = IMU_BNO.getVector(BNO055.adafruit_vector_type_t.VECTOR_EULER);
                //    Debug.Print("Euler: X:" + ATTITUDE_VECTOR.x + " Y:" + ATTITUDE_VECTOR.y + " Z:" + ATTITUDE_VECTOR.z);

                //    Thread.Sleep(500);

                //    Debug.Print("SYS Cal: " + IMU_BNO.Calibration_SYS);
                //    Debug.Print("Mag Cal: " + IMU_BNO.Calibration_MAG);
                //    Debug.Print("Accel Cal: " + IMU_BNO.Calibration_ACCEL);
                //    Debug.Print("Gyro Cal: " + IMU_BNO.Calibration_GYRO);

                //    Thread.Sleep(500);
                //}

                //      + IMU_BNO.Gyroscope.RawX
                //+ "," + IMU_Adafruit.Accelerometer.RawY
                //+ "," + IMU_Adafruit.Accelerometer.RawZ

                //+ "," + IMU_Adafruit.Gyroscope.RawX
                //+ "," + IMU_Adafruit.Gyroscope.RawY
                //+ "," + IMU_Adafruit.Gyroscope.RawZ

                // + "," + IMU_Adafruit.Magnetometer.RawX
                //+ "," + IMU_Adafruit.Magnetometer.RawY
                //+ "," + IMU_Adafruit.Magnetometer.RawZ;

                //roverData += "\r\n"; //add  cr and lf
                roverData += (new string(byteToHex(getChecksum(Encoding.UTF8.GetBytes(roverData)))) + "\r\n"); //add checksum, cr and lf

                Debug.Print(roverData);
                lairdComPort.Write(Encoding.UTF8.GetBytes(roverData), 0, roverData.Length);
                
                //try
                //{
                //    OHS_UartMux.MuxComPort.Write(Encoding.UTF8.GetBytes(roverData), 0, roverData.Length);
                //}
                //catch
                //{
                //    Debug.Print("Compass Out Error.");
                //}
            }
        }

        #endregion

        #region JPEG Camera

        ///// <summary>
        ///// Control the Eco Fly Video Camera.
        ///// </summary>
        //static bool jpegCamControl(string data)
        //{
        //    switch (data)
        //    {
        //        case "00":
        //            //stop cam
        //            break;

        //        case "01":
        //            getPicture();
        //        break;

        //        case "02":
        //            //serial cam on
        //        break;


        //    }

        //    return false;
        //}


        ///// <summary>
        ///// Causes camera to takes a picture
        ///// </summary>
        //static bool getPicture()
        //{
        //    //multiComPort.ActiveComPort = 0; //set to camera port

        //    //DoSuccessFail(CAMERA.Reset, "Reset successfull", "ERROR resetting");
        //    //DoSuccessFail(() => CAMERA.SetPictureSize(LinkspriteCamera.SET_SIZE_640x480), "Size change successfull", "ERROR changing size");
        //    //DoSuccessFail(CAMERA.Reset, "Reset successfull", "ERROR resetting");

        //    if (LINKSPRITE_CAMERA.Reset())
        //    {
        //        //Debug.Print("Looking for file and deleting it if exists.");
        //        if (File.Exists(SD_ROOT_DIRECTORY + @"\" + JPEG_FILENAME + JPEG_FILENUMBER.ToString() + ".jpg"))
        //            File.Delete(SD_ROOT_DIRECTORY + @"\" + JPEG_FILENAME + JPEG_FILENUMBER.ToString() + ".jpg");

        //        //Debug.Print("Testing write...");
        //        FILE_HANDLE = new FileStream(SD_ROOT_DIRECTORY + @"\" + JPEG_FILENAME + JPEG_FILENUMBER.ToString() + ".jpg", FileMode.Create, FileAccess.Write);

        //        LINKSPRITE_CAMERA.GetPicture(SaveCameraChunk);

        //        if (DoSuccessFail(LINKSPRITE_CAMERA.Stop, "Stop successful", "ERROR stopping"))
        //        {
        //            JPEG_FILENUMBER++;
        //            FILE_HANDLE.Close();
        //            return true;
        //        }

        //        FILE_HANDLE.Close();
        //    }

        //    return false;
        //    //turn off read.timeout that Linksprite class sets?
        //    //multiComPort.ComPort.ReadTimeout = -1;

        //    //ps.UnmountFileSystem();
        //}

        ///// <summary>
        ///// Receives bytes from camera
        ///// </summary>
        ///// <param name="bytes">Jpeg picture bytes to write to storage.</param>
        //private static void SaveCameraChunk(byte[] bytes)
        //{
        //    //write bytes to SD card
        //    FILE_HANDLE.Write(bytes, 0, bytes.Length);

        //    //foreach (var byter in bytes)
        //    //Debug.Print(byter.ToString());
        //}

        //private delegate bool FuncBool();

        //private static bool DoSuccessFail(FuncBool action, string success, string fail)
        //{
        //    if (action())
        //    {
        //        Debug.Print(success);
        //        return true;
        //    }

        //    Debug.Print(fail);
        //    return false;
        //}

        #endregion

        #region STEERING/MOTOR

        /// <summary>
        /// Set the servo angle used for steering.
        /// </summary>
        /// <param name="angle">Relative servo angle. (0 to 255, 128 =  center)</param>
        static void servoAngle(byte angle)
        {
            //0x00 is full left on truck, 0xFF (255) is full right, about 128 is center

            //byte fullLeft = 0;
            //byte fullRight = 255;
            //byte center = 128;

            motor_steering_ComPort.Write(new byte[] { 0xFF, 0x01, angle }, 0, 3);
            //Debug.Print("Steering: " + angle.ToString());

            if(DRIVE_MODE_ROVER == DRIVE_MODE.AUTO) turnDirection_CURRENT = (TURN)angle;
        }

        /// <summary>
        /// Set the motor speed.
        /// </summary>
        /// <param name="speed">Relative motor speed. (0 to 255, 75 = center & null)</param>
        static void motorSpeed(byte speedCmd)
        {
            //0x00 is full rev on controller trigger, about 75 is center, 0xFF (255) is full forward on trigger
            //about 139 is start of forward on truck, 255 is max forward speed.
            //on landRover 0 to 134 is reverse, 135 = stop, 136 to 255 is forward.
            //about 139 is start of forward on truck

            //if ((speedCmd > 70) && (speedCmd < 90)) speedCmd = MOTORSTOP; //deadband around neutral
            //else if (speedCmd >= 90) speedCmd = (byte)convertLinearScale(speedCmd, 90, 255, 136, 255);
            //else speedCmd = (byte)convertLinearScale(speedCmd, 0, 70, 0, 134);

            if (DRIVE_MODE_ROVER == DRIVE_MODE.AUTO)
            {
                byte tempSpeed = STOP_SPEED;
                NEW_NAV_SPEED = speedCmd;

                if (NEW_NAV_SPEED == REVERSE_SPEED)
                {
                    motor_steering_ComPort.Write(new byte[] { 0xFF, 0x02, STOP_SPEED }, 0, 3);

                    tempSpeed = STOP_SPEED;
                    //reduce to REVERSE SPEED
                    while (tempSpeed != 0)
                    {
                        Thread.Sleep(10);
                        motor_steering_ComPort.Write(new byte[] { 0xFF, 0x02, (byte)(tempSpeed = (byte)(tempSpeed * .9)) }, 0, 3);
                    }

                    //increase to STOP_SPEED
                    while (tempSpeed <= STOP_SPEED)
                    {
                        Thread.Sleep(10);
                        motor_steering_ComPort.Write(new byte[] { 0xFF, 0x02, tempSpeed += 5 }, 0, 3);
                    }

                    //reduce to REVERSE SPEED
                    while (tempSpeed != 0)
                    {
                        Thread.Sleep(10);
                        motor_steering_ComPort.Write(new byte[] { 0xFF, 0x02, (byte)(tempSpeed = (byte)(tempSpeed * .9)) }, 0, 3);
                    }
                }
                else if (NEW_NAV_SPEED >= CURRENT_NAV_SPEED) //increase or maintain speed
                {
                    tempSpeed = CURRENT_NAV_SPEED;
                    motor_steering_ComPort.Write(new byte[] { 0xFF, 0x02, CURRENT_NAV_SPEED }, 0, 3);

                    while (tempSpeed < NEW_NAV_SPEED)
                    {
                        Thread.Sleep(50);
                        motor_steering_ComPort.Write(new byte[] { 0xFF, 0x02, tempSpeed += 5 }, 0, 3);
                    }
                }
                else if (NEW_NAV_SPEED < CURRENT_NAV_SPEED) //decrease speed
                {
                    tempSpeed = CURRENT_NAV_SPEED;
                    motor_steering_ComPort.Write(new byte[] { 0xFF, 0x02, CURRENT_NAV_SPEED }, 0, 3);

                    while (tempSpeed > NEW_NAV_SPEED)
                    {
                        Thread.Sleep(50);
                        motor_steering_ComPort.Write(new byte[] { 0xFF, 0x02, tempSpeed -= 5 }, 0, 3);
                    }
                }

                CURRENT_NAV_SPEED = NEW_NAV_SPEED;

                return;
            }

            motor_steering_ComPort.Write(new byte[] { 0xFF, 0x02, (byte)speedCmd }, 0, 3);

            //Debug.Print("Motor Speed: " + speed.ToString());
        }

        #endregion

        #region GROUND EFX LIGHT CONTROL

        /// <summary>
        /// Set the sensors and accesories of the rover.
        /// </summary>
        static void groundEfxLedControl(byte data)
        {
            //ground efx lights
            if ((data & 1) == 1)  //1
                GroundEfxTimer.Start(); //groundEfxShow(true);
            else
                GroundEfxTimer.Stop(); //groundEfxShow(false); //0

            //control left side ground efx lights 
            if ((data & 2) == 2)
                lightOnOff("Red", "");
            else
                lightOnOff("RedOff", "");

            if ((data & 4) == 4)
                lightOnOff("Green", "");
            else
                lightOnOff("GreenOff", "");

            if ((data & 8) == 8)
                lightOnOff("Blue", "");
            else
                lightOnOff("BlueOff", "");

            //control right side ground efx lights 
            if ((data & 16) == 16)
                lightOnOff("", "Red");
            else
                lightOnOff("", "RedOff");

            if ((data & 32) == 32)
                lightOnOff("", "Green");
            else
                lightOnOff("", "GreenOff");

            if ((data & 64) == 64)
                lightOnOff("", "Blue");
            else
                lightOnOff("", "BlueOff");

        }

        /// <summary>
        /// Timer to run ground efx led's.
        /// </summary>
        /// <param name="timer"></param>
        void GroundEfxTimer_Tick(GT.Timer timer)
        {
            groundEfxShow(true);
        }

        /// <summary>
        /// Simple ground efx light control. Red, RedOff, Blue, BlueOff, Green, GreenOff, Off
        /// </summary>
        /// <param name="leftColor">Set left side color.</param>
        /// <param name="rightColor">Set right side color.</param>
        static void lightOnOff(string leftColor, string rightColor)
        {
            //Left:  Red = P3, Green = P1, Blue = P2
            //Right: Red = P5, Green = P7, Blue = P6

            switch (leftColor)
            {
                case "Red":
                    roverLoadBrd.P3.Write(true);
                    break;

                case "RedOff":
                    roverLoadBrd.P3.Write(false);
                    break;

                case "Blue":
                    roverLoadBrd.P2.Write(true);
                    break;

                case "BlueOff":
                    roverLoadBrd.P2.Write(false);
                    break;

                case "Green":
                    roverLoadBrd.P1.Write(true);
                    break;

                case "GreenOff":
                    roverLoadBrd.P1.Write(false);
                    break;

                case "Off":
                    roverLoadBrd.P1.Write(false);
                    roverLoadBrd.P2.Write(false);
                    roverLoadBrd.P3.Write(false);
                    break;

                default:
                    //do nothing to light
                    break;
            }

            switch (rightColor)
            {
                case "Red":
                    roverLoadBrd.P5.Write(true);
                    break;

                case "RedOff":
                    roverLoadBrd.P5.Write(false);
                    break;

                case "Blue":
                    roverLoadBrd.P6.Write(true);
                    break;

                case "BlueOff":
                    roverLoadBrd.P6.Write(false);
                    break;

                case "Green":
                    roverLoadBrd.P7.Write(true);
                    break;

                case "GreenOff":
                    roverLoadBrd.P7.Write(false);
                    break;

                case "Off":
                    roverLoadBrd.P5.Write(false);
                    roverLoadBrd.P6.Write(false);
                    roverLoadBrd.P7.Write(false);
                    break;

                default:
                    //do nothing
                    break;
            }

        }

        /// <summary>
        /// Random ground efx light show.
        /// </summary>
        static void groundEfxShow(bool OnOff)
        {
            //Left:  Red = P3, Green = P2, Blue = P1
            //Right: Red = P5, Green = P6, Blue = P7

            bool[] lightStates = new bool[] { false, false, false, false, false, false, false, false };

            if (OnOff)
            {
                byte randByte = 0;
                Random LightShow = new Random(DateTime.Now.Second);

                randByte = (byte)LightShow.Next(255);

                for (var i = 0; i < lightStates.Length; i++)
                {
                    if (((randByte >>= 1) & 0x01) == 1)
                        lightStates[i] = true;
                    else
                        lightStates[i] = false;
                }
            }

            roverLoadBrd.P1.Write(lightStates[0]);
            roverLoadBrd.P2.Write(lightStates[1]);
            roverLoadBrd.P3.Write(lightStates[2]);
            roverLoadBrd.P5.Write(lightStates[3]);
            roverLoadBrd.P6.Write(lightStates[4]);
            roverLoadBrd.P7.Write(lightStates[5]);
        }

        #endregion

        #region ECO VIDEO CAM CONTROL

        /// <summary>
        /// Control the Eco Fly Video Camera.
        /// </summary>
        //static bool ecoFlyCam(byte data)
        //{
        //    if ((data & 1) == 1)
        //        return startEcoCamMode(videoCamMode.Video);
        //    if ((data & 2) == 2)
        //        return startEcoCamMode(videoCamMode.SerialPhoto);
        //    if ((data & 4) == 4)
        //        return startEcoCamMode(videoCamMode.SinglePhoto);
        //    if ((data & 8) == 8)
        //        return stopEcoCamMode();

        //    return false;
        //}

        ///// <summary>
        ///// Starts a given ECO cam mode
        ///// </summary>
        ///// <param name="newCamMode"></param>
        ///// <returns>True if successful.</returns>
        //static bool startEcoCamMode(videoCamMode newCamMode)
        //{
        //    if (EcoCamMode == newCamMode)
        //        return true;

        //    for (int i = 0; i < 5; i++)
        //    {
        //        if (newCamMode == getEcoCamMode(videoCamGreenLed))
        //        {
        //            //start the mode
        //            toggleEcoCamStartStop();
        //            //set mode
        //            EcoCamMode = newCamMode;
        //            return true;
        //        }
        //        else
        //        {
        //            changeEcoCamMode();
        //        }
        //    }

        //    EcoCamMode = videoCamMode.Unknown;

        //    return false;
        //}

        ///// <summary>
        ///// Stops current ECO cam record mode.
        ///// </summary>
        ///// <param name="newCamMode"></param>
        ///// <returns>True if successful.</returns>
        //static bool stopEcoCamMode()
        //{
        //    if (EcoCamMode == videoCamMode.Stopped)
        //        return true;

        //    //single photo stops itself
        //    if (EcoCamMode == videoCamMode.SinglePhoto)
        //    {
        //        EcoCamMode = videoCamMode.Stopped;
        //        return true;
        //    }

        //    videoCamMode tempCamMode = videoCamMode.Unknown;

        //    //serial photo red led timing is inconsistent so can't get mode consistently
        //    //just use change mode to back out of a record mode
        //    if ((EcoCamMode == videoCamMode.SerialPhoto) || (EcoCamMode == videoCamMode.Video))
        //        toggleEcoCamStartStop();

        //    tempCamMode = getEcoCamMode(videoCamGreenLed);

        //    if ((tempCamMode == videoCamMode.SerialPhoto) || (tempCamMode == videoCamMode.Video) || (tempCamMode == videoCamMode.SinglePhoto))
        //    {
        //        EcoCamMode = videoCamMode.Stopped;
        //        return true;
        //    }

        //    changeEcoCamMode();
        //    EcoCamMode = videoCamMode.Unknown;
        //    return false;
        //}

        ///// <summary>
        ///// Gets current ECO Camera mode.
        ///// </summary>
        ///// <param name="ledInput">An analog input connected to ECO cam mode led. </param>
        ///// <returns>videoCamMode enum</returns>
        //static videoCamMode getEcoCamMode(GT.Socket.SocketInterfaces.AnalogInput ledInput)
        //{
        //    bool[] buffer = new bool[15];
        //    int trueCount = 0;
        //    int startCount = 0;

        //    for (int i = 0; i < buffer.Length; i++)
        //    {
        //        if (ledInput.ReadVoltage() > 1.5)
        //            buffer[i] = true;
        //        else
        //            buffer[i] = false;

        //        Thread.Sleep(200);
        //    }

        //    //find pulse edge 
        //    for (int i = 1; i < buffer.Length; i++)
        //    {
        //        if (buffer[i - 1] != buffer[i])
        //        {
        //            startCount = i;
        //            break;
        //        }
        //    }

        //    for (int i = startCount; i < buffer.Length - 1; i++)
        //    {
        //        if (buffer[i + 1] == buffer[i])
        //            trueCount++;
        //        else break;
        //    }

        //    if (trueCount >= 13)
        //        EcoCamMode = videoCamMode.Video;
        //    else if (trueCount >= 6)
        //        EcoCamMode = videoCamMode.SinglePhoto;
        //    else if (trueCount >= 3)
        //        EcoCamMode = videoCamMode.SerialPhoto;
        //    else
        //        EcoCamMode = videoCamMode.Unknown;

        //    return EcoCamMode;
        //}

        ///// <summary>
        ///// Changes ECO Camera's mode. Also backs you out of video and serial record. 
        ///// Single pic mode backs out on its own after one pic.
        ///// </summary>
        //static void changeEcoCamMode()
        //{
        //    MakePinOutput(vidCamSelectBtn);
        //    vidCamSelectBtn.Write(false);
        //    Thread.Sleep(5500);

        //    MakePinInput(vidCamSelectBtn);

        //    Thread.Sleep(500);
        //}

        ///// <summary>
        ///// Toggles a ECO camera mode between ready and recording.
        ///// </summary>
        //static void toggleEcoCamStartStop()
        //{
        //    MakePinOutput(vidCamSelectBtn);
        //    vidCamSelectBtn.Write(false);
        //    Thread.Sleep(500);

        //    MakePinInput(vidCamSelectBtn);

        //    Thread.Sleep(500);
        //}

        /// <summary>
        /// Make a tristate pin an output.
        /// </summary>
        /// <param name="port">The tristate pin reference.</param>
        static void MakePinOutput(Microsoft.SPOT.Hardware.TristatePort port)
        {
            if (port.Active == false)
                port.Active = true;

        }

        /// <summary>
        /// Make a tristate pin an input.
        /// </summary>
        /// <param name="port">The tristate pin refenece.</param>
        static void MakePinInput(Microsoft.SPOT.Hardware.TristatePort port)
        {
            if (port.Active == true)
                port.Active = false;
        }

        #endregion

        /// <summary>
        /// Calculate battery voltage. Returns the voltage measured times 10.
        /// </summary>
        static string getBatteryVoltage()
        {
            float tempBattery;

            //pin 4 port 14 is battery voltage monitor  63.69 mV/Volt
            tempBattery = (float)roverBatteryVoltage.ReadVoltage();
            tempBattery = tempBattery / .06369f;

            if (tempBattery.ToString().Length > 4)
                return tempBattery.ToString().Substring(0, 4);

            //Debug.Print("Bat Voltage: " + tempBattery.ToString() + "V");

            return tempBattery.ToString();
        }

        /// <summary>
        /// Calculate battery current.  Returns the current.
        /// </summary>
        static string getBatteryCurrent()
        {
            float tempBattery;

            //pin 5 port 14 is battery current monitor  36.60 mV/Amp
            tempBattery = (float)roverBatteryCurrent.ReadVoltage();
            tempBattery = tempBattery / .0366f;

            if (tempBattery.ToString().Length > 4)
                return tempBattery.ToString().Substring(0, 4);

            //Debug.Print("Bat Current: " + tempBattery.ToString() + "A");

            return tempBattery.ToString();
        }

        /// <summary>
        /// Get the range from the ultrasonic senosr.
        /// </summary>
        static int getRangeFwd()
        {
            double tempRange;

            //pin 3 port 13 is ultrasonic distance sensor Vcc/512 per inch --> 6.4mV per inch
            tempRange = roverDistanceSensor.ReadVoltage();
            tempRange = tempRange / .0064;
            //Debug.Print("Range: " + tempRange.ToString() + " inches\n\r");

            return (int)tempRange;
        }

        /// <summary>
        /// Get the moving average of the ultrasonic range sensor
        /// </summary>
        /// <returns></returns>
        static int getSonarRangeAvg()
        {
            double tempRange;
            double MA_RANGE = 0;

            //pin 3 port 13 is ultrasonic distance sensor Vcc/512 per inch --> 6.4mV per inch
            tempRange = roverDistanceSensor.ReadVoltage();

            MA_RANGE_ARRAY[MA_RANGE_INDEX] = tempRange / (.0064 * MA_RANGE_PERIOD);

            for (int i = 0; i < MA_RANGE_PERIOD; i++)
                MA_RANGE += MA_RANGE_ARRAY[i];

            MA_RANGE_INDEX = (MA_RANGE_INDEX + 1) % MA_RANGE_PERIOD;

            //Debug.Print("MA Range: " + MA_RANGE.ToString() + " inches\n\r");

            return (int)MA_RANGE;
        }

        #region NAVIGATION METHODS

        /// <summary>
        /// Calculate which way to turn to head to destination
        /// </summary>
        static void calcDesiredTurn()
        {
            HEADING_ERROR = TARGET_HEADING - (int)CURRENT_HEADING;

            // adjust for compass wrap
            if (HEADING_ERROR < -180)
                HEADING_ERROR += 360;
            if (HEADING_ERROR > 180)
                HEADING_ERROR -= 360;

            // calculate which way to turn to intercept the targetHeading
            if (System.Math.Abs(HEADING_ERROR) <= HEADING_TOLERANCE)      // if within tolerance, don't turn
                turnDirection_DESIRED = TURN.STRAIGHT;
            else if (HEADING_ERROR < 0)
                turnDirection_DESIRED = TURN.LEFT;
            else if (HEADING_ERROR > 0)
                turnDirection_DESIRED = TURN.RIGHT;
            else
                turnDirection_DESIRED = TURN.STRAIGHT;
        }

        /// <summary>
        /// returns distance in meters between two positions, both specified 
        /// as signed decimal-degrees latitude and longitude. Uses great-circle 
        /// distance computation for hypothetical sphere of radius 6372795 meters.
        /// Because Earth is no exact sphere, rounding errors may be up to 0.5%.
        /// copied from TinyGPS library
        /// </summary>
        /// <returns>Distance in meters between two GPS positions</returns>
        static int distanceToWaypoint()
        {
            double delta = ((System.Math.PI / 180) * (currentWaypoint.dMapLongitude - targetWaypoint.dMapLongitude)); //convert the angular difference to radians
            double sdlong = System.Math.Sin(delta);
            double cdlong = System.Math.Cos(delta);
            double lat1 = (System.Math.PI / 180) * (currentWaypoint.dMapLatitude); //convert to radians
            double lat2 = (System.Math.PI / 180) * (targetWaypoint.dMapLatitude); //convert to radians
            double slat1 = System.Math.Sin(lat1);
            double clat1 = System.Math.Cos(lat1);
            double slat2 = System.Math.Sin(lat2);
            double clat2 = System.Math.Cos(lat2);
            delta = (clat1 * slat2) - (slat1 * clat2 * cdlong);
            delta = delta * delta;
            delta += (clat2 * sdlong * clat2 * sdlong);
            delta = System.Math.Sqrt(delta);
            double denom = (slat1 * slat2) + (clat1 * clat2 * cdlong);
            delta = System.Math.Atan2(delta, denom);
            distanceToTarget = (int)(delta * 6372795);

            // check to see if we have reached the current waypoint
            if (distanceToTarget <= WAYPOINT_DISTANCE_TOLERANCE)
                nextWaypoint();

            return distanceToTarget;
        }  // distanceToWaypoint()

        /// <summary>
        /// Update the target waypoint
        /// </summary>
        static void nextWaypoint()
        {
            targetWaypointNum++;

            if ((targetWaypoint.dMapLatitude == 0 && targetWaypoint.dMapLongitude == 0) || targetWaypointNum >= NUMBER_WAYPOINTS)    // last waypoint reached? 
            {
                //end mission
                //stop motor and neutral steering
                motorSpeed(STOP_SPEED);
                servoAngle((byte)TURN.STRAIGHT);

                switchMode(DRIVE_MODE.MANUAL);

                return;
                //send mission complete msg to controller
            }

            targetWaypoint.dMapLatitude = targetWaypointList[targetWaypointNum].dMapLatitude;
            targetWaypoint.dMapLongitude = targetWaypointList[targetWaypointNum].dMapLongitude;

            // update the course and distance to waypoint based on our new position
            distanceToTarget = distanceToWaypoint();
            //courseToWaypoint();
            //distanceToTarget = originalDistanceToTarget = distanceToWaypoint();
            //courseToWaypoint();

        }  // nextWaypoint()

        /// <summary>
        /// returns course in degrees (North=0, West=270) from position 1 to position 2,
        /// both specified as signed decimal-degrees latitude and longitude.
        /// Because Earth is no exact sphere, calculated course may be off by a tiny fraction.
        /// copied from TinyGPS library
        /// </summary>
        /// <returns>Course in degrees (North=0, West=270) from position 1 to position 2</returns>
        static int courseToWaypoint()
        {
            double dlon = ((System.Math.PI / 180) * (targetWaypoint.dMapLongitude - currentWaypoint.dMapLongitude));
            double cLat = (System.Math.PI / 180) * (currentWaypoint.dMapLatitude);
            double tLat = (System.Math.PI / 180) * (targetWaypoint.dMapLatitude);
            double a1 = System.Math.Sin(dlon) * System.Math.Cos(tLat);
            double a2 = System.Math.Sin(cLat) * System.Math.Cos(tLat) * System.Math.Cos(dlon);
            a2 = System.Math.Cos(cLat) * System.Math.Sin(tLat) - a2;
            a2 = System.Math.Atan2(a1, a2);
            if (a2 < 0.0)
            {
                a2 += (2 * System.Math.PI);
            }
            TARGET_HEADING = (int)((a2 * 180) / System.Math.PI);
            return TARGET_HEADING;
        }

        /// <summary>
        /// Drive and avoid obstacles if necessary.
        /// </summary>
        static void moveAndAvoid()
        {
            //use this for no obstacle avoidance
            if (turnDirection_DESIRED == TURN.STRAIGHT)
                motorSpeed(FAST_SPEED);
            else
                motorSpeed(TURN_SPEED);

            servoAngle((byte)turnDirection_DESIRED);

            return;

            /*  USE THIS TO TURN ON OBSTACLE AVOIDANCE
            if (SONAR_DISTANCE >= SAFE_DISTANCE)       // no close objects in front of car
            {
                if (turnDirection_DESIRED == TURN.STRAIGHT)
                    motorSpeed(FAST_SPEED);
                else
                    motorSpeed(TURN_SPEED);

                servoAngle((byte)turnDirection_DESIRED);

                return;
            }

            if (SONAR_DISTANCE > TURN_DISTANCE && SONAR_DISTANCE < SAFE_DISTANCE)    // not yet time to turn, but slow down
            {
                if (turnDirection_DESIRED == TURN.STRAIGHT)
                    motorSpeed(NORMAL_SPEED);
                else
                    motorSpeed(TURN_SPEED);

                servoAngle((byte)turnDirection_DESIRED);     //turn to navigate

                return;
            }

            if (SONAR_DISTANCE < TURN_DISTANCE && SONAR_DISTANCE > STOP_DISTANCE)  // getting close, time to turn to avoid object        
            {
                motorSpeed(SLOW_SPEED);      // slow down

                switch (turnDirection_CURRENT)
                {
                    case TURN.STRAIGHT:                  // going straight currently, so start new turn
                        {
                            if (HEADING_ERROR <= 0)
                                turnDirection_DESIRED = TURN.LEFT;
                            else
                                turnDirection_DESIRED = TURN.RIGHT;
                            servoAngle((byte)turnDirection_DESIRED);  // turn in the new direction
                            break;
                        }
                    case TURN.LEFT:                         // if already turning left, try right
                        {
                            servoAngle((byte)TURN.RIGHT);
                            break;
                        }
                    case TURN.RIGHT:                       // if already turning right, try left
                        {
                            servoAngle((byte)TURN.LEFT);
                            break;
                        }
                } // end SWITCH

                return;
            }


            if (SONAR_DISTANCE < STOP_DISTANCE)              // too close, stop and back up
            {
                motorSpeed(STOP_SPEED);                      // stop 
                servoAngle((byte)TURN.STRAIGHT);             // straighten up
                turnDirection_DESIRED = TURN.STRAIGHT;
                motorSpeed(REVERSE_SPEED);                   // go back at higher speed

                while (SONAR_DISTANCE < TURN_DISTANCE)       // backup until we get safe clearance
                {
                    //get current GPS position --------- PUT A VALID LOCK/FIX CHECK ON GPS
                    currentWaypoint.dMapLatitude = roverGPS.MapLatitude;
                    currentWaypoint.dMapLongitude = roverGPS.MapLongitude;

                    // Process GPS -- update the course and distance to waypoint based on our new position
                    distanceToWaypoint();
                    courseToWaypoint();

                    //get current compass heading
                    ATTITUDE_VECTOR = IMU_BNO.getVector(BNO055.adafruit_vector_type_t.VECTOR_EULER);

                    CURRENT_HEADING = (int)ATTITUDE_VECTOR.x;
                    calcDesiredTurn();  // calculate how we would optimatally turn, without regard to obstacles   

                    //check sonar distance
                    SONAR_DISTANCE = getSonarRangeAvg();
                    Debug.Print("$REVERSE: " + SONAR_DISTANCE);//updateDisplay(); 
                    Thread.Sleep(500);
                } // while (sonarDistance < TURN_DISTANCE)

                motorSpeed(STOP_SPEED);         // stop backing up
                
                return;
            } // end of IF TOO CLOSE
             */
        }

        #endregion

        /// <summary>
        /// Linearly scales standard 0 to 3.3V input.
        /// </summary>
        /// <param name="analogOutput">Analog input reading to convert.</param>
        /// <param name="scaleMin">Minimum scale reading.</param>
        /// <param name="scaleMax">Maximum scale reading.</param>
        static int analogLinearScale(double analogOutput, double scaleMin, double scaleMax)
        {
            int test = ((int)(analogOutput * ((scaleMax - scaleMin) / 3.3)));
            return ((int)(analogOutput * ((scaleMax - scaleMin) / 3.3)));
        }

        /// <summary>
        /// Scale an input linear scale to an output linear scale
        /// </summary>
        /// <param name="inputData"></param>
        /// <param name="inputMin"></param>
        /// <param name="inputMax"></param>
        /// <param name="outputMin"></param>
        /// <param name="outputMax"></param>
        /// <returns></returns>
        static public int convertLinearScale(double inputData, double inputMin, double inputMax, double outputMin, double outputMax)
        {
            if (inputData > inputMax)
                inputData = inputMax;

            if (inputData < inputMin)
                inputData = inputMin;

            double outputData = (((inputData - inputMin) / (inputMax - inputMin)) * (outputMax - outputMin)) + outputMin;
            //int test = (int)outputData;
            return (int)outputData;
        }

        /// <summary>
        /// Convert a byte to its ascii 2 character representation.
        /// </summary>
        /// <param name="data">Byte to convert.</param>
        /// <remarks>Example: if data = 255, FF will be returned in a character array.</remarks>
        static char[] byteToHex(byte data)
        {
            byte[] nib = new byte[] { 0, 0 };

            nib[1] = (byte)(data & 0x0F);        //LSB
            nib[0] = (byte)((data & 0xF0) / 16); //MSB

            for (var i = 0; i < nib.Length; i++)
            {
                if (nib[i] <= 9)
                {
                    nib[i] = (byte)(nib[i] + '0');
                }
                else
                {
                    nib[i] = (byte)(nib[i] + 55);
                }
            }

            return (Encoding.UTF8.GetChars(nib));

            //return new string(Encoding.UTF8.GetChars(nib));
        }

        /// <summary>
        /// Calculate a checksum for given byte array.
        /// </summary>
        /// <param name="data">Byte array used to calculate checksum.</param>
        /// <remarks>Checksum includes all bytes in array beteen '$ and '*'.
        /// This function returns XOR checksum of data
        /// XOR is the odd function, high when odd inputs</remarks>
        static byte getChecksum(byte[] data)
        {
            int i = 2;
            byte result = 0;

            result = data[1];

            while ((data[i] != 0x2A) && (i < data.Length))
            {
                result = (byte)(result ^ data[i]);
                i++;
            }

            return result;
        }


        /// <summary>
        /// Write lines to a file.
        /// </summary>
        /// <param name="fileName">File to read.</param>
        /// <param name="data">Lines to write to the file.</param>
        static bool WriteFileLines(string fileName, string[] lines)
        {
            //bool cardMounted = false;
            StreamWriter FileHandle = null;

            try
            {
                if (!roverSDCARD.IsCardMounted)
                {
                    Debug.Print("Card not mounted!");
                    //roverSDCARD.Mount();
                    // Thread.Sleep(100);
                    return false;
                }

                //cardMounted = Program.roverSDCARD.Mount();

                //if (!cardMounted) return;// if card not mounted

                //SD_ROOT_DIRECTORY = VolumeInfo.GetVolumes()[0].RootDirectory;

                //Debug.Print("Looking for file " + fileName + " and deleting it if exists.");
                //if (File.Exists(SD_ROOT_DIRECTORY + @"\" + fileName))
                //    File.Delete(SD_ROOT_DIRECTORY + @"\" + fileName);
                //else
                //    File.OpenWrite(SD_ROOT_DIRECTORY + @"\" + fileName);

                Debug.Print("Writing file..." + fileName);
                FileHandle = new StreamWriter(SD_ROOT_DIRECTORY + @"\" + fileName, true);

                //write file
                foreach (string line in lines)
                {
                    if (line != string.Empty)
                        FileHandle.WriteLine(line);
                }
                FileHandle.Close();
                //roverSDCARD.Unmount();
            }
            catch (Exception)
            {
                Debug.Print("Error: No File Handle.");
                return false;
            }

            // Flush everything to make sure we are starting fresh.
            //ps.UnmountFileSystem();
            Thread.Sleep(10);
            return true;

            //ps.Dispose();
        }

        /// <summary>
        /// Reads a given number of lines from a file.
        /// </summary>
        /// <param name="fileName">File to read.</param>
        /// <param name="numLines">Number of lines to read from the file.</param>
        static public string[] ReadFileLines(string fileName, int numLines)
        {
            string[] buffer = new string[numLines];
            //ps.MountFileSystem();

            try
            {
                if (!roverSDCARD.IsCardMounted)
                {
                    Debug.Print("Card not mounted!");
                    //roverSDCARD.Mount();
                    // Thread.Sleep(100);
                    return null;
                }

                StreamReader FileHandle = null;

                Debug.Print("Looking if " + fileName + " exists.");

                if (!File.Exists(SD_ROOT_DIRECTORY + "\\" + fileName))
                    return null;

                FileHandle = new StreamReader(SD_ROOT_DIRECTORY + "\\" + fileName);

                Debug.Print("Reading File Lines..." + fileName);
                for (int i = 0; i < buffer.Length; i++)
                {
                    buffer[i] = FileHandle.ReadLine();
                    if (buffer[i] == "")
                        break;
                }

                FileHandle.Close();

                Thread.Sleep(500);
                //ps.UnmountFileSystem();
                //ps.Dispose();

            }
            catch (Exception)
            {
                Debug.Print("Could not read SD card file!");
                return null;
            }

            return buffer;
        }

        static public void filter()
        {

        }

        /*
         byte[] tempArray = new byte[25];
         string testString = "Hi from port ";
            testPort = (testPort+1) % 4;

            switch (testPort)
            {
                case 0:
                    muxS0.Write(false);
                    muxS1.Write(false);
                    multiComPort.Write(Encoding.UTF8.GetBytes (testString + testPort.ToString() + "!\n\r"), 0, testString.Length + testPort.ToString().Length + 3);
                    break;

                case 1:
                    muxS0.Write(true);
                    muxS1.Write(false);
                    multiComPort.Write(Encoding.UTF8.GetBytes(testString + testPort.ToString() + "!\n\r"), 0, testString.Length + testPort.ToString().Length + 3);
                    break;

                case 2:
                    muxS0.Write(false);
                    muxS1.Write(true);
                    multiComPort.Write(Encoding.UTF8.GetBytes(testString + testPort.ToString() + "!\n\r"), 0, testString.Length + testPort.ToString().Length + 3);
                    break;

                case 3:
                    muxS0.Write(true);
                    muxS1.Write(true);
                    multiComPort.Write(Encoding.UTF8.GetBytes(testString + testPort.ToString() + "!\n\r"), 0, testString.Length + testPort.ToString().Length + 3);

                    if (multiComPort.BytesToRead > 0)
                    {
                        multiComPort.Read(tempArray, 0, tempArray.Length);
                        //Debug.Print( new string(Encoding.UTF8.GetChars(tempArray)) );
                        multiComPort.Write(tempArray, 0, tempArray.Length);
                    }

                    break;
            } 
         
         */


    }
}
