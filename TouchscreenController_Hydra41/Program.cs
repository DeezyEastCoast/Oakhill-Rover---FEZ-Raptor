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

        enum DRIVE_MODE { MANUAL, ASSIST, AUTO };
        static DRIVE_MODE SYSTEM_MODE = DRIVE_MODE.MANUAL;

        static GT.Timer ManualRoverTimer;
        static GT.Timer AutonomousRoverTimer;
        static GT.Timer ReportRoverDataTimer;

        static GT.Timer GroundEfxTimer; //Timer to run ground efx show

        
        static bool DEBUG_LED_STATE = true;
        static bool READCOMPLETE_IMU = false;

        static int MA_RANGE_INDEX = 0;
        static int MAX_RANGE = 100;  //max range in inches to care about
        static int MA_RANGE_PERIOD = 5;
        static double[] MA_RANGE_ARRAY = new double[MA_RANGE_PERIOD];
        static int sonarDistance = 0;
        //static int eraseMe = 0;

        static int picCounter = 0;
        //static bool roverHeadLights = false;
        //static bool roverTakePicture = false;

        static GT.SocketInterfaces.AnalogInput roverDistanceSensor;
        static GT.SocketInterfaces.AnalogInput roverBatteryVoltage;
        static GT.SocketInterfaces.AnalogInput roverBatteryCurrent;

        static SerialPort motor_steering_ComPort;
        static OHS_UartMux muxComPort;
        //static EMIC2 roverVoice;
        static MTK3339 roverGPS;

        #region AUTONOMOUS NAVIGATION
        //Compass Navigation
        static int targetHeading;  // where we want to go to reach current waypoint
        static int currentHeading; // where we are actually facing now
        static int headingError;  // signed (+/-) difference between targetHeading and currentHeading
        static readonly int HEADING_TOLERANCE = 5;  // tolerance +/- (in degrees) within which we don't attempt to turn to intercept targetHeading

        //GPS Navigation
        static WAYPOINT homeWaypoint;
        static WAYPOINT startingWaypoint;
        static WAYPOINT currentWaypoint;
        static WAYPOINT targetWaypoint;
        static int distanceToTarget;  // current distance to target (current waypoint)
        static int originalDistanceToTarget;  // distance to original waypoing when we started navigating to it

        //Waypoints
        struct WAYPOINT
        {
            //gps coords must be in decimal degrees
            WAYPOINT(string inLat, string inLon, sbyte inNum = -1)
            {
                lat = inLat;
                lon = inLon;
                number = inNum;
            }

            public string lat, lon;

            public double dLatitude
            {
                get
                {
                    return Convert.ToDouble(lat);
                }
                set
                {
                    dLatitude = value;
                }
            }

            public double dLongitude
            {
                get
                {
                    return Convert.ToDouble(lon);
                }
                set
                {
                    dLongitude = value;
                }
            }

            public int number;
        }

        static readonly int WAYPOINT_DISTANCE_TOLERANCE = 5; // tolerance in meters to waypoint; once within this tolerance, will advance to the next waypoint
        static readonly int NUMBER_WAYPOINTS = 5; // enter the numebr of way points here (will run from 0 to (n-1))
        static int waypointNumber = -1;  // current waypoint number; will run from 0 to (NUMBER_WAYPOINTS -1); start at -1 and gets initialized during setup()
        static WAYPOINT[] waypointList = new WAYPOINT[NUMBER_WAYPOINTS];

        //Steering/Turning
        enum TURN { LEFT = 100, RIGHT = 200, STRAIGHT= 128 };
        static TURN turnDirection = TURN.STRAIGHT;

        //Object avoidance distances (inches)
        static byte SAFE_DISTANCE = 70;
        static byte TURN_DISTANCE = 40;
        static byte STOP_DISTANCE = 12;

        //Speeds 
        static long motorTimeout = 0;
        static byte STOP_SPEED = 135;
        static byte FAST_SPEED = 200;
        static byte NORMAL_SPEED = 170;
        static byte TURN_SPEED = 150;
        static byte SLOW_SPEED = 150;
        static byte BACK_SPEED = 70;
        static byte CURRENT_NAV_SPEED = NORMAL_SPEED;

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

        static Unified IMU_Adafruit = Unified.Instance;

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

            //pin 3 port 13 is ultrasonic distance sensor Vcc/512 per inch --> 6.4mV per inch
            //pin 6 port 13 is ultrasonic distance sensor RX pin, High = enabled, Low = Disabled
            //pin 4 port 13 is battery voltage monitor  63.69 mV/Volt
            //pin 5 port 13 is battery current monitor  36.60 mV/Volt
            //pin 8 port 13 is unconnected but wired to 2X3 connector
            roverBatteryCurrent = extender_Battery_Input.CreateAnalogInput(GT.Socket.Pin.Five);
            roverBatteryVoltage = extender_Battery_Input.CreateAnalogInput(GT.Socket.Pin.Four);
            roverDistanceSensor = extender_Battery_Input.CreateAnalogInput(GT.Socket.Pin.Three);

            roverLoadBrd = loadBrd;

            try
            {
                sdCard.Mount();
                SD_ROOT_DIRECTORY = VolumeInfo.GetVolumes()[0].RootDirectory;
                STATUS_DEVICES += 1;
            }
            catch (Exception)
            {
                picCounter = -1;  
            }

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
            lairdComPort.Open();

            lairdReset = new Microsoft.SPOT.Hardware.OutputPort(lairdResetPin, true);
            lairdCts = new Microsoft.SPOT.Hardware.OutputPort(lairdCtsPin, false);
            //lairdRts = new Microsoft.SPOT.Hardware.InputPort(lairdRtsPin, false, Microsoft.SPOT.Hardware.Port.ResistorMode.PullUp);

            lairdWirelessBuffer = new SerialBuffer(72);

            #endregion

            #region MOTOR & SERVO COM PORT

            string steering_motor_ComName = GT.Socket.GetSocket(12, true, null, null).SerialPortName;
            motor_steering_ComPort = new SerialPort(steering_motor_ComName, 9600);
            motor_steering_ComPort.Open();

            #endregion

            #region Adafruit10DOF Setup
            //IMU_Adafruit.ReadCompleted += (t) =>
            //    {
            //        //Debug.Print("Current Temperature: " + IMU_Adafruit.Bmp180.Temperature + " \u00B0C");
            //        Debug.Print("Current Temperature: " + ((IMU_Adafruit.Bmp180.Temperature * 1.8000) + 32) + " \u00B0F");
            //        Debug.Print("Accelerometer X: " + IMU_Adafruit.Accelerometer.X + " Y: " + IMU_Adafruit.Accelerometer.Y + " Z: " + IMU_Adafruit.Accelerometer.Z);
            //        Debug.Print("Magnetometer X: " + IMU_Adafruit.Magnetometer.X + " Y: " + IMU_Adafruit.Magnetometer.Y + " Z: " + IMU_Adafruit.Magnetometer.Z);
            //        Debug.Print("Gyroscope X: " + IMU_Adafruit.Gyroscope.X + " Y: " + IMU_Adafruit.Gyroscope.Y + " Z: " + IMU_Adafruit.Gyroscope.Z);
            //        Debug.Print("Heading: " + IMU_Adafruit.Heading);
            //    };
            IMU_Adafruit.ReadCompleted += (t) =>
                {
                    IMU_Adafruit.ContinuousRead = false;
                    READCOMPLETE_IMU = true;
                    
                };
            IMU_Adafruit.ContinuousRead = true;
            IMU_Adafruit.BeginAsync();

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
            //GPS.CoordinatesUpdated += GPS_CoordinatesUpdated;

            //void GPS_CoordinatesUpdated(MTK3339 sender)
            //{
            //    Debug.Print("Coordinates: " + roverGPS.Latitude + ", " + roverGPS.Longitude + " (" + roverGPS.MapLatitude + ", " + roverGPS.MapLongitude + ")");
            //}

            #endregion

            #region TIMERS

            ManualRoverTimer = new GT.Timer(50);
            ManualRoverTimer.Tick += new GT.Timer.TickEventHandler(ManualRoverTimer_Tick);
            ManualRoverTimer.Start();

            AutonomousRoverTimer = new GT.Timer(100);
            AutonomousRoverTimer.Tick += new GT.Timer.TickEventHandler(AutonomousRoverTimer_Tick);
            //AutonomousRoverTimer.Start();

            ReportRoverDataTimer = new GT.Timer(500);
            ReportRoverDataTimer.Tick += new GT.Timer.TickEventHandler(ReportRoverDataTimer_Tick);
            ReportRoverDataTimer.Start();

            GroundEfxTimer = new GT.Timer(500);
            GroundEfxTimer.Tick += new GT.Timer.TickEventHandler(GroundEfxTimer_Tick);
            //GroundEfxTimer.Start();

            #endregion

            //display_T35.SimpleGraphics.DisplayImage(SD_ROOT_DIRECTORY + @"\roverPic17.jpg", Bitmap.BitmapImageType.Jpeg, 0, 0);
        }



        #region TIMERS

        /// <summary>
        /// Timer handler to manually driver rover. Get commands from joystick.
        /// </summary>
        static void ManualRoverTimer_Tick(GT.Timer timer)
        {
            //start_time = DateTime.Now;
            motorTimeout++;

            lairdWirelessBuffer.LoadSerial(lairdComPort);
            if ((ps2DataLine = lairdWirelessBuffer.ReadLine()) != null)
            {
                try
                {
                     //verify checksum
                    if ((byte)(Convert.ToInt32(ps2DataLine.Substring(ps2DataLine.LastIndexOf("*") + 1, 2), 16)) == getChecksum(Encoding.UTF8.GetBytes(ps2DataLine)))
                    {
                        switch (ps2DataLine.Substring(0, 4))
                        {
                            case "$OJC": //Oakhill Joysick Control

                                servoAngle((byte)(Convert.ToInt32(ps2DataLine.Substring(5, 2), 16)));
                                motorSpeed((byte)(Convert.ToInt32(ps2DataLine.Substring(8, 2), 16)));

                                motorTimeout = 0; //reset command timeout

                                DEBUG_LED_STATE = !DEBUG_LED_STATE;
                                //led on shows connected
                                Mainboard.SetDebugLED(DEBUG_LED_STATE);
                                //'$','O','J','C',','  
                                //   ,'0','0',','  //bytes 5,6    get steering
                                //   ,'0','0',','  //bytes 8,9:   get motor speed
                                //   ,'0','0',     //bytes 11,12  get joystick clicks/expansion
                                //   ,'*'
                                //   ,'0','0'
                                //   ,0x0D,0x0A};

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

                       
                    }

                    ////verify checksum
                    //if ((byte)(Convert.ToInt32(ps2DataLine.Substring(20, 2), 16)) == getChecksum(Encoding.UTF8.GetBytes(ps2DataLine)))
                    //{
                    //    //if((joystickCounter % 30) == 0) setRoverIO(Convert.ToInt32(ps2DataLine.Substring(1, 2) + ps2DataLine.Substring(4, 2), 16));
                    //    servoAngle((byte)(Convert.ToInt32(ps2DataLine.Substring(7, 2), 16)));
                    //    motorSpeed(Convert.ToInt32(ps2DataLine.Substring(16, 2), 16));

                    //    //char ps2Data[] = {'$'
                    //    //  ,'0','0',',' //bytes 1,2    get sensors and accessories commands byte 1
                    //    //  ,'0','0',',' //bytes 4,5:   get sensors and accessories commands byte 2
                    //    //  ,'0','0',',' //bytes 7,8:   right joystick
                    //    //  ,'0','0',',' //bytes 10,11
                    //    //  ,'0','0',',' //bytes 13,14
                    //    //  ,'0','0',',' //bytes 16,17: left joystick
                    //    //  ,'*'
                    //    //  ,'0','0'
                    //    //  ,0x0D,0x0A};

                    //    ////reset command timeout
                    //    motorTimeout = 0;
                    //    Mainboard.SetDebugLED(true);
                    //}

                }
                catch (Exception)
                { }
            }

            //if timeout has occurred
            if (motorTimeout > 75)
            {
                Mainboard.SetDebugLED(false);

                //stop motor
                motorSpeed(STOP_SPEED);

                motorTimeout = 76;
            }

            //end_time = DateTime.Now;
            //ts = end_time - start_time;
            //Debug.Print("Total time: " + ts.ToString());
            //Debug.Print("Total time in seconds: " + ((float)ts.Ticks / TimeSpan.TicksPerSecond).ToString() + " seconds");
        }

        static void AutonomousRoverTimer_Tick(GT.Timer timer)
        {
            //check kill switch

            //get current GPS position --------- PUT A VALID LOCK/FIX CHECK ON GPS
            currentWaypoint.dLatitude = roverGPS.MapLatitude;
            currentWaypoint.dLongitude = roverGPS.MapLongitude;
            // Process GPS -- update the course and distance to waypoint based on our new position
            distanceToWaypoint();
            courseToWaypoint();   

            //check sonar distance
            sonarDistance = getSonarRangeAvg();

            //get current compass heading
            if (READCOMPLETE_IMU)
            {
                currentHeading = (int)IMU_Adafruit.Heading;
                calcDesiredTurn();

                READCOMPLETE_IMU = false;

                IMU_Adafruit.ContinuousRead = true;
                IMU_Adafruit.BeginAsync();
            }

            //output oakhill rover autonomous
            roverData = "$ORA," +
                        getSonarRangeAvg() + " in"
                        + "," + IMU_Adafruit.Heading + " deg"
                        + "," + roverGPS.MapLatitude
                        + "," + roverGPS.MapLongitude
                //+ "," + (1.0 - ((IMU_Adafruit.Bmp180.Pressure/101910)^.19)) //convert pressure to altitude in meters, 1019.1hPa as sealevel in Pawtucket
                        + "*";
            roverData += (new string(byteToHex(getChecksum(Encoding.UTF8.GetBytes(roverData)))) + "\r\n"); //add checksum, cr and lf

            //Debug.Print(roverData);
            lairdComPort.Write(Encoding.UTF8.GetBytes(roverData), 0, roverData.Length);

        }

        /// <summary>
        /// Timer handler to report Rover data to controller.
        /// </summary>
        void ReportRoverDataTimer_Tick(GT.Timer timer)
        {
            if (READCOMPLETE_IMU)
            {
                //output oakhill rover data
                roverData = "$ORD," +
                                    getBatteryVoltage() + " V"
                            + "," + getBatteryCurrent() + " A"
                            + "," + getSonarRangeAvg() + " in"
                            + "," + IMU_Adafruit.Heading + " deg"
                            + "," + ((IMU_Adafruit.Bmp180.Temperature * 1.8000) + 32).ToString().Substring(0, 4) + " F"
                            + "," + (IMU_Adafruit.Bmp180.Pressure / 6895).ToString().Substring(0, 4) + " psi"
                            + "," + roverGPS.Latitude
                            + "," + roverGPS.Longitude
                            //+ "," + (1.0 - ((IMU_Adafruit.Bmp180.Pressure/101910)^.19)) //convert pressure to altitude in meters, 1019.1hPa as sealevel in Pawtucket
                            + "*";
                roverData += (new string(byteToHex(getChecksum(Encoding.UTF8.GetBytes(roverData)))) + "\r\n"); //add checksum, cr and lf
                READCOMPLETE_IMU = false;

                //Debug.Print(roverData);
                lairdComPort.Write(Encoding.UTF8.GetBytes(roverData), 0, roverData.Length);

                IMU_Adafruit.ContinuousRead = true;
                IMU_Adafruit.BeginAsync();
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
        }

        /// <summary>
        /// Set the motor speed.
        /// </summary>
        /// <param name="speed">Relative motor speed. (0 to 255, 75 = center & null)</param>
        static void motorSpeed(byte speedCmd)
        {
            //0x00 is full rev on controller trigger, about 75 is center, 0xFF (255) is full forward on trigger
            //about 139 is start of forward on truck, 255 is max forward speed.
            //on landRover  0 to 134 is reverse, 135 = stop, 136 to 255 is forward.
            //about 139 is start of forward on truck

            //if ((speedCmd > 70) && (speedCmd < 90)) speedCmd = MOTORSTOP; //deadband around neutral
            //else if (speedCmd >= 90) speedCmd = (byte)convertLinearScale(speedCmd, 90, 255, 136, 255);
            //else speedCmd = (byte)convertLinearScale(speedCmd, 0, 70, 0, 134);

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

            MA_RANGE_INDEX = (MA_RANGE_INDEX+1) % MA_RANGE_PERIOD;

            //Debug.Print("MA Range: " + MA_RANGE.ToString() + " inches\n\r");

            return (int)MA_RANGE;
        }
        
        #region NAVIGATION METHODS
        static void calcDesiredTurn()
        {
            // calculate where we need to turn to head to destination
            headingError = targetHeading - currentHeading;

            // adjust for compass wrap
            if (headingError < -180)
                headingError += 360;
            if (headingError > 180)
                headingError -= 360;

            // calculate which way to turn to intercept the targetHeading
            if (System.Math.Abs(headingError) <= HEADING_TOLERANCE)      // if within tolerance, don't turn
                turnDirection = TURN.STRAIGHT;
            else if (headingError < 0)
                turnDirection = TURN.LEFT;
            else if (headingError > 0)
                turnDirection = TURN.RIGHT;
            else
                turnDirection = TURN.STRAIGHT;
        }

        /// <summary>
        /// returns distance in meters between two positions, both specified 
        // as signed decimal-degrees latitude and longitude. Uses great-circle 
        // distance computation for hypothetical sphere of radius 6372795 meters.
        // Because Earth is no exact sphere, rounding errors may be up to 0.5%.
        // copied from TinyGPS library
        /// </summary>
        /// <returns>Distance in meters between two GPS positions</returns>
        static int distanceToWaypoint()
        {
            double delta = ((System.Math.PI/180)*(currentWaypoint.dLongitude - targetWaypoint.dLongitude)); //convert the angular difference to radians
            double sdlong = System.Math.Sin(delta);
            double cdlong = System.Math.Cos(delta);
            double lat1 = (System.Math.PI / 180) * (currentWaypoint.dLatitude); //convert to radians
            double lat2 = (System.Math.PI / 180) * (targetWaypoint.dLatitude); //convert to radians
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

        static void nextWaypoint()
        {
            waypointNumber++;
            targetWaypoint.dLatitude = waypointList[waypointNumber].dLatitude;
            targetWaypoint.dLongitude = waypointList[waypointNumber].dLongitude;

            if ((targetWaypoint.dLatitude == 0 && targetWaypoint.dLongitude == 0) || waypointNumber >= NUMBER_WAYPOINTS)    // last waypoint reached? 
            {
                //end mission
                //stop motor and neutral steering
                motorSpeed(STOP_SPEED);
                servoAngle((byte)TURN.STRAIGHT);

                //send mission complete msg to controller
            }

            // update the course and distance to waypoint based on our new position
            distanceToWaypoint();
            courseToWaypoint();
            distanceToTarget = originalDistanceToTarget = distanceToWaypoint();
            courseToWaypoint();

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
            double dlon = ((System.Math.PI / 180) * (targetWaypoint.dLongitude - currentWaypoint.dLongitude));
            double cLat = (System.Math.PI / 180) * (currentWaypoint.dLatitude);
            double tLat = (System.Math.PI / 180) * (targetWaypoint.dLatitude);
            double a1 = System.Math.Sin(dlon) * System.Math.Cos(tLat);
            double a2 = System.Math.Sin(cLat) * System.Math.Cos(tLat) * System.Math.Cos(dlon);
            a2 = System.Math.Cos(cLat) * System.Math.Sin(tLat) - a2;
            a2 = System.Math.Atan2(a1, a2);
            if (a2 < 0.0)
            {
                a2 += (2*System.Math.PI);
            }
            targetHeading = (int)((a2*180)/System.Math.PI);
            return targetHeading;
        }   

        /// <summary>
        /// Drive and avoid obstacles if necessary.
        /// </summary>
        static void moveAndAvoid()
        {
            if (sonarDistance >= SAFE_DISTANCE)       // no close objects in front of car
            {
                if (turnDirection == TURN.STRAIGHT)
                    CURRENT_NAV_SPEED = FAST_SPEED;
                else
                    CURRENT_NAV_SPEED = TURN_SPEED;

                servoAngle((byte)turnDirection);
                motorSpeed(CURRENT_NAV_SPEED);

                return;
            }

            if (sonarDistance > TURN_DISTANCE && sonarDistance < SAFE_DISTANCE)    // not yet time to turn, but slow down
            {
                if (turnDirection == TURN.STRAIGHT)
                    CURRENT_NAV_SPEED = NORMAL_SPEED;
                else
                {
                    CURRENT_NAV_SPEED = TURN_SPEED;
                    servoAngle((byte)turnDirection);     // alraedy turning to navigate
                }
                motorSpeed(CURRENT_NAV_SPEED);
                return;
            }

            if (sonarDistance < TURN_DISTANCE && sonarDistance > STOP_DISTANCE)  // getting close, time to turn to avoid object        
            {
                CURRENT_NAV_SPEED = SLOW_SPEED;
                motorSpeed(CURRENT_NAV_SPEED);      // slow down

                switch (turnDirection)
                {
                    case TURN.STRAIGHT:                  // going straight currently, so start new turn
                        {
                            if (headingError <= 0)
                                turnDirection = TURN.LEFT;
                            else
                                turnDirection = TURN.RIGHT;
                            servoAngle((byte)turnDirection);  // turn in the new direction
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


            if (sonarDistance < STOP_DISTANCE)          // too close, stop and back up
            {
                motorSpeed(STOP_SPEED);             // stop 
                servoAngle((byte)TURN.STRAIGHT);             // straighten up
                turnDirection = TURN.STRAIGHT;
                motorSpeed(BACK_SPEED);  // go back at higher speed

                while (sonarDistance < TURN_DISTANCE)       // backup until we get safe clearance
                {
                    //get current GPS position --------- PUT A VALID LOCK/FIX CHECK ON GPS
                    currentWaypoint.lat = roverGPS.Latitude;
                    currentWaypoint.lon = roverGPS.Longitude;
                    // Process GPS -- update the course and distance to waypoint based on our new position
                    distanceToWaypoint();
                    courseToWaypoint(); 

                    //get current compass heading
                    if (READCOMPLETE_IMU)
                    {
                        currentHeading = (int)IMU_Adafruit.Heading;
                        calcDesiredTurn();  // calculate how we would optimatally turn, without regard to obstacles   

                        READCOMPLETE_IMU = false;

                        IMU_Adafruit.ContinuousRead = true;
                        IMU_Adafruit.BeginAsync();
                    }
                
                    //check sonar distance
                    sonarDistance = getSonarRangeAvg();
                    //updateDisplay();
                    Thread.Sleep(100);
                } // while (sonarDistance < TURN_DISTANCE)
                motorSpeed(STOP_SPEED);         // stop backing up
                return;
            } // end of IF TOO CLOSE

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
