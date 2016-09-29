/*
Copyright 2010 Thomas W. Holtquist
www.skewworks.com
 * 
Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/

using System;
using System.IO.Ports;
using System.Text;

using Microsoft.SPOT;

namespace Skewworks.Drivers.GPS
{

    #region Event Delegates

    public delegate void OnAltitudeChanged(MTK3339 sender, double meters, int feet);
    public delegate void OnCoordinatesUpdated(MTK3339 sender);
    public delegate void OnCourseChanged(MTK3339 sender, double degrees);
    public delegate void OnError(MTK3339 sender, string data);
    public delegate void OnFixModeChanged(MTK3339 sender, MTK3339.FixModes e);
    public delegate void OnFixTypeChanged(MTK3339 sender, MTK3339.FixTypes e);
    public delegate void OnGSAModeChanged(MTK3339 sender, MTK3339.GSAModes e);
    public delegate void OnRMCModeChanged(MTK3339 sender, MTK3339.RMCModes e);
    public delegate void OnSatellitesInViewChanged(MTK3339 sender, MTK3339.SatelliteInView[] e);
    public delegate void OnSatellitesUsedChanged(MTK3339 sender, int count);
    public delegate void OnSpeedChanged(MTK3339 sender, double knots, double mph);
    public delegate void OnTimeChanged(MTK3339 sender, int hour, int minute, int second);
    public delegate void OnDateChanged(MTK3339 sender, int month, int day, int year);
    public delegate void OnVTGModeChanged(MTK3339 sender, MTK3339.VTGModes e);

    #endregion

    public class MTK3339
    {

        #region Constants

        private const string UPDATE_RATE_1HZ = "$PMTK220,1000*1F\r\n";
        private const string UPDATE_RATE_5HZ = "$PMTK220,200*2C\r\n";
        private const string UPDATE_RATE_10HZ = "$PMTK220,100*2F\r\n";

        private const string SUBSCRIBE_RMCONLY = "$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29\r\n";
        private const string SUBSCRIBE_RMCGGA = "$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n";
        private const string SUBSCRIBE_ALLDATA = "$PMTK314,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n";
        private const string SUBSCRIBE_OUTOFF =  "$PMTK314,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n";

        #endregion

        #region Enumerations

        public enum FixModes
        {
            Manual = 0,
            Automatic = 1,
        }

        public enum FixTypes
        {
            NoFix = 0,
            GPSFix = 1,
            DiffGPS = 2,
        }

        public enum GSAModes
        {
            NoFix = 0,
            Fix2D = 1,
            Fix3D = 2,
        }

        public enum RMCModes
        {
            Autonomous = 0,
            Differential = 1,
            Estimated = 2,
        }

        public enum SubscriptionLevels
        {
            AllData = 0,
            RMCGGA = 1,
            RMCOnly = 2,
            OutputOff = 3,
        }

        public enum UpdateRates
        {
            GPS_1HZ = 0,
            GPS_5HZ = 1,
            GPS_10HZ = 2,
        }

        public enum VTGModes
        {
            Autonomous = 0,
            Differential = 1,
            Estimated = 2,
        }

        #endregion

        #region Structures

        public struct SatelliteInView
        {
            public int PRNNumber;
            public int Elevation;
            public int Azimuth;
            public int SignalNoiseRatio;
            public SatelliteInView(int PRNNumber, int Elevation, int Azimuth, int SignalNoiseRatio)
            {
                this.PRNNumber = PRNNumber;
                this.Elevation = Elevation;
                this.Azimuth = Azimuth;
                this.SignalNoiseRatio = SignalNoiseRatio;
            }
        }

        #endregion

        #region Variables

        private SerialPort _gpsSP;
        private UpdateRates _rate;
        private SubscriptionLevels _subLevel;

        private string _dataBuffer;

        private string _latitude;
        private double _mapLatitude;

        private string _longitude;
        private double _mapLongitude;

        private int _timezoneOffset;
        private bool _dstEnable;

        private FixTypes _fixType;
        private int _satsUsed;

        private double _HOD;
        private double _POD;
        private double _VOD;

        private double _altitude;
        private double _geoSep;
        private int _diffAge;

        private double _speed;
        private double _course;
        private RMCModes _rmcMode;

        private FixModes _fixMode;
        private GSAModes _gsaMode;
        private int[] _gsaSatellites;

        private SatelliteInView[] _sv;
        private SatelliteInView[] _svTmp;
        private int _svCount;

        private VTGModes _vtgMode;

        #endregion

        #region Constructors

        public MTK3339(string COMPort, SubscriptionLevels SubscriptionLevel, UpdateRates UpdateRate = UpdateRates.GPS_1HZ)
        {
            // Create COM Port
            _gpsSP = new SerialPort(COMPort, 9600);
            _gpsSP.Open();
            _gpsSP.DataReceived += new SerialDataReceivedEventHandler(_gpsSP_DataReceived);

            // Set our Update Rate
            _rate = UpdateRate;
            SetUpdateRate();

            // Set our Subscription Level
            _subLevel = SubscriptionLevel;
            SetSubscriptionLevel();

            // Time value
            _timezoneOffset = -6;
            _dstEnable = true;
        }

        #endregion

        #region Events

        public event OnAltitudeChanged AltitudeChanged;
        protected virtual void OnAltitudeChanged(MTK3339 sender, double meters, int feet)
        {
            if (AltitudeChanged != null)
                AltitudeChanged(sender, meters, feet);
        }

        public event OnCoordinatesUpdated CoordinatesUpdated;
        protected virtual void OnCoordinatesUpdated(MTK3339 sender)
        {
            if (CoordinatesUpdated != null)
                CoordinatesUpdated(sender);
        }

        public event OnCourseChanged CourseChanged;
        protected virtual void OnCourseChanged(MTK3339 sender, double degrees)
        {
            if (CourseChanged != null)
                CourseChanged(sender, degrees);
        }

        public event OnDateChanged DateChanged;
        protected virtual void OnDateChanged(MTK3339 sender, int month, int day, int year)
        {
            if (DateChanged != null)
                DateChanged(sender, month, day, year);
        }

        public event OnError Error;
        protected virtual void OnError(MTK3339 sender, string data)
        {
            if (Error != null)
                Error(sender, data);
        }

        public event OnFixModeChanged FixModeChanged;
        protected virtual void OnFixModeChanged(MTK3339 sender, FixModes e)
        {
            if (FixModeChanged != null)
                FixModeChanged(sender, e);
        }

        public event OnFixTypeChanged FixTypeChanged;
        protected virtual void OnFixTypeChanged(MTK3339 sender, FixTypes e)
        {
            if (FixTypeChanged != null)
                FixTypeChanged(sender, e);
        }

        public event OnGSAModeChanged GSAModeChanged;
        protected virtual void OnGSAModeChanged(MTK3339 sender, GSAModes e)
        {
            if (GSAModeChanged != null)
                GSAModeChanged(sender, e);
        }

        public event OnRMCModeChanged RMCModeChanged;
        protected virtual void OnRMCModeChanged(MTK3339 sender, RMCModes e)
        {
            if (RMCModeChanged != null)
                RMCModeChanged(sender, e);
        }

        public event OnSatellitesInViewChanged SatellitesInViewChanged;
        protected virtual void OnSatellitesInViewChanged(MTK3339 sender, SatelliteInView[] e)
        {
            if (SatellitesInViewChanged != null)
                SatellitesInViewChanged(sender, e);
        }

        public event OnSatellitesUsedChanged SatellitesUsedChanged;
        protected virtual void OnSatellitesUsedChanged(MTK3339 sender, int count)
        {
            if (SatellitesUsedChanged != null)
                SatellitesUsedChanged(sender, count);
        }

        public event OnSpeedChanged SpeedChanged;
        protected virtual void OnSpeedChanged(MTK3339 sender, double knots, double mph)
        {
            if (SpeedChanged != null)
                SpeedChanged(sender, knots, mph);
        }

        public event OnTimeChanged TimeChanged;
        protected virtual void OnTimeChanged(MTK3339 sender, int hour, int minute, int second)
        {
            if (TimeChanged != null)
                TimeChanged(sender, hour, minute, second);
        }

        public event OnVTGModeChanged VTGModeChanged;
        protected virtual void OnVTGModeChanged(MTK3339 sender, VTGModes e)
        {
            if (VTGModeChanged != null)
                VTGModeChanged(sender, e);
        }

        #endregion

        #region Properties

        public int AgeOfDifferential
        {
            get { return _diffAge; }
        }

        public double Altitude
        {
            get { return _altitude; }
        }

        public int AltitudeInFeet
        {
            get { return (int)MetersToFeet(_altitude); }
        }

        public double Course
        {
            get { return _course; }
        }

        public bool DaylightSavingsTime
        {
            get { return _dstEnable; }
            set { _dstEnable = value; }
        }

        public bool FixAvailable
        {
            get { return _fixType != FixTypes.NoFix; }
        }

        public FixModes FixMode
        {
            get { return _fixMode; }
        }

        public FixTypes FixType
        {
            get { return _fixType; }
        }

        public GSAModes GSAMode
        {
            get { return _gsaMode; }
        }

        public int[] GSASatellites
        {
            get { return _gsaSatellites; }
        }

        public double GeoidalSeparation
        {
            get { return _geoSep; }
        }

        public double HorizontalDilution
        {
            get { return _HOD; }
        }

        public string Latitude
        {
            get { return _latitude; }
        }

        public string Longitude
        {
            get { return _longitude; }
        }

        public double MapLatitude
        {
            get { return _mapLatitude; }
        }

        public double MapLongitude
        {
            get { return _mapLongitude; }
        }

        public double PositionDilution
        {
            get { return _POD; }
        }

        public RMCModes RMCMode
        {
            get { return _rmcMode; }
        }

        public SatelliteInView[] SatellitesInView
        {
            get { return _sv; }
        }

        public int SatellitesUsed
        {
            get { return _satsUsed; }
        }

        public double Speed
        {
            get { return _speed; }
        }

        public double SpeedInMPH
        {
            get { return _speed * 1.151; }
        }

        public SubscriptionLevels SubscriptionLevel
        {
            get { return _subLevel; }
            set
            {
                if (_subLevel == value)
                    return;
                _subLevel = value;
                SetSubscriptionLevel();
            }
        }

        public int TimeZoneOffset
        {
            get { return _timezoneOffset; }
            set { _timezoneOffset = value; }
        }

        public UpdateRates UpdateRate
        {
            get { return _rate; }
            set
            {
                if (_rate == value)
                    return;
                _rate = value;
                SetUpdateRate();
            }
        }

        public double VerticalDilution
        {
            get { return _VOD; }
        }

        public VTGModes VTGMode
        {
            get { return _vtgMode; }
        }

        #endregion

        #region Private Methods

        /// <summary>
        /// Reads Serial data on DataReceived event
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void _gpsSP_DataReceived(object sender, SerialDataReceivedEventArgs e)
        {
            int read, pos;
            string tmp = string.Empty;
            string cmd;
            string[] data;

            try
            {
                byte[] b = new byte[_gpsSP.BytesToRead];

                // Check for a 0 byte buffer
                if (b.Length == 0)
                    return;

                // Read data
                read = _gpsSP.Read(b, 0, b.Length);

                // Convert to string (using any previously stored buffer)
                tmp = _dataBuffer + new string(UTF8Encoding.UTF8.GetChars(b));

                // Find lines
                while (true)
                {
                    // Check for end of data
                    if (tmp == string.Empty)
                        break;

                    // Look for next end of line
                    pos = tmp.IndexOf("\r\n");

                    if (pos < 0)
                    {
                        // None found, store in buffer
                        _dataBuffer = tmp;
                        break;
                    }
                    else
                    {
                        // Clear out buffer
                        _dataBuffer = null;

                        // Separate line
                        cmd = tmp.Substring(0, pos);

                        // Remove line
                        tmp = tmp.Substring(pos + 2);

                        // Remove Checksum
                        if (cmd.Substring(cmd.Length - 3, 1) == "*")
                            cmd = cmd.Substring(0, cmd.Length - 3);

                        // Split line
                        data = cmd.Split(',');

                        // Proccess command
                        switch (data[0])
                        {
                            case "$PMTK001":
                                // Just echoing, do nothing
                                break;
                            case "$GPGGA":
                                ParseGGAData(data);
                                break;
                            case "$GPGSA":
                                ParseGSAData(data);
                                break;
                            case "$GPRMC":
                                ParseRMCData(data);
                                break;
                            case "$GPVTG":
                                ParseVTGData(data);
                                break;
                            case "$GPGSV":
                                ParseGSVData(data);
                                break;
                        }
                    }
                }
            }
            catch (Exception)
            {
                Debug.Print("Parse error");
                OnError(this, tmp);
                _dataBuffer = null;
            }
        }

        private void GeneralParseTime(string UTC)
        {
            int h = int.Parse(UTC.Substring(0, 2));
            int m = int.Parse(UTC.Substring(2, 2));
            int s = int.Parse(UTC.Substring(4, 2));

            h += _timezoneOffset;

            if (_dstEnable)
                h += 1;

            if (h >= 24)
                h -= 24;
            else if (h < 0)
                h += 24;

            OnTimeChanged(this, h, m, s);
        }

        private void GeneralParseLatitude(string Value, string NSIndicator)
        {
            int pos = Value.IndexOf(".");
            double val;
            string tmp;

            _latitude = Value + NSIndicator;

            // Remove decimal point
            if (pos >= 0)
                Value = Value.Substring(0, pos) + Value.Substring(pos + 1);

            // Fix remainder
            tmp = (double.Parse(Value.Substring(2)) / 60).ToString();

            // Remove remainder decimal point
            pos = tmp.IndexOf(".");
            if (pos >= 0)
                tmp = tmp.Substring(0, pos) + tmp.Substring(pos + 1);

            // Join pieces
            val = double.Parse(Value.Substring(0, 2) + "." + tmp);

            // Put negative if South
            if (NSIndicator == "S")
                val = -val;

            _mapLatitude = val;
        }

        private void GeneralParseLongitude(string Value, string EWIndicator)
        {
            int pos = Value.IndexOf(".");
            double val;
            string tmp;

            _longitude = Value + EWIndicator;

            // Remove decimal point
            if (pos >= 0)
                Value = Value.Substring(0, pos) + Value.Substring(pos + 1);

            // Fix remainder
            tmp = (double.Parse(Value.Substring(3)) / 60).ToString();

            // Remove remainder decimal point
            pos = tmp.IndexOf(".");
            if (pos >= 0)
                tmp = tmp.Substring(0, pos) + tmp.Substring(pos + 1);

            // Join pieces
            val = double.Parse(Value.Substring(0, 3) + "." + tmp);

            // Put negative if South
            if (EWIndicator == "W")
                val = -val;

            _mapLongitude = val;
        }

        private double MetersToFeet(double Value)
        {
            return Value * 3.2808399;
        }

        private void ParseGGAData(string[] Data)
        {
            double lastLat = _mapLatitude;
            double lastLng = _mapLongitude;
            double dTmp;
            int iTmp;

            // UTC 
            GeneralParseTime(Data[1]);

            // Latitude
            if (Data[2] != string.Empty && Data[3] != string.Empty)
                GeneralParseLatitude(Data[2], Data[3]);

            // Longitude
            if (Data[4] != string.Empty && Data[5] != string.Empty)
                GeneralParseLongitude(Data[4], Data[5]);

            // Fix Indicator
            if (Data[6] != string.Empty)
            {
                if (_fixType != (FixTypes)int.Parse(Data[6]))
                {
                    _fixType = (FixTypes)int.Parse(Data[6]);
                    OnFixTypeChanged(this, _fixType);
                }
            }

            // Satellites Used
            if (Data[7] != string.Empty)
            {
                iTmp = int.Parse(Data[7]);
                if (_satsUsed != iTmp)
                {
                    _satsUsed = iTmp;
                    OnSatellitesUsedChanged(this, _satsUsed);
                }
            }

            // Horizontal Dilution of Precision (in meters)
            if (Data[8] != string.Empty)
                _HOD = double.Parse(Data[8]);

            // MSL Altitude (in meters)
            if (Data[9] != string.Empty)
            {
                dTmp = double.Parse(Data[9]);
                if (_altitude != dTmp)
                {
                    _altitude = dTmp;
                    OnAltitudeChanged(this, _altitude, (int)MetersToFeet(_altitude));
                }
            }

            // Item 10 is ALWAYS M for Meters; so skip it.

            // Geoidal Separation
            if (Data[11] != string.Empty)
                _geoSep = double.Parse(Data[11]);

            // Item 12 is ALWAYS M for Meters; so skip it.

            // Age of Differential Correction (in seconds)
            if (Data[13] != string.Empty)
                _diffAge = int.Parse(Data[13]);

            // Raise Events
            if (_fixType != FixTypes.NoFix && (lastLat != _mapLatitude || lastLng != _mapLongitude))
                OnCoordinatesUpdated(this);
        }

        private void ParseGSAData(string[] Data)
        {
            int SatCount = 0;
            int i;

            // Fix selection Manual/Automatic
            if (Data[1] == string.Empty || Data[1] == "M")
            {
                if (_fixMode != FixModes.Manual)
                {
                    _fixMode = FixModes.Manual;
                    OnFixModeChanged(this, _fixMode);
                }
            }
            else
            {
                if (_fixMode != FixModes.Automatic)
                {
                    _fixMode = FixModes.Automatic;
                    OnFixModeChanged(this, _fixMode);
                }
            }
            
            // GSAMode
            if (Data[2] == string.Empty || Data[2] == "1")
            {
                if (_gsaMode != GSAModes.NoFix)
                {
                    _gsaMode = GSAModes.NoFix;
                    OnGSAModeChanged(this, _gsaMode);
                }
            }
            else if (Data[2] == "2")
            {
                if (_gsaMode != GSAModes.Fix2D)
                {
                    _gsaMode = GSAModes.Fix2D;
                    OnGSAModeChanged(this, _gsaMode);
                }
            }
            else
            {
                if (_gsaMode != GSAModes.Fix3D)
                {
                    _gsaMode = GSAModes.Fix3D;
                    OnGSAModeChanged(this, _gsaMode);
                }
            }

            // Satellites used
            for (i = 3; i < 15; i++)
            {
                if (Data[i] != string.Empty)
                    SatCount += 1;
                else
                    break;
            }

            // Move to array
            _gsaSatellites = new int[SatCount];
            for (i = 0; i < SatCount; i++)
                _gsaSatellites[i] = int.Parse(Data[i + 3]);

            // Position Dilution of Precision
            if (Data[15] != string.Empty)
                _POD = double.Parse(Data[15]);

            // Horizontal Dilution of Precision
            if (Data[16] != string.Empty)
                _HOD = double.Parse(Data[16]);

            // Vertical Dilution of Precision
            if (Data[17] != string.Empty)
                _VOD = double.Parse(Data[17]);
        }

        private void ParseGSVData(string[] Data)
        {
            int iIndex;

            // Number of sentences
            if (Data[1] == string.Empty || Data[2] == string.Empty)
                return;
            _svCount = int.Parse(Data[1]);

            // Current sentence
            int iMsg = int.Parse(Data[2]);

            // Satellites in view
            if (iMsg == 1)
                _svTmp = new SatelliteInView[int.Parse(Data[3])];

            // Get Array Index
            iIndex = iMsg * 4;

            // Sat 1 Data
            if (Data[4] != string.Empty)
                _svTmp[iIndex++] = new SatelliteInView(int.Parse(Data[4]), int.Parse(Data[5]), int.Parse(Data[6]), int.Parse(Data[7]));

            // Sat 2 Data
            if (Data[8] != string.Empty)
                _svTmp[iIndex++] = new SatelliteInView(int.Parse(Data[8]), int.Parse(Data[9]), int.Parse(Data[10]), int.Parse(Data[11]));

            // Sat 3 Data
            if (Data[12] != string.Empty)
                _svTmp[iIndex++] = new SatelliteInView(int.Parse(Data[12]), int.Parse(Data[13]), int.Parse(Data[14]), int.Parse(Data[15]));

            // Sat 4 Data
            if (Data[16] != string.Empty)
                _svTmp[iIndex++] = new SatelliteInView(int.Parse(Data[16]), int.Parse(Data[17]), int.Parse(Data[18]), int.Parse(Data[19]));

            // Raise event
            if (iMsg == _svCount)
            {
                _sv = _svTmp;
                OnSatellitesInViewChanged(this, _sv);
            }
        }

        private void ParseRMCData(string[] Data)
        {
            double lastLat = _mapLatitude;
            double lastLng = _mapLongitude;
            double dTmp;

            // Check Status [2] first
            // Don't bother parsing invalid data
            if (Data[2] != "A")
                return;

            // UTC 
            GeneralParseTime(Data[1]);

            // Latitude
            if (Data[3] != string.Empty && Data[4] != string.Empty)
                GeneralParseLatitude(Data[3], Data[4]);

            // Longitude
            if (Data[5] != string.Empty && Data[6] != string.Empty)
                GeneralParseLongitude(Data[5], Data[6]);

            // Raise Lat/Lng Event
            if (lastLat != _mapLatitude || lastLng != _mapLongitude)
                OnCoordinatesUpdated(this);

            // Speed (in Knots)
            if (Data[7] != string.Empty)
            {
                dTmp = double.Parse(Data[7]);
                if (_speed != dTmp)
                {
                    _speed = dTmp;
                    OnSpeedChanged(this, _speed, _speed * 1.151);
                }
            }

            // Course (in degrees)
            if (Data[8] != string.Empty)
            {
                dTmp = double.Parse(Data[8]);
                if (_course != dTmp)
                {
                    _course = dTmp;
                    OnCourseChanged(this, _course);
                }
            }

            // Date
            if (Data[9] != string.Empty)
                OnDateChanged(this, int.Parse(Data[9].Substring(2, 2)), int.Parse(Data[9].Substring(0, 2)), 2000 + int.Parse(Data[9].Substring(4)));

            // Next 2 messages are only available w/ GlobalTop Customization Service
            // So ignore them

            // Mode
            if (Data[12] != string.Empty)
            {
                switch (Data[12])
                {
                    case "D":
                        if (_rmcMode != RMCModes.Differential)
                        {
                            _rmcMode = RMCModes.Differential;
                            OnRMCModeChanged(this, _rmcMode);
                        }
                        break;
                    case "E":
                        if (_rmcMode != RMCModes.Estimated)
                        {
                            _rmcMode = RMCModes.Estimated;
                            OnRMCModeChanged(this, _rmcMode);
                        }
                        break;
                    default:
                        if (_rmcMode != RMCModes.Autonomous)
                        {
                            _rmcMode = RMCModes.Autonomous;
                            OnRMCModeChanged(this, _rmcMode);
                        }
                        break;
                }
            }
        }

        private void ParseVTGData(string[] Data)
        {
            double dTmp;

            // Course (in degrees)
            if (Data[1] != string.Empty)
            {
                dTmp = double.Parse(Data[1]);
                if (_course != dTmp)
                {
                    _course = dTmp;
                    OnCourseChanged(this, _course);
                }
            }

            // Ignore next field (reference, always T)

            // Ignore next 2 fields (needs GlobalTop Customization Service)

            // Speed (in Knots)
            if (Data[5] != string.Empty)
            {
                dTmp = double.Parse(Data[5]);
                if (_speed != dTmp)
                {
                    _speed = dTmp;
                    OnSpeedChanged(this, _speed, _speed * 1.151);
                }
            }

            // Ignore Next field (reference, always N)

            // Ignore next 2 fields (same speed but in km/hr)

            // VTG Mode
            switch (Data[9])
            {
                case "D":   // Differential
                    if (_vtgMode != VTGModes.Differential)
                    {
                        _vtgMode = VTGModes.Differential;
                        OnVTGModeChanged(this, _vtgMode);
                    }
                    break;
                case "E":   // Estimated
                    if (_vtgMode != VTGModes.Estimated)
                    {
                        _vtgMode = VTGModes.Estimated;
                        OnVTGModeChanged(this, _vtgMode);
                    }
                    break;
                default:    // Autonomous
                    if (_vtgMode != VTGModes.Autonomous)
                    {
                        _vtgMode = VTGModes.Autonomous;
                        OnVTGModeChanged(this, _vtgMode);
                    }
                    break;
            }

        }

        /// <summary>
        /// Set the GPS subscription level
        /// </summary>
        private void SetSubscriptionLevel()
        {
            byte[] b;

            switch (_subLevel)
            {
                case SubscriptionLevels.RMCGGA:
                    b = UTF8Encoding.UTF8.GetBytes(SUBSCRIBE_RMCGGA);
                    break;
                case SubscriptionLevels.RMCOnly:
                    b = UTF8Encoding.UTF8.GetBytes(SUBSCRIBE_RMCONLY);
                    break;
                case SubscriptionLevels.OutputOff:
                    b = UTF8Encoding.UTF8.GetBytes(SUBSCRIBE_OUTOFF);
                    break;
                default: // All Data
                    b = UTF8Encoding.UTF8.GetBytes(SUBSCRIBE_ALLDATA);
                    break;
            }

            _gpsSP.Write(b, 0, b.Length);
        }

        /// <summary>
        /// Sets the Update Rate (1, 5 or 10Hz) of the GPS
        /// </summary>
        private void SetUpdateRate()
        {
            byte[] b;

            switch (_rate)
            {
                case UpdateRates.GPS_10HZ:
                    b = UTF8Encoding.UTF8.GetBytes(UPDATE_RATE_10HZ);
                    break;
                case UpdateRates.GPS_5HZ:
                    b = UTF8Encoding.UTF8.GetBytes(UPDATE_RATE_5HZ);
                    break;
                default:    // 1HZ
                    b = UTF8Encoding.UTF8.GetBytes(UPDATE_RATE_1HZ);
                    break;
            }

            _gpsSP.Write(b, 0, b.Length);
        }

        #endregion

    }
}
