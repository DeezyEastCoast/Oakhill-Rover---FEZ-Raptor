using System;
using System.IO.Ports;
using System.Text;
using System.IO;

using Microsoft.SPOT;
using Microsoft.SPOT.IO;

using Gadgeteer;

namespace Power_Uart_Mux
{
    class OHS_UartMux
    {
        static Microsoft.SPOT.Hardware.Cpu.Pin muxS0Pin;
        static Microsoft.SPOT.Hardware.OutputPort muxS0;
        static Microsoft.SPOT.Hardware.Cpu.Pin muxS1Pin;
        static Microsoft.SPOT.Hardware.OutputPort muxS1;
        static Microsoft.SPOT.Hardware.Cpu.Pin muxEnPin;
        static Microsoft.SPOT.Hardware.OutputPort muxEn;

        /// <summary>
        /// Reference to hardware serial port used by power uart mux.
        /// </summary>
        static public SerialPort MuxComPort;
        static private int[] MuxBaudRates = new int[] { 9600, 9600, 9600, 9600 };

        static public string MuxComPortName;
        static private int activeMuxComPort = 0;

        /// <summary>
        /// Constructor.
        /// </summary>
        /// <param name="kSocket">Socket number of a "K" type socket.</param>
        /// <param name="baudRate">Initial baud rate.</param>
        public OHS_UartMux(int kSocket)
        {
            MuxComPortName = Gadgeteer.Socket.GetSocket(kSocket, true, null, null).SerialPortName;

            muxS0Pin = Gadgeteer.Socket.GetSocket(kSocket, true, null, null).CpuPins[3];
            muxS1Pin = Gadgeteer.Socket.GetSocket(kSocket, true, null, null).CpuPins[6]; //usually RTS output pin on a K socket
            muxEnPin = Gadgeteer.Socket.GetSocket(kSocket, true, null, null).CpuPins[7]; //usually CTS input pin on a K socket

            muxS0 = new Microsoft.SPOT.Hardware.OutputPort(muxS0Pin, false);
            muxS1 = new Microsoft.SPOT.Hardware.OutputPort(muxS1Pin, false);
            muxEn = new Microsoft.SPOT.Hardware.OutputPort(muxEnPin, false); //active low enable

            //for compass calibration
            //MuxComPort = new SerialPort(MuxComPortName,115200);
           // MuxComPort.Open();

        }

        /// <summary>
        /// Constructor.
        /// </summary>
        /// <param name="kSocket">Socket number of a "K" type socket.</param>
        /// <param name="baudRate">Initial baud rate.</param>
        public OHS_UartMux(int kSocket, int baudRate0 = 9600, int baudRate1 = 9600, int baudRate2 = 9600, int baudRate3 = 9600)
        {
            MuxComPortName = Gadgeteer.Socket.GetSocket(kSocket, true, null, null).SerialPortName;

            muxS0Pin = Gadgeteer.Socket.GetSocket(kSocket, true, null, null).CpuPins[3];
            muxS1Pin = Gadgeteer.Socket.GetSocket(kSocket, true, null, null).CpuPins[6]; //usually RTS output pin on a K socket
            muxEnPin = Gadgeteer.Socket.GetSocket(kSocket, true, null, null).CpuPins[7]; //usually CTS input pin on a K socket

            muxS0 = new Microsoft.SPOT.Hardware.OutputPort(muxS0Pin, false);
            muxS1 = new Microsoft.SPOT.Hardware.OutputPort(muxS1Pin, false);
            muxEn = new Microsoft.SPOT.Hardware.OutputPort(muxEnPin, false); //active low enable

            MuxBaudRates[0] = baudRate0;
            MuxBaudRates[1] = baudRate1;
            MuxBaudRates[2] = baudRate2;
            MuxBaudRates[3] = baudRate3;

            MuxComPortName = Gadgeteer.Socket.GetSocket(kSocket, true, null, null).SerialPortName;
            MuxComPort = new SerialPort(MuxComPortName, baudRate0);

            MuxComPort.Open();      
        }

        /// <summary>
        /// 
        /// </summary>
        /// <param name="bytesToWrite"></param>
        /// <param name="port"></param>
        public void writeMuxCom(byte[] bytesToWrite, int port)
        {
            ActiveComPort = port;
            MuxComPort.Write(bytesToWrite, 0, bytesToWrite.Length);
        }

        /// <summary>
        /// Set/get the com port address.
        /// </summary>
        public int ActiveComPort
        {
            set
            {
                //switch baud rate if necessary
                if (value != activeMuxComPort)
                {
                    //disable mux
                    EnableMuxCom = false;

                    if ((value >= 0) && (value <= 3))
                    {
                        MuxComPort.Close();
                        MuxComPort.BaudRate = MuxBaudRates[value];
                        //MuxComPort.WriteTimeout = 500; //used for EMIC voic device
                        //MuxComPort.ReadTimeout = 500;
                        MuxComPort.Open();
                    }

                    switch (value)
                    {
                        case 0:
                            {
                                muxS0.Write(false);
                                muxS1.Write(false);
                                break;
                            }
                        case 1:
                            {
                                muxS0.Write(true);
                                muxS1.Write(false);
                                break;
                            }
                        case 2:
                            {
                                muxS0.Write(false);
                                muxS1.Write(true);
                                break;
                            }
                        case 3:
                            {
                                muxS0.Write(true);
                                muxS1.Write(true);
                                break;
                            }
                    }

                    activeMuxComPort = value;
                    EnableMuxCom = true; //enable com ports
                }
            }

            get
            {
                return activeMuxComPort;
            }
        }

        /// <summary>
        /// Set/get the com port address.
        /// </summary>
        static public int myActiveComPort
        {
            set
            {
                if ((value >= 0) && (value <= 3))
                {
                    //disable mux
                    EnableMuxCom = false;

                    switch (value)
                    {
                        case 0:
                            {
                                muxS0.Write(false);
                                muxS1.Write(false);
                                break;
                            }
                        case 1:
                            {
                                muxS0.Write(true);
                                muxS1.Write(false);
                                break;
                            }
                        case 2:
                            {
                                muxS0.Write(false);
                                muxS1.Write(true);
                                break;
                            }
                        case 3:
                            {
                                muxS0.Write(true);
                                muxS1.Write(true);
                                break;
                            }
                    }

                    activeMuxComPort = value;
                    EnableMuxCom = true; //enable com ports
                }
            }

            get
            {
                return activeMuxComPort;
            }
        }

        /// <summary>
        /// Enable/disable all com ports.
        /// </summary>
        static public bool EnableMuxCom
        {
            //active low enable
            set { muxEn.Write(!value); }

            get { return !muxEn.Read(); }
        }

    }
}
