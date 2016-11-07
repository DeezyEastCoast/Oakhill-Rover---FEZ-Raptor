using System;
using Microsoft.SPOT;

using Microsoft.SPOT.Hardware;
using Microsoft.SPOT.IO;
using System.IO;
using System.Text;
using System.Threading;
using System.IO.Ports;

namespace Oakhill_Rover
{
    #region ALCAM

    public class ALCAM
    {
        public static byte[] pictureData;

        public static int BlockSize { get { return 4 * 1024; } }

        private CommunicationInterface alcam;

        public enum CommunicationMode
        {
            Uart,
            Spi,
            I2C
        }

        public ALCAM(CommunicationMode mode,  Cpu.Pin resetPin)
        {
            var reset = new OutputPort(resetPin, false);

            switch (mode)
            {
                case CommunicationMode.Uart:
                    break;
            }

            reset.Write(false);
            Thread.Sleep(200);
            reset.Write(true);
            Thread.Sleep(200);

            reset.Dispose();
            Thread.Sleep(200);
        }

        public ALCAM(CommunicationMode mode, Cpu.Pin sselPin, Cpu.Pin mosiPin, Cpu.Pin resetPin)
        {
            var ssel = new OutputPort(sselPin, false);
            var mosi = new OutputPort(mosiPin, false);
            var reset = new OutputPort(resetPin, false);

            switch (mode)
            {
                case CommunicationMode.Uart:
                    ssel.Write(false);
                    mosi.Write(false);

                    break;

                case CommunicationMode.Spi:
                    ssel.Write(true);
                    mosi.Write(true);


                    break;

                case CommunicationMode.I2C:
                    ssel.Write(true);
                    mosi.Write(false);

                    break;
            }

            reset.Write(false);
            Thread.Sleep(200);
            reset.Write(true);
            Thread.Sleep(200);

            ssel.Dispose();
            mosi.Dispose();
            reset.Dispose();
            Thread.Sleep(200);
        }

        public void SetInterface(CommunicationInterface device)
        {
            this.alcam = device;
        }

        private int HexStringToInt(string value)
        {
            try
            {
                return Convert.ToInt32(value.Substring(2), 16);
            }
            catch
            {
                return 0;
            }
        }

        private void SendCommand(string cmd)
        {
            this.alcam.Write(Encoding.UTF8.GetBytes(cmd + '\n'));

            Thread.Sleep(500);
        }

        private string Read(int length)
        {
            var data = new byte[length];
            var received = alcam.Read(data);

            return new string(Encoding.UTF8.GetChars(data, 0, received));
        }

        private string ReadLine()
        {
            var s = "";

            while (true)
            {
                var value = (char)alcam.Read();

                s += value;

                if (value == '\n' || value == '\r')
                    break;
            }

            return s;
        }

        public void InitializeCamera()
        {
            this.SendCommand("I C");

            this.ReadLine();
        }

        public string InitializeUsb()
        {
            this.SendCommand("I D:H");

            return this.ReadLine();
        }

        public string InitializeSD()
        {
            this.SendCommand("I S");

            return this.ReadLine();
        }

        public string ReadBanner()
        {
            return this.Read(0x49);
        }

        public string GetVersion()
        {
            this.SendCommand("V");

            return this.Read(11);
        }

        /// <summary>
        /// Set picture frame size. (0 thru 5)
        /// </summary>
        /// <returns></returns>
        public string ChangeSize()
        {
            //this.SendCommand("C S>0");
            this.SendCommand("C S>5");

            return this.ReadLine();
        }

        public string TakePictureToSD()
        {
            this.SendCommand(@"P C>S:\Demo3.jpg");

            var s = this.ReadLine();

            if (s.IndexOf("!00") >= 0)
            {
                s += this.ReadLine(); //size
                s += this.ReadLine(); //error
            }

            return s;
        }

        public string TakePictureToHost(string fileName = "DEMO01")
        {
            var s = "";

            //using (var stream = new FileStream(VolumeInfo.GetVolumes()[0].RootDirectory + @"\DEMO02.jpg", FileMode.Create))
            using (var stream = new FileStream(VolumeInfo.GetVolumes()[0].RootDirectory + @"\" + fileName + ".jpg", FileMode.Create))
            {
                this.SendCommand("P R");

                s += this.ReadLine();

                var s1 = this.ReadLine();
                var hexSize = s1.Trim(new char[] { '$', '\n' });

                s += s1;

                var size = this.HexStringToInt(hexSize);
                var block = size / ALCAM.BlockSize;
                var remainder = size % ALCAM.BlockSize;
                var data = new byte[ALCAM.BlockSize];

                while (block > 0)
                {
                    alcam.Read(data);
                    stream.Write(data, 0, ALCAM.BlockSize);
                    block--;
                }

                if (remainder > 0)
                {
                    alcam.Read(data, 0, remainder);
                    stream.Write(data, 0, remainder);
                }

                s += this.ReadLine();
            }

            VolumeInfo.GetVolumes()[0].FlushAll();

            return s;
        }

        public string TakePictureToScreen()
        {
            var s = "";

                this.SendCommand("P R");

                s += this.ReadLine();

                var s1 = this.ReadLine();
                var hexSize = s1.Trim(new char[] { '$', '\n' });

                s += s1;

                var size = this.HexStringToInt(hexSize);
                var block = size / ALCAM.BlockSize;
                var remainder = size % ALCAM.BlockSize;
                var data = new byte[ALCAM.BlockSize];

                byte[] picData = new byte[(int)size];
                int i = 0;

                while (block > 0)
                {
                    alcam.Read(data);

                    Array.Copy(data, 0, picData, block*i,block); 

                    //stream.Write(data, 0, ALCAM.BlockSize);

                    block--;
                    i++;
                }

                if (remainder > 0)
                {
                    alcam.Read(data, 0, remainder);

                    Array.Copy(data, 0, picData, block * i, remainder); 

                    //stream.Write(data, 0, remainder);
                }

                s += this.ReadLine();

                //GHI.Glide.Glide.MainWindow = Program.pictureWindow;

                //Bitmap myPic = new Bitmap(picData, Bitmap.BitmapImageType.Jpeg);
                //GHI.Glide.UI.Image image = (GHI.Glide.UI.Image)Program.pictureWindow.GetChildByName("imgPicture");
                //image.Bitmap.DrawImage(0, 0, myPic, 0, 0, 320, 240);
                //image.Invalidate();

            return s;
        }

        public string RecordMovie()
        {
            this.SendCommand(@"M B>S:\Movie.avi");

            var s = this.ReadLine();

            if (s.IndexOf("!00") >= 0)
            {
                Thread.Sleep(1000 * 10);

                this.SendCommand("M E");

                s += this.ReadLine(); //ack
                s += this.ReadLine(); //size
                s += this.ReadLine(); //error
            }

            return s;
        }

        public string WriteFile()
        {
            var s = "";

            this.SendCommand(@"F W>S:\WriteTest.txt>10");

            s = this.ReadLine();

            if (s.IndexOf("!00") >= 0)
            {
                alcam.Write(Encoding.UTF8.GetBytes("This is a test.\n"));

                s += this.ReadLine(); //size
                s += this.ReadLine(); //error
            }

            return s;
        }

        public string CloseFile()
        {
            this.SendCommand("F C");

            return this.ReadLine();
        }

        public string ReadFile()
        {
            this.SendCommand(@"F R>S:\WriteTest.txt");

            var result = this.ReadLine();

            if (result.IndexOf("!00") >= 0)
            {
                var s1 = this.ReadLine();

                result += s1;

                var hexSize = s1.Trim(new char[] { '$', '\n' });
                var size = this.HexStringToInt(hexSize);
                var block = size / ALCAM.BlockSize;
                var remainder = size % ALCAM.BlockSize;
                var buffer = new byte[ALCAM.BlockSize];

                while (block > 0)
                {
                    this.alcam.Read(buffer);
                    result += Encoding.UTF8.GetChars(buffer, 0, ALCAM.BlockSize);

                    block--;
                }

                if (remainder > 0)
                {
                    this.alcam.Read(buffer, 0, remainder);

                    result += new string(Encoding.UTF8.GetChars(buffer, 0, remainder));
                }
            }

            result += this.ReadLine();

            return result;
        }
    }

    #endregion

    #region COMMUNICATION INTERFACE ABSTRACT CLASS
    public abstract class CommunicationInterface
    {
        public abstract int Write(byte[] data, int index, int length);
        public abstract int Read(byte[] data, int index, int length);

        public int Write(byte value)
        {
            return this.Write(new byte[] { value });
        }

        public int Write(byte[] data)
        {
            return this.Write(data, 0, data.Length);
        }

        public byte Read()
        {
            var data = new byte[1];

            this.Read(data);

            return data[0];
        }

        public int Read(byte[] data)
        {
            return this.Read(data, 0, data.Length);
        }
    }
    #endregion

    #region UART INTERFACE

    //TX is pin 4 on mainboard gadgeteer port
    //RX is pin 5 on mainboard gadgeteer port
    //RTS is pin 6 on mainboard gadgeteer port
    //CTS is pin 7 on mainboard gadgeteer port

    public class UartInterface : CommunicationInterface
    {
        private SerialPort port;
        private OutputPort rts;
        private InputPort cts;

        public UartInterface(string comPort, Microsoft.SPOT.Hardware.Cpu.Pin _rts, Microsoft.SPOT.Hardware.Cpu.Pin _cts)
        {
            this.port = new SerialPort(comPort, 115200, Parity.None, 8, StopBits.One);
            this.port.Handshake = Handshake.None;
            this.port.WriteTimeout = 10000;
            this.port.ReadTimeout = 10000;
            this.port.Open();
            rts = new OutputPort(_rts, true);
            cts = new InputPort(_cts, false, Port.ResistorMode.PullUp);
        }

        public override int Write(byte[] data, int offset, int length)
        {
            if (port.Handshake == Handshake.RequestToSend)
            {
                port.Write(data, offset, length);

                return length;
            }
            else //No busy pin
            {
                var i = offset;
                var block = length / ALCAM.BlockSize;
                var remain = length % ALCAM.BlockSize;

                while (block > 0)
                {
                    port.Write(data, i, ALCAM.BlockSize);

                    i += ALCAM.BlockSize;
                    block--;
                }

                if (remain > 0)
                {
                    port.Write(data, i, remain);

                    i += remain;
                }

                return i;
            }
        }

        public override int Read(byte[] data, int offset, int length)
        {
            var i = 0;
            rts.Write(false);
            while (length > 0)
            {
                var available = port.BytesToRead;

                if (available > length)
                    available = length;

                if (available != 0)
                {
                    port.Read(data, offset + i, available);

                    i += available;
                    length -= available;
                }
            }
            rts.Write(true);
            return i;
        }
    }

    #endregion

    #region SPI INTERFACE

    public class SpiInterface : CommunicationInterface
    {
        private SPI port;
        private int timeout;
        private byte[] payloadBuffer;
        private byte[] sendBuffer;
        private byte[] receiveBuffer;

        public SpiInterface(SPI.SPI_module spiModule, Cpu.Pin chipSelect, Cpu.Pin busyPin)
        {
            this.port = new SPI(new SPI.Configuration(chipSelect, false, 0, 0, false, true, 1000, spiModule, busyPin, true));
            this.timeout = 50000;
            this.payloadBuffer = new byte[ALCAM.BlockSize];
            this.sendBuffer = new byte[3];
            this.receiveBuffer = new byte[3];
        }

        public override int Write(byte[] data, int offset, int length)
        {
            var left = timeout;

            sendBuffer[0] = (byte)0x01;
            sendBuffer[1] = (byte)((length >> 0) & 0xFF);
            sendBuffer[2] = (byte)((length >> 8) & 0xFF);

            receiveBuffer[0] = 0;
            receiveBuffer[1] = 0;
            receiveBuffer[2] = 0;

            while (receiveBuffer[1] != 1 && timeout > 0)
            {
                port.WriteRead(this.sendBuffer, 0, 2, this.receiveBuffer, 0, 2, 0);

                left--;

                Thread.Sleep(1);
            }

            if (left > 0)
            {
                port.WriteRead(this.sendBuffer, 2, 1, this.receiveBuffer, 2, 1, 0);
                port.WriteRead(data, offset, length, this.payloadBuffer, offset, length, 0);

                return length;
            }

            return 0;
        }

        public override int Read(byte[] data, int offset, int length)
        {
            var index = offset;

            while (length > 0)
            {
                var left = timeout;
                var actualSize = 0;

                while (timeout > 0)
                {
                    this.sendBuffer[0] = (byte)0x02;
                    this.sendBuffer[1] = (byte)((length >> 0) & 0xFF);
                    this.sendBuffer[2] = (byte)((length >> 8) & 0xFF);

                    port.WriteRead(this.sendBuffer, 0, 3, this.receiveBuffer, 0, 3, 0);

                    actualSize = this.receiveBuffer[1] | (this.receiveBuffer[2] << 8);

                    if (actualSize > 0)
                        break;

                    Thread.Sleep(1);

                    timeout--;
                }

                if (timeout == 0)
                    return index - offset;

                var size = length > actualSize ? actualSize : length;

                Array.Clear(this.payloadBuffer, index, size);

                port.WriteRead(this.payloadBuffer, index, size, data, index, size, 0);

                index += size;
                length -= size;
            }

            return index - offset;
        }
    }

    #endregion
}
