using System;
using System.IO.Ports;
using System.Text;
using System.Threading;

using Microsoft.SPOT;

namespace Oakhill_Rover
{
    public class EMIC2
    {
        private static SerialPort emic;
        private static byte[] readBuf = new byte[1];


        public EMIC2()
        {
            emic = new SerialPort("com1", 9600, Parity.None, 8, StopBits.One);
            setupEmic();
        }

        public EMIC2(string portname)
        {
            emic = new SerialPort(portname, 9600, Parity.None, 8, StopBits.One);
            setupEmic();
        }


        public EMIC2(SerialPort portname)
        {
            emic = portname;
        }

        private void setupEmic()
        {
            //            emic.ReadTimeout = Timeout.Infinite;
            //            emic.WriteTimeout = Timeout.Infinite;
            emic.ReadTimeout = 500;
            emic.WriteTimeout = 500;
            try
            {
                emic.Open();
            }
            catch
            {
                throw new Exception("Unhandled failure to open serial port");
            }


        }

        public bool isConnected()
        {
            if (emic != null)
            {
                return true;
            }
            else
                return false;
        }

        public bool isReady()
        {
            if (emic != null)
            {
                // wait for serial ":" or send CR and get ":" back
                return true;
            }
            else
                return false;
        }

        public bool Say(string sentence)
        {
            if (this.isConnected())
            {
                while (sentence.Length > 1020)
                {
                    string sub;
                    sub = sentence.Substring(0, 1020);
                    sendString(sub);
                    sentence = sentence.Substring(1021, sentence.Length - 1020);
                }
                sendString("S" + sentence);
                return true;
            }
            else
                return false;
        }

        public void changeVoice(int voice)
        {
            if (this.isConnected())
            {
                //send N
                //wait for ":"

                if (voice > 8)
                    voice = 8;
                else if (voice < 0)
                    voice = 0;

                string buffer = "N" + voice.ToString();
                sendString(buffer);
            }
        }

        public void changeLanguage(int lang)
        {
            if (this.isConnected())
            {
                //send L
                //wait for ":"

                if (lang > 2)
                    lang = 2;
                else if (lang < 0)
                    lang = 0;

                string buffer = "L" + lang.ToString();
                sendString(buffer);
            }
        }

        public void changeVolume(int volume)
        {
            if (this.isConnected())
            {
                //send V
                //wait for ":"

                if (volume > 18)
                    volume = 18;
                else if (volume < -48)
                    volume = -48;

                string buffer = "V" + volume.ToString();
                sendString(buffer);
            }
        }

        public void changeSpeed(int speed)
        {
            if (this.isConnected())
            {
                //send W
                //wait for ":"

                if (speed > 600)
                    speed = 600;
                else if (speed < 75)
                    speed = 75;

                string buffer = "W" + speed.ToString();
                sendString(buffer);
            }
        }

        public void changeParser(int parser)
        {
            if (this.isConnected())
            {
                //send P

                if (parser != 0 || parser != 1)
                    parser = 1;

                string buffer = "P" + parser.ToString();
                sendString(buffer);
            }
        }

        private bool sendString(string str)
        {
            string buffer = str + "\r\n";
            Debug.Print("Sending:");
            Debug.Print(buffer);
            emic.Write(UTF8Encoding.UTF8.GetBytes(buffer), 0, buffer.Length);
            // wait for the ":" after processing the instruction
            bool returnVal = ReadLine(emic, (int)':');
            if (returnVal == true)
                Debug.Print("readval True ");
            else
                Debug.Print("readval false");

            return returnVal;

        }
        public static bool ReadLine(SerialPort port, int delim = 10)
        {
            //CR 13 LF 10 = "\r\n"
            bool keepGoing = true;
            int curPos = 0;
            readBuf[curPos] = 0;
            int read = 0;

            while (keepGoing && readBuf[curPos] != delim)
            {
                read = port.Read(readBuf, curPos, 1);
                if (read > 0)
                {
                    Debug.Print("Read= " + (char)readBuf[curPos]);
                    if (readBuf[curPos] == delim)
                    {
                        // we got a delimeter; it might only be in a stream of chars, so try a read-ahead
                        keepGoing = false;
                        read = port.Read(readBuf, curPos, 1);
                        if (read != 0)
                        {
                            // if we read a character here, the timeout didn't come into play, therefore the delimeter we picked up was not the terminator and we have to keep reading the serial port
                            Debug.Print("Read_ = " + (char)readBuf[curPos]);
                            keepGoing = true;
                        }
                    }
                }
            }

            if (readBuf[curPos] == delim)
                return true;
            else
                return false;

        }



    }
}
