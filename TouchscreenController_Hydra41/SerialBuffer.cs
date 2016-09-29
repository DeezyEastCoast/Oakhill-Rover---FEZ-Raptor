using System;
using Microsoft.SPOT;
using System.IO.Ports;

namespace TouchscreenController_Hydra41
{
    class SerialBuffer
    {
        private System.Text.Decoder decoder = System.Text.UTF8Encoding.UTF8.GetDecoder();
        private byte[] buffer;
        private int startIndex = 0;
        private int endIndex = 0;
        private char[] charBuffer;

        public SerialBuffer(int initialSize)
        {
            buffer = new byte[initialSize];
            charBuffer = new char[256];
        }

        public void LoadSerial(SerialPort port)
        {
            int bytesToRead = port.BytesToRead;
            if (buffer.Length < endIndex + bytesToRead) // do we have enough buffer to hold this read?
            {
                // if not, look and see if we have enough free space at the front
                if (buffer.Length - DataSize >= bytesToRead)
                {
                    ShiftBuffer();
                }
                else
                {
                    // not enough room, we'll have to make a bigger buffer
                    ExpandBuffer(DataSize + bytesToRead);
                }
            }
            //Debug.Print("serial buffer load " + bytesToRead + " bytes read, " + DataSize + " buffer bytes before read");
            port.Read(buffer, endIndex, bytesToRead);
            endIndex += bytesToRead;

        }


        private void ShiftBuffer()
        {
            // move the data to the left, reclaiming space from the data already read out
            Array.Copy(buffer, startIndex, buffer, 0, DataSize);
            endIndex = DataSize;
            startIndex = 0;
        }

        private void ExpandBuffer(int newSize)
        {
            byte[] newBuffer = new byte[newSize];
            Array.Copy(buffer, startIndex, newBuffer, 0, DataSize);
            buffer = newBuffer;
            endIndex = DataSize;
            startIndex = 0;
        }

        public byte[] Buffer
        {
            get
            {
                return buffer;
            }
        }

        public int DataSize
        {
            get
            {
                return endIndex - startIndex;
            }
        }

        public string ReadLine()
        {
            lock (buffer)
            {
                int lineEndPos = Array.IndexOf(buffer, '\n', startIndex, DataSize);  // HACK: not looking for \r, just assuming that they'll come together       
                if (lineEndPos > 0)
                {
                    int lineLength = lineEndPos - startIndex;
                    if (charBuffer.Length < lineLength)  // do we have enough space in our char buffer?
                    {
                        charBuffer = new char[lineLength];
                    }
                    int bytesUsed, charsUsed;
                    bool completed;
                    decoder.Convert(buffer, startIndex, lineLength, charBuffer, 0, lineLength, true, out bytesUsed, out charsUsed, out completed);
                    string line = new string(charBuffer, 0, lineLength);
                    startIndex = lineEndPos + 1;


                    //Debug.Print("found string length " + lineLength + "; new buffer = " + startIndex + " to " + endIndex);
                    return line;
                }
                else
                {
                    return null;
                }
            }
        }
    }
}
