using System;
using Microsoft.SPOT;
using Microsoft.SPOT.Hardware;

namespace Adafruit10DOFIMU
{
	public sealed class LSM303HDLCA : SensorBase
	{
		readonly float _MGLSB = 0.001F;   // 1, 2, 4 or 12 mg per lsb
		public const byte ACCEL_CTRL_REG1 = 0x20;   // 00000111   rw
		public const byte ACCEL_CTRL_REG2 = 0x21;   // 00000000   rw
		public const byte ACCEL_CTRL_REG3 = 0x22;   // 00000000   rw
		public const byte ACCEL_CTRL_REG4 = 0x23;   // 00000000   rw
		public const byte ACCEL_CTRL_REG5 = 0x24;   // 00000000   rw
		public const byte ACCEL_CTRL_REG6 = 0x25;   // 00000000   rw
		public const byte ACCEL_REFERENCE = 0x26;   // 00000000   r
		public const byte ACCEL_STATUS_REG = 0x27;   // 00000000   r

		public const byte ACCEL_OUT_X_L = 0x28;
		public const byte ACCEL_OUT_X_H = 0x29;
		public const byte ACCEL_OUT_Y_L = 0x2A;
		public const byte ACCEL_OUT_Y_H = 0x2B;
		public const byte ACCEL_OUT_Z_L = 0x2C;
		public const byte ACCEL_OUT_Z_H = 0x2D;

		public const byte ACCEL_FIFO_CTRL_REG = 0x2E;
		public const byte ACCEL_FIFO_SRC_REG = 0x2F;
		public const byte ACCEL_INT1_CFG = 0x30;
		public const byte ACCEL_INT1_SOURCE = 0x31;
		public const byte ACCEL_INT1_THS = 0x32;
		public const byte ACCEL_INT1_DURATION = 0x33;
		public const byte ACCEL_INT2_CFG = 0x34;
		public const byte ACCEL_INT2_SOURCE = 0x35;
		public const byte ACCEL_INT2_THS = 0x36;
		public const byte ACCEL_INT2_DURATION = 0x37;
		public const byte ACCEL_CLICK_CFG = 0x38;
		public const byte ACCEL_CLICK_SRC = 0x39;
		public const byte ACCEL_CLICK_THS = 0x3A;
		public const byte ACCEL_TIME_LIMIT = 0x3B;
		public const byte ACCEL_TIME_LATENCY = 0x3C;
		public const byte ACCEL_TIME_WINDOW = 0x3D;

		public LSM303HDLCA()
		{
			_Configuration = new I2CDevice.Configuration(0x32 >> 1, 400);
			base.Write(ACCEL_CTRL_REG1, 0x57);
		}

		public override bool Read()
		{

			//Reads all the output registers in one shot
			var buffer = Read(ACCEL_OUT_X_L | 0x80, 6);


			//Calculate the 2's complement the multiply the sensitivity
			//9.81 standard gravitational pull in m/s^2
            base.RawX = CalculateTwoComplement16(buffer[0], buffer[1]);
            base.RawY = CalculateTwoComplement16(buffer[2], buffer[3]);
            base.RawZ = CalculateTwoComplement16(buffer[4], buffer[5]);

			base.X = (CalculateTwoComplement16(buffer[0], buffer[1]) >> 4) * _MGLSB * 9.78f;
			base.Y = (CalculateTwoComplement16(buffer[2], buffer[3]) >> 4) * _MGLSB * 9.78f;
			base.Z = (CalculateTwoComplement16(buffer[4], buffer[5]) >> 4) * _MGLSB * 9.78f;

			return true;

		}



	}
}
