using System;
using Microsoft.SPOT;
using Microsoft.SPOT.Hardware;
using System.Threading;

namespace Adafruit10DOFIMU
{
	public sealed class L3GD20 : SensorBase
	{
		public enum GyroRangeOptions
		{
			GYRO_RANGE_250DPS = 250,
			GYRO_RANGE_500DPS = 500,
			GYRO_RANGE_2000DPS = 2000

		}
		#region CONSTANT MEMBER VARIABLES
		public const float GYRO_SENSITIVITY_250DPS = 0.00875F;
		public const float GYRO_SENSITIVITY_500DPS = 0.0175F;
		public const float GYRO_SENSITIVITY_2000DPS = 0.070F;

		public const byte GYRO_REGISTER_WHOM_I = 0x0F;       // 11010100   r

		public const byte GYRO_REGISTER_CTRL_REG1 = 0x20;      // 00000111   rw
		public const byte GYRO_REGISTER_CTRL_REG2 = 0x21;      // 00000000   rw
		public const byte GYRO_REGISTER_CTRL_REG3 = 0x22;      // 00000000   rw
		public const byte GYRO_REGISTER_CTRL_REG4 = 0x23;      // 00000000   rw
		public const byte GYRO_REGISTER_CTRL_REG5 = 0x24;      // 00000000   rw
		public const byte GYRO_REGISTER_REFERENCE = 0x25;      // 00000000   rw
		public const byte GYRO_REGISTER_OUT_TEMP = 0x26;       //            r
		public const byte GYRO_REGISTER_STATUS_REG = 0x27;     //            r

		public const byte GYRO_REGISTER_OUT_X_L = 0x28;        //            r
		public const byte GYRO_REGISTER_OUT_X_H = 0x29;        //            r
		public const byte GYRO_REGISTER_OUT_Y_L = 0x2A;        //            r
		public const byte GYRO_REGISTER_OUT_Y_H = 0x2B;        //            r
		public const byte GYRO_REGISTER_OUT_Z_L = 0x2C;        //            r
		public const byte GYRO_REGISTER_OUT_Z_H = 0x2D;        //            r

		public const byte GYRO_REGISTER_FIFO_CTRL_REG = 0x2E;  // 00000000   rw
		public const byte GYRO_REGISTER_FIFO_SRC_REG = 0x2F;   //            r
		public const byte GYRO_REGISTER_INT1_CFG = 0x30;       // 00000000   rw
		public const byte GYRO_REGISTER_INT1_SRC = 0x31;       //            r
		public const byte GYRO_REGISTER_TSH_XH = 0x32;         // 00000000   rw
		public const byte GYRO_REGISTER_TSH_XL = 0x33;         // 00000000   rw
		public const byte GYRO_REGISTER_TSH_YH = 0x34;         // 00000000   rw
		public const byte GYRO_REGISTER_TSH_YL = 0x35;         // 00000000   rw
		public const byte GYRO_REGISTER_TSH_ZH = 0x36;         // 00000000   rw
		public const byte GYRO_REGISTER_TSH_ZL = 0x37;         // 00000000   rw
		public const byte GYRO_REGISTER_INT1_DURATION = 0x38;  // 00000000   rw 
		#endregion

		#region MEMBERS AND PROPERTIES
        private readonly ushort _L3GD20_ID = 215; //id for L3GD20H
        //private readonly ushort _L3GD20_ID = 212; //old id for L3GD20

		private GyroRangeOptions _GyroRangeOption = GyroRangeOptions.GYRO_RANGE_250DPS;
		public GyroRangeOptions GyroRangeOption
		{
			set
			{
				_GyroRangeOption = value;
				switch (value)
				{
					case GyroRangeOptions.GYRO_RANGE_250DPS:
						base.Write(GYRO_REGISTER_CTRL_REG4, 0x00);
						break;
					case GyroRangeOptions.GYRO_RANGE_500DPS:
						base.Write(GYRO_REGISTER_CTRL_REG4, 0x10);
						break;
					case GyroRangeOptions.GYRO_RANGE_2000DPS:
						base.Write(GYRO_REGISTER_CTRL_REG4, 0x20);
						break;
				}
		
		} }
		#endregion

		public L3GD20(GyroRangeOptions gyroRangeOption = GyroRangeOptions.GYRO_RANGE_500DPS)
		{
            _Configuration = new I2CDevice.Configuration(0x6B, 400);
			_GyroRangeOption = gyroRangeOption;
			if (Read(GYRO_REGISTER_WHOM_I) != _L3GD20_ID)
			{
				Debug.Print("Gyro device not found.");
				//throw new Exception("Device not found.");
			}

			//Reset to default
			base.Write(GYRO_REGISTER_CTRL_REG1, 0x00);

			//Enables all access
			base.Write(GYRO_REGISTER_CTRL_REG1, 0x0F);

			switch (_GyroRangeOption)
			{
				case GyroRangeOptions.GYRO_RANGE_250DPS:
					base.Write(GYRO_REGISTER_CTRL_REG4, 0x00);
					break;
				case GyroRangeOptions.GYRO_RANGE_500DPS:
					base.Write(GYRO_REGISTER_CTRL_REG4, 0x10);
					break;
				case GyroRangeOptions.GYRO_RANGE_2000DPS:
					base.Write(GYRO_REGISTER_CTRL_REG4, 0x20);
					break;
			}

		}

        /// <summary>
        /// Check that device is responsive.
        /// </summary>
        /// <returns></returns>
        public override bool IsAlive()
        {
            if (Read(GYRO_REGISTER_WHOM_I) != _L3GD20_ID)
            {
                //Debug.Print("Gyro device not found.");
                //throw new Exception("Device not found.");
                return false;
            }

            return true;
        }

		public override bool Read()
		{


			float sensitivity = 1f;

			switch (_GyroRangeOption)
			{
				case GyroRangeOptions.GYRO_RANGE_250DPS:
					sensitivity = (float)GYRO_SENSITIVITY_250DPS;
					break;
				case GyroRangeOptions.GYRO_RANGE_500DPS:
					sensitivity = (float)GYRO_SENSITIVITY_500DPS;
					break;
				case GyroRangeOptions.GYRO_RANGE_2000DPS:
					sensitivity = (float)GYRO_SENSITIVITY_2000DPS;
					break;
			}



			//Reads all the output registers in one shot
			var buffer = Read(GYRO_REGISTER_OUT_X_L | 0x80, 6);


			//Calculate the 2's complement the multiply the sensitivity
            base.RawX = CalculateTwoComplement16(buffer[0], buffer[1]);
            base.RawY = CalculateTwoComplement16(buffer[2], buffer[3]);
            base.RawZ = CalculateTwoComplement16(buffer[4], buffer[5]);

			base.X = CalculateTwoComplement16(buffer[0], buffer[1]) * sensitivity * 0.02f;
			base.Y = CalculateTwoComplement16(buffer[2], buffer[3]) * sensitivity * 0.02f;
			base.Z = CalculateTwoComplement16(buffer[4], buffer[5]) * sensitivity * 0.02f;

			return true;
		}
	}
}
