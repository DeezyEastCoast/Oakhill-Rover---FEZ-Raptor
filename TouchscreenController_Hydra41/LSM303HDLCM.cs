using System;
using Microsoft.SPOT;
using Microsoft.SPOT.Hardware;

namespace Adafruit10DOFIMU
{
	public class LSM303HDLCM : SensorBase
	{
		public enum GainOptions
		{
			GAIN_1_3 = 0x20,  // +/- 1.3
			GAIN_1_9 = 0x40,  // +/- 1.9
			GAIN_2_5 = 0x60,  // +/- 2.5
			GAIN_4_0 = 0x80,  // +/- 4.0
			GAIN_4_7 = 0xA0,  // +/- 4.7
			GAIN_5_6 = 0xC0,  // +/- 5.6
			GAIN_8_1 = 0xE0   // +/- 8.1
		}


		public const byte LSM303_REGISTER_MAG_CRA_REG_M = 0x00;
		public const byte LSM303_REGISTER_MAG_CRB_REG_M = 0x01;
		public const byte LSM303_REGISTER_MAG_MR_REG_M = 0x02;
		public const byte LSM303_REGISTER_MAG_OUT_X_H_M = 0x03;
		public const byte LSM303_REGISTER_MAG_OUT_X_L_M = 0x04;
		public const byte LSM303_REGISTER_MAG_OUT_Z_H_M = 0x05;
		public const byte LSM303_REGISTER_MAG_OUT_Z_L_M = 0x06;
		public const byte LSM303_REGISTER_MAG_OUT_Y_H_M = 0x07;
		public const byte LSM303_REGISTER_MAG_OUT_Y_L_M = 0x08;
		public const byte LSM303_REGISTER_MAG_SR_REG_Mg = 0x09;
		public const byte LSM303_REGISTER_MAG_IRA_REG_M = 0x0A;
		public const byte LSM303_REGISTER_MAG_IRB_REG_M = 0x0B;
		public const byte LSM303_REGISTER_MAG_IRC_REG_M = 0x0C;
		public const byte LSM303_REGISTER_MAG_TEMP_OUT_H_M = 0x31;
		public const byte LSM303_REGISTER_MAG_TEMP_OUT_L_M = 0x32;

		private GainOptions _Gain = GainOptions.GAIN_1_3;

		private float _M_GN_XY = 1100f;
		private float _M_GN_Z = 980f;
		public GainOptions Gain
		{
			set
			{
				_Gain = value;
				switch (value)
				{
					case GainOptions.GAIN_1_3:
						_M_GN_XY = 1100f;
						_M_GN_Z = 980f;
						break;
					case GainOptions.GAIN_1_9:
						_M_GN_XY = 855f;
						_M_GN_Z = 760f;
						break;
					case GainOptions.GAIN_2_5:
						_M_GN_XY = 670f;
						_M_GN_Z = 600f;
						break;
					case GainOptions.GAIN_4_0:
						_M_GN_XY = 450f;
						_M_GN_Z = 400f;
						break;
					case GainOptions.GAIN_4_7:
						_M_GN_XY = 400f;
						_M_GN_Z = 355f;
						break;
					case GainOptions.GAIN_5_6:
						_M_GN_XY = 330f;
						_M_GN_Z = 295f;
						break;
					case GainOptions.GAIN_8_1:
						_M_GN_XY = 230f;
						_M_GN_Z = 205f;
						break;
				}
				//Set gain for magnetometer
				base.Write(LSM303_REGISTER_MAG_CRB_REG_M, (byte)_Gain);
			}
		}


		public LSM303HDLCM()
		{
			base._Configuration = new I2CDevice.Configuration(0x3C >> 1, 400);
			base.Write(LSM303_REGISTER_MAG_MR_REG_M, 0x00);
			base.Write(LSM303_REGISTER_MAG_CRB_REG_M, (byte)_Gain);
		}

        /// <summary>
        /// Check that device is responsive.
        /// </summary>
        /// <returns></returns>
        public override bool IsAlive()
        {
            // LSM303DLHC has no WHOAMI register so read CRA_REG_M to check
            // the default value (0b00010000/0x10)
            byte reg1_a = Read(LSM303_REGISTER_MAG_CRA_REG_M);
            if (reg1_a != 0x10)
            {
                return false;
            }
            return true;
        }

		public override bool Read()
		{


			//Read magnetometer device

			byte MSBx_m = this.Read(LSM303_REGISTER_MAG_OUT_X_H_M);
			byte LSBx_m = this.Read(LSM303_REGISTER_MAG_OUT_X_L_M);

            base.RawX = CalculateTwoComplement16(LSBx_m, MSBx_m);
			base.X = CalculateTwoComplement16(LSBx_m, MSBx_m) / _M_GN_XY * 100f;

			byte MSBy_m = this.Read(LSM303_REGISTER_MAG_OUT_Y_H_M);
			byte LSBy_m = this.Read(LSM303_REGISTER_MAG_OUT_Y_L_M);
            base.RawY = CalculateTwoComplement16(LSBy_m, MSBy_m);
			base.Y = CalculateTwoComplement16(LSBy_m, MSBy_m) / _M_GN_XY * 100f;


			byte MSBz_m = this.Read(LSM303_REGISTER_MAG_OUT_Z_H_M);
			byte LSBz_m = this.Read(LSM303_REGISTER_MAG_OUT_Z_L_M);
            base.RawZ = CalculateTwoComplement16(LSBz_m, MSBz_m);
			base.Z = CalculateTwoComplement16(LSBz_m, MSBz_m) / _M_GN_Z * 100f;



			return true;
		}



	}
}
