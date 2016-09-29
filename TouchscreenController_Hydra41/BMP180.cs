using System;
using Microsoft.SPOT;
using System.Threading;

namespace Adafruit10DOFIMU
{
    /// <summary>
    /// Barometric prossure sensor 300 to 1100hPa
    /// </summary>
	public class BMP180 : SensorBase
	{
		public enum OSR
		{
			Ultra_Low = 0,
			Standard = 1,
			High = 2,
			Ultra_High = 3
		}

		public const byte BMP085_REGISTER_CAL_AC1 = 0xAA;  // R   Calibration data (16 bits)
		public const byte BMP085_REGISTER_CAL_AC2 = 0xAC;  // R   Calibration data (16 bits)
		public const byte BMP085_REGISTER_CAL_AC3 = 0xAE;  // R   Calibration data (16 bits)
		public const byte BMP085_REGISTER_CAL_AC4 = 0xB0;  // R   Calibration data (16 bits)
		public const byte BMP085_REGISTER_CAL_AC5 = 0xB2;  // R   Calibration data (16 bits)
		public const byte BMP085_REGISTER_CAL_AC6 = 0xB4;  // R   Calibration data (16 bits)
		public const byte BMP085_REGISTER_CAL_B1 = 0xB6;  // R   Calibration data (16 bits)
		public const byte BMP085_REGISTER_CAL_B2 = 0xB8;  // R   Calibration data (16 bits)
		public const byte BMP085_REGISTER_CAL_MB = 0xBA;  // R   Calibration data (16 bits)
		public const byte BMP085_REGISTER_CAL_MC = 0xBC;  // R   Calibration data (16 bits)
		public const byte BMP085_REGISTER_CAL_MD = 0xBE;  // R   Calibration data (16 bits)
		public const byte BMP085_REGISTER_CHIPID = 0xD0;
		public const byte BMP085_REGISTER_VERSION = 0xD1;
		public const byte BMP085_REGISTER_SOFTRESET = 0xE0;
		public const byte BMP085_REGISTER_CONTROL = 0xF4;
		public const byte BMP085_REGISTER_READTEMPCMD = 0x2E;
		public const byte BMP085_REGISTER_READPRESSURECMD = 0x34;

		public const byte REGISTER_MSB = 0xF6;
		public const byte REGISTER_LSB = 0xF7;
		public const byte REGISTER_XLSB = 0xF8;



		short _AC1 = 408;
		short _AC2 = -72;
		short _AC3 = -14383;
		ushort _AC4 = 32741;
		ushort _AC5 = 32757;
		ushort _AC6 = 23153;
		short _B1 = 6190;
		short _B2 = 4;
		short _MB = -32768;
		short _MC = -8711;
		short _MD = 2868;
		OSR _OSR;




		public BMP180(OSR oSR = OSR.Ultra_Low)
		{
			_Configuration = new Microsoft.SPOT.Hardware.I2CDevice.Configuration(0x77, 400);
			_OSR = oSR;
			var ac1 = Read(BMP085_REGISTER_CAL_AC1, 2);
			var ac2 = Read(BMP085_REGISTER_CAL_AC2, 2);
			var ac3 = Read(BMP085_REGISTER_CAL_AC3, 2);
			var ac4 = Read(BMP085_REGISTER_CAL_AC4, 2);
			var ac5 = Read(BMP085_REGISTER_CAL_AC5, 2);
			var ac6 = Read(BMP085_REGISTER_CAL_AC6, 2);
			var b1 = Read(BMP085_REGISTER_CAL_B1, 2);
			var b2 = Read(BMP085_REGISTER_CAL_B2, 2);
			var mb = Read(BMP085_REGISTER_CAL_MB, 2);
			var mc = Read(BMP085_REGISTER_CAL_MC, 2);
			var md = Read(BMP085_REGISTER_CAL_MD, 2);

			_AC1 = (short)(ac1[0] << 8 | ac1[1]);
			_AC2 = (short)(ac2[0] << 8 | ac2[1]);
			_AC3 = (short)(ac3[0] << 8 | ac3[1]);
			_AC4 = (ushort)(ac4[0] << 8 | ac4[1]);
			_AC5 = (ushort)(ac5[0] << 8 | ac5[1]);
			_AC6 = (ushort)(ac6[0] << 8 | ac6[1]);
			_B1 = (short)(b1[0] << 8 | b1[1]);
			_B2 = (short)(b2[0] << 8 | b2[1]);
			_MB = (short)(mb[0] << 8 | mb[1]);
			_MC = (short)(mc[0] << 8 | mc[1]);
			_MD = (short)(md[0] << 8 | md[1]);
		}


		public override bool Read()
		{

			base.Temperature = CalculateTrueTemperature() / 10f;
			base.Pressure = CalculateTruePressure();

			return true;
		}

		private short CalculateTrueTemperature()
		{
			//Start temperature conversion
			base.Write(BMP085_REGISTER_CONTROL, BMP085_REGISTER_READTEMPCMD);

			//This should give the device plenty of time to complete the conversion.
			Thread.Sleep(30);

			//Read the temperature values
			var msb = base.Read(REGISTER_MSB);
			var lsb = base.Read(REGISTER_LSB);

			short ut = (short)((msb << 8) + lsb);

			var x1 = ((ut - _AC6) * _AC5) >> 15;
			var x2 = (_MC << 11) / (x1 + _MD);

			var temp = (short)((x1 + x2 + 8) >> 4);


			return temp;
		}


		public float CalculateTruePressure()
		{
			var temp = CalculateTrueTemperature();

			//Start Pressure conversion
			base.Write(BMP085_REGISTER_CONTROL, (byte)(BMP085_REGISTER_READPRESSURECMD + ((byte)_OSR << 6)));

			//This should give the device plenty of time to complete its conversion.
			Thread.Sleep(75);

			//Read the temperature values
			float uP = ((base.Read(REGISTER_MSB) << 16 | base.Read(REGISTER_LSB) << 8) | base.Read(REGISTER_XLSB)) >> (8 - (int)_OSR);

			var b6 = temp - 4000;

			long x1 = (_B2 * ((b6 * b6) >> 12)) >> 11;
			long x2 = (_AC2 * b6) >> 11;
			long x3 = x1 + x2;
			long b3 = (((_AC1 * 4 + x3) << (int)_OSR) + 2) >> 2;

			x1 = (_AC3 * b6) >> 13;
			x2 = (_B1 * ((b6 * b6) >> 12)) >> 16;
			x3 = ((x1 + x2) + 2) >> 2;

			ulong b4 = (ulong)((_AC4 * (x3 + 32768)) >> 15);
			ulong b7 = (ulong)((uP - b3) * (50000 >> (int)_OSR));

			ulong pressure = 0;
			if (b7 < 0x80000000)
			{
				pressure = (ulong)((b7 << 1) / b4);
			}
			else
			{
				pressure = (b7 / b4) << 1;
			}

			long v = -7357L;

			x1 = ((long)(pressure >> 8)) * ((long)(pressure >> 8));
			x1 = (x1 * 3038) >> 16;
			x2 = (v * (long)pressure) >> 16;



			pressure = pressure + (ulong)((x1 + x2 + 3791) >> 4); //pressure is calculated in Pa (1hPa = 100Pa)


			return pressure;
		}
	}
}
