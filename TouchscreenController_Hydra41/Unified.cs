
using System;
using Microsoft.SPOT;
using System.Threading;

namespace Adafruit10DOFIMU
{
	public delegate void EventHandler(object sender);
	public class Unified
	{
		#region Members and Properties
		public double Heading { get; private set; }

		/// <summary>
		/// Calculated pitch
		/// </summary>
		public double Pitch { get; private set; }

		/// <summary>
		/// Calculated roll
		/// </summary>
		public double Roll { get; private set; }

		private byte _Sleep = 4;
		/// <summary>
		/// The time between each read cycle 0-255;
		/// </summary>
		public byte Sleep
		{
			set
			{
				if (value >= 0x00)
					_Sleep = value;
				else { _Sleep = 1; }
			}
		}

		private bool _Continuous = false;
		/// <summary>
		/// Set to true to allow continous sensor read.
		/// </summary>
		public bool ContinuousRead
		{
			set { _Continuous = value; }
		}

		/// <summary>
		/// Class implementation for the L3GD20 gyro.
		/// </summary>
		public L3GD20 Gyroscope { get; private set; }

		/// <summary>
		/// Class implementation for the accelerometer portion of the LSM303HDLC.
		/// </summary>
		public LSM303HDLCA Accelerometer { get; private set; }

		/// <summary>
		/// Class implementation for the magnetometer portion of the LSM303HDLC.
		/// </summary>
		public LSM303HDLCM Magnetometer { get; private set; }

		/// <summary>
		/// Class implementation for the BMP180 class.
		/// </summary>
		public BMP180 Bmp180 { get; private set; }
		#endregion

		public event EventHandler ReadCompleted;
		private static Unified _Instance;
		private DateTime _GyroReadingTimestamp;
		public static Unified Instance
		{
			get
			{
				if (_Instance == null)
					_Instance = new Unified();
				return _Instance;
			}
		}
		private Unified()
		{
			Gyroscope = new L3GD20();
			Accelerometer = new LSM303HDLCA();
			Magnetometer = new LSM303HDLCM();
			Bmp180 = new BMP180(BMP180.OSR.Standard);
			Gyroscope.GyroRangeOption = L3GD20.GyroRangeOptions.GYRO_RANGE_250DPS;
		}

		/// <summary>
		/// Call this asynchronous method to read all the I2CDevices from the Adafruit 10 DOF imu. Before this method, set the ContinuousRead property to true before call this method.
		/// </summary>
		public void BeginAsync()
		{

			if (Accelerometer.Read()) { }

			this.Pitch = (System.Math.Atan2(Accelerometer.Y, Accelerometer.Z) + System.Math.PI) * (180 / System.Math.PI);
			this.Roll = (System.Math.Atan2(Accelerometer.X, Accelerometer.Z) + System.Math.PI) * (180 / System.Math.PI);
			if (Bmp180.Read()) { }

			_GyroReadingTimestamp = DateTime.Now;

			new Thread(() =>
			{
				int read = 10000;
				do
				{
                    //if (read == 0)
                    //{
                    //    if (Bmp180.Read())
                    //    {
                    //        read = 10000;
                    //    }
                    //}

					if (Accelerometer.Read()) { }
					if (Gyroscope.Read())
					{
						// Calculate the time delta since the last reading
						DateTime now = DateTime.Now;
						double dt = (now.Ticks - _GyroReadingTimestamp.Ticks) / (double)TimeSpan.TicksPerSecond;
						_GyroReadingTimestamp = now;

						// Use a complementary filter to combine the data from the gyro and the accelerometer to
						// compensate for the gyro drift.
						this.Pitch = 0.98 * (this.Pitch + Gyroscope.Y * dt) + 0.02 * ((System.Math.Atan2(Accelerometer.Y, Accelerometer.Z) + System.Math.PI) * (180 / System.Math.PI));

						// Use a complementary filter to combine the data from the gyro and the accelerometer to
						// compensate for the gyro drift.
						this.Roll = 0.98 * (this.Roll + Gyroscope.X * dt) + 0.02 * ((System.Math.Atan2(Accelerometer.X, Accelerometer.Z) + System.Math.PI) * (180 / System.Math.PI));

					}
					if (Magnetometer.Read()) { }



					CalculateDeviceHeading();
					RaisedReadCompleted();
					read--;
					//Thread.Sleep((int)_Sleep);
				} while (_Continuous);
			}).Start();
		}

		private void CalculateDeviceHeading()
		{

			double pitch = 0.0;
			var roll = System.Math.Atan2(Accelerometer.Y, Accelerometer.Z);

			if (Accelerometer.Y * System.Math.Sin(roll) + Accelerometer.Z * System.Math.Cos(roll) == 0)
				pitch = Accelerometer.X > 0 ? (System.Math.PI / 2) : (-System.Math.PI / 2);
			else
				pitch = (float)System.Math.Atan(-Accelerometer.X / (Accelerometer.Y * System.Math.Sin(roll) +
																				 Accelerometer.Z * System.Math.Cos(roll)));


			var heading = (double)System.Math.Atan2(Magnetometer.Z * System.Math.Sin(roll) - Magnetometer.Y * System.Math.Cos(roll),
									  Magnetometer.X * System.Math.Cos(pitch) +
									  Magnetometer.Y * System.Math.Sin(pitch) * System.Math.Sin(roll) +
									  Magnetometer.Z * System.Math.Sin(pitch) * System.Math.Cos(roll));



			heading = heading * (180 / System.Math.PI);
			this.Heading = System.Math.Round(heading < 0 ? heading + 360 : heading);

			/*I commented this code out because of some weird readings for pitch and roll
			//this.Pitch = System.Math.Round(pitch * 180 / (180 / System.Math.PI));
			//this.Roll = System.Math.Round(roll * (180 / System.Math.PI));
			//this.Pitch = pitch < 0 ? 360 + pitch : pitch;
			*/

		}

		private void RaisedReadCompleted()
		{
			var evt = ReadCompleted;
			if (evt != null)
				evt(this);
		}


	}
}
