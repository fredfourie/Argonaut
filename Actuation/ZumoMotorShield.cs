using System;
using System.Diagnostics;
using System.Threading.Tasks;
using Windows.Devices.Gpio;

namespace Actuation
{
    public class ZumoMotorShieldConfig
    {
        public ZumoMotorShieldConfig()
        {
            LeftMotorDirPin = 0;
            RightPwmChannel = 0;
            LeftPwmChannel = 0;
            RightPwmChannel = 0;
            PwmDriverSlaveAddress = 0;
        }

        public int LeftMotorDirPin;
        public int RightMotorDirPin;
        public int LeftPwmChannel;
        public int RightPwmChannel;
        public int PwmDriverSlaveAddress;

        public override string ToString()
        {
            return string.Format(
                "LeftMotorDirPin={0}, RightMotorDirPin={1}, LeftPwmChannel={2}, RightPwmChannel={3}, PwmDriverSlaveAddress=0x{4}",
                LeftMotorDirPin,
                RightMotorDirPin,
                LeftPwmChannel,
                RightPwmChannel,
                PwmDriverSlaveAddress.ToString("X2"));
        }
    }

    public enum ZumoMotorDirection
    {
        Forward,
        Backward
    }

    public class ZumoMotorShield : IDisposable
    {
        public ZumoMotorShield(ZumoMotorShieldConfig config)
        {
            Config = config;
        }

        public async Task Init()
        {
            Debug.WriteLine("Initializaing ZumoMotorShield");

            Debug.WriteLine(Config.ToString());

            var gpioCtrlr = GpioController.GetDefault();

            LeftMotorDir = gpioCtrlr.OpenPin(Config.LeftMotorDirPin);
            Debug.Assert(LeftMotorDir != null);

            RightMotorDir = gpioCtrlr.OpenPin(Config.RightMotorDirPin);
            Debug.Assert(RightMotorDir!= null);

            LeftMotorDir.SetDriveMode(GpioPinDriveMode.Output);
            RightMotorDir.SetDriveMode(GpioPinDriveMode.Output);

            PwmDriver = new PCA9685(Config.PwmDriverSlaveAddress);
            await PwmDriver.Init();
        }

        public void SetLeftMotorPower(ZumoMotorDirection dir, float power)
        {
            Debug.WriteLine("LeftMotor: {0} {1}", dir, power * 100.0f);

            if (dir == ZumoMotorDirection.Forward)
                LeftMotorDir.Write(GpioPinValue.Low);
            else
                LeftMotorDir.Write(GpioPinValue.High);

            PwmDriver.SetChannelDutyCycle(Config.LeftPwmChannel, power);
        }

        public void SetRightMotorPower(ZumoMotorDirection dir, float power)
        {
            Debug.WriteLine("RightMotor: {0} {1}", dir, power * 100.0f);

            if (dir == ZumoMotorDirection.Forward)
                RightMotorDir.Write(GpioPinValue.Low);
            else
                RightMotorDir.Write(GpioPinValue.High);

            PwmDriver.SetChannelDutyCycle(Config.RightPwmChannel, power);
        }

        public void LeftMotorStop()
        {
            Debug.WriteLine("LefttMotor: Stop");

            SetLeftMotorPower(ZumoMotorDirection.Forward, 0.0f);
        }

        public void RightMotorStop()
        {
            Debug.WriteLine("RightMotor: Stop");

            SetRightMotorPower(ZumoMotorDirection.Forward, 0.0f);
        }

        public void Dispose()
        {
            if (PwmDriver != null)
            {
                PwmDriver.Dispose();
                PwmDriver = null;
            }
        }

        GpioPin LeftMotorDir;
        GpioPin RightMotorDir;
        PCA9685 PwmDriver;
        ZumoMotorShieldConfig Config;
    }
}
