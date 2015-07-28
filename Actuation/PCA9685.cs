using System;
using Windows.Devices.I2c;
using Windows.Devices.Enumeration;
using System.Diagnostics;
using System.Threading.Tasks;

namespace Actuation
{
    public class PCA9685 : IDisposable
    {
        private const byte MODE1 = 0x00;
        private const byte MODE2 = 0x01;
        private const byte SUBADDR1 = 0x02;
        private const byte SUBADDR2 = 0x03;
        private const byte PRESCALE = 0xFE;
        private const byte LED0_ON_L = 0x06;
        private const byte LED0_ON_H = 0x07;
        private const byte LED0_OFF_L = 0x08;
        private const byte LED0_OFF_H = 0x09;
        private const byte ALL_LED_ON_L = 0xFA;
        private const byte ALL_LED_ON_H = 0xFB;
        private const byte ALL_LED_OFF_L = 0xFC;
        private const byte ALL_LED_OFF_H = 0xFD;

        private const byte RESTART = 0x80;
        private const byte SLEEP = 0x10;
        private const byte ALLCALL = 0x01;
        private const byte INVRT = 0x10;
        private const byte OUTDRV = 0x04;
        private const byte SWRST = 0x6;

        private const int PwmCounterMax = 4095;
        private const int GeneralCallSlaveAddress = 0;

        public PCA9685(int slaveAddress)
        {
            SlaveAddress = slaveAddress;
        }

        public async Task Init()
        {
            Debug.WriteLine("Initializing PCA9685");

            string aqs = I2cDevice.GetDeviceSelector();
            var dis = await DeviceInformation.FindAllAsync(aqs);
            if (dis.Count == 0)
            {
                Debug.WriteLine("No I2C controllers were found on the system");
                return;
            }

            var settings = new I2cConnectionSettings(GeneralCallSlaveAddress);
            settings.BusSpeed = I2cBusSpeed.FastMode;
            GeneralCallDev = await I2cDevice.FromIdAsync(dis[0].Id, settings);
            if (GeneralCallDev == null)
            {
                Debug.WriteLine(
                    string.Format(
                        "Slave address {0} on I2C Controller {1} is currently in use by " +
                        "another application. Please ensure that no other applications are using I2C.",
                        settings.SlaveAddress,
                        dis[0].Id));
                return;
            }

            SoftwareReset();

            settings = new I2cConnectionSettings(SlaveAddress);
            settings.BusSpeed = I2cBusSpeed.FastMode;
            Dev = await I2cDevice.FromIdAsync(dis[0].Id, settings);
            if (Dev == null)
            {
                Debug.WriteLine(
                    string.Format(
                        "Slave address {0} on I2C Controller {1} is currently in use by " +
                        "another application. Please ensure that no other applications are using I2C.",
                        settings.SlaveAddress,
                        dis[0].Id));
                return;
            }

            Debug.WriteLine("PCA9685 I2C channels created");

            SetAllChannelsDutyCycle(0.0f);
            // Output drive mode is totem-pole not open drain
            WriteReg(MODE2, OUTDRV);
            // Turn-offi oscillator and acknowledge All-Call transfers
            WriteReg(MODE1, ALLCALL);
            await Task.Delay(1);

            byte mode1 = ReadReg(MODE1);
            // Trun-on oscillator
            mode1 &= unchecked((byte)~SLEEP);
            WriteReg(MODE1, mode1);
            await Task.Delay(1);

            Debug.WriteLine("PCA9685 initialization complete");
        }

        public void SetPwmFrequency(int freq)
        {
            float prescaleval = 25000000.0f; // 25MHz
            prescaleval /= 4096.0f; // 12-bit
            prescaleval /= (float)freq;
            prescaleval -= 1.0f;

            Debug.WriteLine(
                "Setting PWM frequency to {0}Hz, Estimated pre-scale: {1}",
                freq,
                prescaleval);

            float prescale = (float)Math.Floor((double)prescaleval + 0.5);
            int effectiveFreq = (int)(25000000.0f / ((prescale + 1.0f) * 4096.0f));
            Debug.WriteLine("Final pre-scale: {0} with effective frequency {1}Hz", prescale, effectiveFreq);

            byte oldmode = ReadReg(MODE1);
            // Sleep to turn-off oscillator and disable Restart
            byte newmode = (byte)((oldmode & 0x7F) | 0x10);
            WriteReg(MODE1, newmode);

            WriteReg(PRESCALE, (byte)prescale);

            // Wake-up and enable oscillator
            WriteReg(MODE1, oldmode);
            // Enable Restart
            WriteReg(MODE1, (byte)(oldmode | 0x80));
        }

        public void SetChannelDutyCycle(int channel, float dutyCycle)
        {
            if (dutyCycle < 0f)
                dutyCycle = 0f;
            else if (dutyCycle > 1.0f)
                dutyCycle = 1.0f;

            if (channel < 0 || channel > 15)
            {
                Debug.WriteLine("Channel must be in the range [0,15]");
                return;
            }

            int onTime = (int)(dutyCycle * (float)PwmCounterMax);
            Debug.Assert(
                onTime >= 0 && onTime <= PwmCounterMax,
                "Channel signal on time must be in range [0,4095]");

            byte offset = (byte)(4 * channel);
            WriteReg((byte)(LED0_ON_L + offset), 0);
            WriteReg((byte)(LED0_ON_H + offset), 0);
            WriteReg((byte)(LED0_OFF_L + offset), (byte)(onTime & 0xFF));
            WriteReg((byte)(LED0_OFF_H + offset), (byte)(onTime >> 8));

            Debug.WriteLine(
                string.Format(
                    "Channel#{0} Duty={1} ON={2}",
                    channel,
                    dutyCycle,
                    onTime));
        }

        public void SetAllChannelsDutyCycle(float dutyCycle)
        {
            if (dutyCycle < 0f)
                dutyCycle = 0f;
            else if (dutyCycle > 1.0f)
                dutyCycle = 1.0f;

            int onTime = (int)(dutyCycle * (float)PwmCounterMax);
            Debug.Assert(
                onTime >= 0 && onTime <= PwmCounterMax,
                "Channel signal on time must be in range [0,4095]");

            WriteReg(ALL_LED_ON_L, 0);
            WriteReg(ALL_LED_ON_H, 0);
            WriteReg(ALL_LED_OFF_L, (byte)(onTime & 0xFF));
            WriteReg(ALL_LED_OFF_H, (byte)(onTime >> 8));

            Debug.WriteLine(
                string.Format(
                    "All Channel Duty={0} ON={1}",
                    dutyCycle,
                    onTime));
        }

        private void WriteReg(byte regAddr, byte val)
        {
            RegWriteBuff[0] = regAddr;
            RegWriteBuff[1] = val;
            Dev.Write(RegWriteBuff);
        }

        public void SoftwareReset()
        {
            Debug.WriteLine("Performing PCA9685 Software Reset ...");

            GeneralCallDev.Write(new byte[] { SWRST });
            Task.Delay(1);
        }

        private byte ReadReg(byte regAddr)
        {
            RegReadBuff[0] = regAddr;
            Dev.Read(RegReadBuff);
            return RegReadBuff[0];
        }

        public void Dispose()
        {
            if (Dev != null)
            {
                Dev.Dispose();
                Dev = null;
            }

            if (GeneralCallDev != null)
            {
                GeneralCallDev.Dispose();
                GeneralCallDev = null;
            }
        }

        byte[] RegWriteBuff = new byte[2];
        byte[] RegReadBuff = new byte[1];
        private I2cDevice Dev;
        private I2cDevice GeneralCallDev;
        private int SlaveAddress;
    }
}