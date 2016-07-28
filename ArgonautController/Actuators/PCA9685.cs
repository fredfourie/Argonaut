using System;
using System.Diagnostics;
using System.Threading.Tasks;

namespace ArgonautController.Actuators
{
    using Windows.Devices.Enumeration;
    using Windows.Devices.I2c;

    public class PCA9685 : IDisposable
    {
        // Device-specific constants
        // See "PCA9685 I2C LED Controller specs.pdf" in Resources folder
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

        // IO buffers
        byte[] RegWriteBuff = new byte[2];
        byte[] RegReadBuff = new byte[1];

        // Global device instances
        private I2cDevice Dev;
        private I2cDevice GeneralCallDev;

        // I2C bus address
        private int SlaveAddress = -1;
        
        /// <summary>
        /// Creates a new instance of the <see cref="PCA9685"/> class.
        /// </summary>
        /// <param name="slaveAddress">Index of the I2C device to initiate against.</param>
        public PCA9685(int slaveAddress)
        {
            SlaveAddress = slaveAddress;
        }

        /// <summary>
        /// Initializes the I2C connection and channels.
        /// </summary>
        /// <returns>A <see cref="Task"/> instance which can be awaited on for completion.</returns>
        public async Task Init()
        {
            if (SlaveAddress < 0) throw new Exception("Invalid SlaveAddress value configured. Please call the class constructor with a valid I2C bus slave address.");
            Debug.WriteLine("Initializing PCA9685 against slave address {0}.", SlaveAddress);

            string aqs = I2cDevice.GetDeviceSelector();
            var i2cDevices = await DeviceInformation.FindAllAsync(aqs);
            if (i2cDevices.Count == 0)
            {
                Debug.WriteLine("No I2C controllers were found on the system.");
                return;
            }

            var settings = new I2cConnectionSettings(GeneralCallSlaveAddress);
            settings.BusSpeed = I2cBusSpeed.FastMode;
            GeneralCallDev = await I2cDevice.FromIdAsync(i2cDevices[0].Id, settings);
            if (GeneralCallDev == null)
            {
                var errorMessage =  string.Format(
                        "Slave address {0} on I2C Controller {1} is currently in use by another application.",
                        settings.SlaveAddress,
                        i2cDevices[0].Id);

                throw new Exception(errorMessage);
            }

            SoftwareReset();

            settings = new I2cConnectionSettings(SlaveAddress);
            settings.BusSpeed = I2cBusSpeed.FastMode;
            Dev = await I2cDevice.FromIdAsync(i2cDevices[0].Id, settings);
            if (Dev == null)
            {
                var errorMessage = string.Format(
                        "Slave address {0} on I2C Controller {1} is currently in use by another application.",
                        settings.SlaveAddress,
                        i2cDevices[0].Id);

                throw new Exception(errorMessage);
            }

            Debug.WriteLine("PCA9685 I2C channels created.");

            SetAllChannelsDutyCycle(0.0f);

            // Output drive mode is totem-pole not open drain
            WriteReg(MODE2, OUTDRV);

            // Turn-off oscillator and acknowledge All-Call transfers
            WriteReg(MODE1, ALLCALL);
            await Task.Delay(1);

            byte mode1 = ReadReg(MODE1);
            // Turn on oscillator
            mode1 &= unchecked((byte)~SLEEP);
            WriteReg(MODE1, mode1);
            await Task.Delay(1);

            Debug.WriteLine("PCA9685 initialization complete.");
        }

        /// <summary>
        /// Sets the pulse width modulation (PWM) frequency.
        /// </summary>
        /// <param name="frequency">The desired frequency in Hertz.</param>
        public void SetPwmFrequency(int frequency)
        {
            float prescaleVal = 25000000f / (4096f * (float)frequency) - 1f;
            Debug.WriteLine(
                "Setting PWM frequency to {0} Hz; Estimated pre-scale: {1}",
                frequency,
                prescaleVal);

            float prescale = (float)Math.Floor((double)prescaleVal + 0.5);
            int effectiveFreq = (int)(25000000f / ((prescale + 1f) * 4096f));
            Debug.WriteLine("Final pre-scale: {0} with effective frequency {1} Hz", prescale, effectiveFreq);

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

        /// <summary>
        /// Sets the PWM duty cycle for a specified channel.
        /// </summary>
        /// <param name="channel">The channel number</param>
        /// <param name="dutyCycle">The desired duty cycle</param>
        public void SetChannelDutyCycle(int channel, float dutyCycle)
        {
            if (channel < 0 || channel > 15)
            {
                throw new ArgumentException("Channel must be in the range [0,15]");
            }

            // Clamp the dutyCycle to the range [0,1]
            if      (dutyCycle < 0f)   dutyCycle = 0f;
            else if (dutyCycle > 1.0f) dutyCycle = 1f;

            int onTime = (int)(dutyCycle * PwmCounterMax);
            Debug.Assert(
                onTime >= 0 && onTime <= PwmCounterMax,
                string.Format("Channel signal on time must be in range [0,{0}]", PwmCounterMax));

            // Write values to registers
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

        /// <summary>
        /// Sets the dudty cycle for all channels.
        /// </summary>
        /// <param name="dutyCycle">The desired duty cycle</param>
        public void SetAllChannelsDutyCycle(float dutyCycle)
        {
            // Clamp the dutyCycle to the range [0,1]
            if (dutyCycle < 0f) dutyCycle = 0f;
            else if (dutyCycle > 1.0f) dutyCycle = 1f;

            int onTime = (int)(dutyCycle * PwmCounterMax);
            Debug.Assert(
                onTime >= 0 && onTime <= PwmCounterMax,
                string.Format("Channel signal on time must be in range [0,{0}]", PwmCounterMax));

            // Write values to registers
            WriteReg(ALL_LED_ON_L, 0);
            WriteReg(ALL_LED_ON_H, 0);
            WriteReg(ALL_LED_OFF_L, (byte)(onTime & 0xFF));
            WriteReg(ALL_LED_OFF_H, (byte)(onTime >> 8));

            Debug.WriteLine(
                string.Format(
                    "All Channels: Duty={0} ON={1}",
                    dutyCycle,
                    onTime));
        }

        /// <summary>
        /// Writes a given value to a specified register.
        /// </summary>
        /// <param name="registerAddress">The targeted register address</param>
        /// <param name="value">The desired value to write</param>
        private void WriteReg(byte registerAddress, byte value)
        {
            RegWriteBuff[0] = registerAddress;
            RegWriteBuff[1] = value;
            Dev.Write(RegWriteBuff);
        }

        /// <summary>
        /// Performs a software reset of the PCA9685 controller
        /// </summary>
        public void SoftwareReset()
        {
            Debug.WriteLine("Performing PCA9685 software reset...");

            GeneralCallDev.Write(new byte[] { SWRST });
            Task.Delay(1);
        }

        /// <summary>
        /// Reads the value from a specified register.
        /// </summary>
        /// <param name="registerAddress">The address of the targeted register</param>
        /// <returns></returns>
        private byte ReadReg(byte registerAddress)
        {
            RegReadBuff[0] = registerAddress;
            Dev.Read(RegReadBuff);
            return RegReadBuff[0];
        }

        /// <summary>
        /// Cleans of the device resources.
        /// </summary>
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
    }
}