using Windows.UI.Xaml.Controls;
using System.Threading.Tasks;
using System.Diagnostics;
using Actuation;
using System.Threading;


// The Blank Page item template is documented at http://go.microsoft.com/fwlink/?LinkId=402352&clcid=0x409

namespace ActuationTest
{
    /// <summary>
    /// An empty page that can be used on its own or navigated to within a Frame.
    /// </summary>
    public sealed partial class MainPage : Page
    {
        public MainPage()
        {
            this.InitializeComponent();
            Unloaded += MainPage_Unloaded;
            Loaded += MainPage_Loaded;

            Config = new ZumoMotorShieldConfig();
            Config.LeftMotorDirPin = 4;
            Config.RightMotorDirPin = 5;
            Config.LeftPwmChannel = 0;
            Config.RightPwmChannel = 1;
            Config.PwmDriverSlaveAddress = 0x40;

            MotorDriver = new ZumoMotorShield(Config);
        }

        private async void MainPage_Loaded(object sender, object args)
        {
            await MotorDriver.Init();
            await MotorControlSmokeTest();
        }

        private async Task MotorControlSmokeTest()
        {
            bool flipDir = false;
            ZumoMotorDirection dirA = ZumoMotorDirection.Forward;
            ZumoMotorDirection dirB = ZumoMotorDirection.Backward;

            for (int i = 20; i <= 100; i += 20)
            {
                Debug.WriteLine("Motor Control Ticking");

                if (!flipDir)
                {
                    MotorDriver.SetLeftMotorPower(dirA, (float)i / 100.0f);
                    MotorDriver.SetRightMotorPower(dirB, (float)i / 100.0f);
                }
                else
                {
                    MotorDriver.SetLeftMotorPower(dirB, (float)i / 100.0f);
                    MotorDriver.SetRightMotorPower(dirA, (float)i / 100.0f);
                }

                flipDir = !flipDir;

                await Task.Delay(1500);
            }

            MotorDriver.LeftMotorStop();
            MotorDriver.RightMotorStop();

            await Task.Delay(1000);

            MotorDriver.SetLeftMotorPower(ZumoMotorDirection.Forward, 0.5f);
            await Task.Delay(2000);
            MotorDriver.LeftMotorStop();

            await Task.Delay(500);

            MotorDriver.SetRightMotorPower(ZumoMotorDirection.Forward, 0.5f);
            await Task.Delay(2000);
            MotorDriver.RightMotorStop();
        }

        private void MainPage_Unloaded(object sender, object args)
        {
            if (MotorDriver != null)
            {
                MotorDriver.Dispose();
                MotorDriver = null;
            }
        }

        ZumoMotorShield MotorDriver;
        ZumoMotorShieldConfig Config;
    }
}
