//------------------------------------------------------------------------------
// <auto-generated>
//     This code was generated by a tool.
//     Runtime Version:4.0.30319.34209
//
//     Changes to this file may cause incorrect behavior and will be lost if
//     the code is regenerated.
// </auto-generated>
//------------------------------------------------------------------------------

namespace TouchscreenController_Hydra41 {
    using Gadgeteer;
    using GTM = Gadgeteer.Modules;
    
    
    public partial class Program : Gadgeteer.Program {
        
        /// <summary>The Load module using socket 15 of the mainboard.</summary>
        private Gadgeteer.Modules.GHIElectronics.Load load;
        
        /// <summary>The Extender module using socket 12 of the mainboard.</summary>
        private Gadgeteer.Modules.GHIElectronics.Extender extender_Motor_Servo;
        
        /// <summary>The SDCard module using socket 9 of the mainboard.</summary>
        private Gadgeteer.Modules.GHIElectronics.SDCard sdCard;
        
        /// <summary>The Extender module using socket 13 of the mainboard.</summary>
        private Gadgeteer.Modules.GHIElectronics.Extender extender_Battery_Input;
        
        /// <summary>The XBee Adapter module using socket 11 of the mainboard.</summary>
        private Gadgeteer.Modules.GHIElectronics.XBeeAdapter lairdWireless;
        
        /// <summary>The Extender module using socket 3 of the mainboard.</summary>
        private Gadgeteer.Modules.GHIElectronics.Extender ALCAM_SPI;
        
        /// <summary>The Extender module using socket 4 of the mainboard.</summary>
        private Gadgeteer.Modules.GHIElectronics.Extender OHS_MULTI_COM;
        
        /// <summary>This property provides access to the Mainboard API. This is normally not necessary for an end user program.</summary>
        protected new static GHIElectronics.Gadgeteer.FEZRaptor Mainboard {
            get {
                return ((GHIElectronics.Gadgeteer.FEZRaptor)(Gadgeteer.Program.Mainboard));
            }
            set {
                Gadgeteer.Program.Mainboard = value;
            }
        }
        
        /// <summary>This method runs automatically when the device is powered, and calls ProgramStarted.</summary>
        public static void Main() {
            // Important to initialize the Mainboard first
            Program.Mainboard = new GHIElectronics.Gadgeteer.FEZRaptor();
            Program p = new Program();
            p.InitializeModules();
            p.ProgramStarted();
            // Starts Dispatcher
            p.Run();
        }
        
        private void InitializeModules() {
            this.load = new GTM.GHIElectronics.Load(15);
            this.extender_Motor_Servo = new GTM.GHIElectronics.Extender(12);
            this.sdCard = new GTM.GHIElectronics.SDCard(9);
            this.extender_Battery_Input = new GTM.GHIElectronics.Extender(13);
            this.lairdWireless = new GTM.GHIElectronics.XBeeAdapter(11);
            this.ALCAM_SPI = new GTM.GHIElectronics.Extender(3);
            this.OHS_MULTI_COM = new GTM.GHIElectronics.Extender(4);
        }
    }
}
