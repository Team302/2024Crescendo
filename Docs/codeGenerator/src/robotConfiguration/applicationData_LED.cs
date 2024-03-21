using Configuration;
using DataConfiguration;
using System;
using System.Collections;
using System.Collections.Generic;
using System.ComponentModel;
using System.ComponentModel.DataAnnotations;
using System.Linq;
using System.Reflection;
using System.Runtime.InteropServices;
using System.Runtime.Remoting.Channels;
using System.Security.AccessControl;
using System.Security.Authentication.ExtendedProtection;
using System.Security.Cryptography;
using System.Security.Policy;
using System.Text;
using System.Xml.Linq;
using System.Xml.Serialization;
using static ApplicationData.generatorContext;
using static ApplicationData.motorControlData;
using static ApplicationData.TalonFX;
using static ApplicationData.TalonSRX;
using static System.Net.Mime.MediaTypeNames;

namespace ApplicationData
{
    [Serializable()]
    [UserIncludeFile("hw/DragonLeds.h")]
    public class Led : baseRobotElementClass
    {
        [DefaultValue(0u)]
        [Range(typeof(uint), "0", "9")]
        public uintParameter PwmId { get; set; }

        public List<LedSegment> Segments { get; set; }

        public Led()
        {
        }

        public override string getDisplayName(string propertyName, out helperFunctions.RefreshLevel refresh)
        {
            refresh = helperFunctions.RefreshLevel.none;
            return "LEDs";
        }
    }

    [Serializable()]
    public class LedSegment : baseRobotElementClass
    {
        [DefaultValue(0u)]
        public uintParameter Count { get; set; }

        public LedSegment()
        {
        }
    }
}
