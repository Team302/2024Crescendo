using Configuration;
using DataConfiguration;
using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.ComponentModel.DataAnnotations;
using System.Xml.Serialization;

// =================================== Rules =====================================
// A property named __units__ will be converted to the list of physical units
// A property named value__ will not be shown in the tree directly. Its value is shown in the parent node


// Attributes are only allowed on the standard types (uint, int, double, bool) and on doubleParameter, unitParameter, intParameter, boolParameter
// The attribute PhysicalUnitsFamily can only be applied on doubleParameter, uintParameter, intParameter, boolParameter


namespace ApplicationData
{
    public partial class applicationData
    {
#if enableTestAutomation
        public testClass testClass { get; set; }
        public List<doubleParameterUserDefinedTunable> parameter { get; set; }
        public List<testMotor> motor { get; set; }
        public testpdp pdp { get; set; }
        public List<mechanismInstance> mechanismInstance { get; set; }

        [DefaultValue(1u)]
        [Range(typeof(uint), "1", "9999")]
        public uintParameter robotID { get; set; }

        public applicationData()
        {
            helperFunctions.initializeNullProperties(this);
            helperFunctions.initializeDefaultValues(this);
        }

        public string getDisplayName(string propertyName, out helperFunctions.RefreshLevel refresh)
        {
            refresh = helperFunctions.RefreshLevel.none;

            if (string.IsNullOrEmpty(propertyName))
                return string.Format("Robot #{0}", robotID.value);
            else if (propertyName == "testClass")
                return string.Format("{0} ({1})", propertyName, testClass.name);
            //else if (propertyName == "pdp")
            //    return string.Format("{0} ({1})", propertyName, pdp.type);

            return "robot class - incomplete getDisplayName";
        }
#endif
    }

    public partial class mechanism
    {
#if enableTestAutomation
        public List<testClosedLoopControlParameters> testClosedLoopControlParameters { get; set; }
        public List<testMotor> testMotor { get; set; }

        public mechanism()
        {
            GUID = Guid.NewGuid();

            helperFunctions.initializeNullProperties(this);

            name = GetType().Name;

            helperFunctions.initializeDefaultValues(this);
        }

        public string getDisplayName()
        {
            return string.Format("{0}", name);
        }
#endif
    }

    [Serializable()]
    public class testClosedLoopControlParameters
    {
        [ConstantInMechInstance]
        public string name { get; set; }

        [DefaultValue(0D)]
        [System.ComponentModel.Description("The proportional gain of the PID controller.")]
        [TunableParameter()]
        public doubleParameter pGain { get; set; }

        [DefaultValue(0D)]
        [System.ComponentModel.Description("The integral gain of the PID controller.")]
        [TunableParameter()]
        public doubleParameter iGain { get; set; }

        [DefaultValue(0D)]
        [System.ComponentModel.Description("The differential gain of the PID controller.")]
        [TunableParameter()]
        public doubleParameter dGain { get; set; }

        [DefaultValue(0D)]
        [System.ComponentModel.Description("The feed forward gain of the PID controller.")]
        [TunableParameter()]
        public doubleParameter fGain { get; set; }

        [DefaultValue(0D)]
        [TunableParameter()]
        public doubleParameter iZone { get; set; }

        public testClosedLoopControlParameters()
        {
            helperFunctions.initializeNullProperties(this);
            helperFunctions.initializeDefaultValues(this);

            name = GetType().Name;

        }

        public string getDisplayName()
        {
            return string.Format("{0}", name);
        }
    }

    [Serializable()]
    public class testpdp
    {
        public enum pdptype { CTRE, REV, }

        [DefaultValue(pdptype.REV)]
        public pdptype type { get; set; }

        public testpdp()
        {
            helperFunctions.initializeNullProperties(this);
            helperFunctions.initializeDefaultValues(this);
        }

        public string getDisplayName(string propertyName, out helperFunctions.RefreshLevel refresh)
        {
            refresh = helperFunctions.RefreshLevel.parentHeader;
            if (string.IsNullOrEmpty(propertyName))
                refresh = helperFunctions.RefreshLevel.none;
            return "Pdp (" + type.ToString() + ")";
        }
    }

    [Serializable()]
    [XmlInclude(typeof(testFalcon_Motor))]
    [XmlInclude(typeof(testTalonSRX_Motor))]
    public class testMotor
    {
        [XmlIgnore]
        [Constant()]
        public string motorType { get; protected set; }

        [ConstantInMechInstance]
        public string name { get; set; }

        [DefaultValue(0u)]
        [Range(typeof(uint), "0", "62")]
        public uintParameter CAN_ID { get; set; }

        public testMotor()
        {
            helperFunctions.initializeNullProperties(this);

            string temp = this.GetType().Name;
            motorType = temp.Substring(0, temp.LastIndexOf('_'));
            name = motorType;

            helperFunctions.initializeDefaultValues(this);
        }
        public string getDisplayName()
        {
            return name;
        }
    }

    [Serializable()]
    public class testFalcon_Motor : testMotor
    {
        [DefaultValue(1.15)]
        [Range(typeof(double), "0", "62")]
        [PhysicalUnitsFamily(physicalUnit.Family.percent)]
        [TunableParameter()]
        public doubleParameter deadbandPercent { get; set; }

        [DefaultValue(5.55)]
        [Range(typeof(double), "0", "100")]
        [PhysicalUnitsFamily(physicalUnit.Family.percent)]
        [TunableParameter()]
        public doubleParameter deadband { get; set; }

        [DefaultValue(2.2)]
        [Range(typeof(double), "-1.0", "3.0")]
        [PhysicalUnitsFamily(physicalUnit.Family.current)]
        [TunableParameter()]
        public doubleParameter peakMin { get; set; }

        [DefaultValue(4.4)]
        [Range(typeof(double), "-10.0", "20.0")]
        [PhysicalUnitsFamily(physicalUnit.Family.current)]
        public doubleParameter peakMax { get; set; }

        public testFalcon_Motor()
        {
        }
    }

    [Serializable()]
    public class testTalonSRX_Motor : testMotor
    {
        [DefaultValue(1.1)]
        [Range(typeof(double), "0", "62")]
        [TunableParameter()]
        public doubleParameter deadbandPercent_ { get; set; }

        [DefaultValue(2.2)]
        [Range(typeof(double), "-1.0", "3.0")]
        public doubleParameter peakMin_ { get; set; }

        [DefaultValue(4.4)]
        [Range(typeof(double), "-10.0", "20.0")]
        [TunableParameter()]
        public doubleParameter peakMax_ { get; set; }

        public testTalonSRX_Motor()
        {
        }
    }



    [Serializable()]
    public class testClass
    {
        public string name { get; set; }

        public doubleParameterUserDefinedTunable myDoubleTunable { get; set; }

        [DefaultValue(-4)]
        [Range(typeof(double), "-10", "10")]
        [PhysicalUnitsFamily(physicalUnit.Family.length)]
        [TunableParameter()]
        public doubleParameter aDouble { get; set; }

        [PhysicalUnitsFamily(physicalUnit.Family.mass)]
        [TunableParameter()]
        public doubleParameter anotherDouble { get; set; }

        public List<doubleParameterUserDefinedNonTunable> aListOfDoubles { get; set; }

        [TunableParameter()]
        public List<doubleParameterUserDefinedTunable> aListOfTunableDoubles { get; set; }


        [DefaultValue(-40)]
        [Range(typeof(int), "-100", "10")]
        [PhysicalUnitsFamily(physicalUnit.Family.mass)]
        [TunableParameter()]
        public intParameter anInt { get; set; }

        [PhysicalUnitsFamily(physicalUnit.Family.voltage)]
        [TunableParameter()]
        public intParameter anotherInt { get; set; }

        public List<intParameterUserDefinedNonTunable> aListOfInts { get; set; }

        [TunableParameter()]
        public List<intParameterUserDefinedTunable> aListOfTunableInts { get; set; }

        [DefaultValue(40)]
        [Range(typeof(int), "0", "43")]
        [PhysicalUnitsFamily(physicalUnit.Family.mass)]
        [TunableParameter()]
        public uintParameter aUint { get; set; }

        [PhysicalUnitsFamily(physicalUnit.Family.current)]
        [TunableParameter()]
        public uintParameter anotherUint { get; set; }

        public List<uintParameterUserDefinedNonTunable> aListOfUints { get; set; }

        public List<uintParameterUserDefinedTunable> aListOfTunableUints { get; set; }

        public boolParameter aBool { get; set; }

        [TunableParameter]
        public boolParameter aTunableBool { get; set; }

        public List<boolParameterUserDefinedNonTunable> aListOfNonTunableBools { get; set; }
        public List<boolParameterUserDefinedTunable> aListOfTunableBools { get; set; }


        public testClass()
        {
            aDouble = new doubleParameter();
            anotherDouble = new doubleParameter();
            aListOfDoubles = new List<doubleParameterUserDefinedNonTunable>();
            aListOfTunableDoubles = new List<doubleParameterUserDefinedTunable>();
            myDoubleTunable = new doubleParameterUserDefinedTunable();

            anInt = new intParameter();
            anotherInt = new intParameter();
            aListOfInts = new List<intParameterUserDefinedNonTunable>();
            aListOfTunableInts = new List<intParameterUserDefinedTunable>();

            aUint = new uintParameter();
            anotherUint = new uintParameter();
            aListOfUints = new List<uintParameterUserDefinedNonTunable>();
            aListOfTunableUints = new List<uintParameterUserDefinedTunable>();

            aBool = new boolParameter();
            aTunableBool = new boolParameter();
            aListOfNonTunableBools = new List<boolParameterUserDefinedNonTunable>();
            aListOfTunableBools = new List<boolParameterUserDefinedTunable>();

            name = this.GetType().Name;
            helperFunctions.initializeDefaultValues(this);
        }

        public string getDisplayName(string propertyName, out helperFunctions.RefreshLevel refresh)
        {
            refresh = helperFunctions.RefreshLevel.none;

            if (string.IsNullOrEmpty(propertyName))
            {
                return name;
            }
            else if (propertyName == "aDouble")
            {
                return aDouble.getDisplayName(propertyName, out refresh);
            }
            else
            {
                return propertyName;
            }
        }
    }

}
