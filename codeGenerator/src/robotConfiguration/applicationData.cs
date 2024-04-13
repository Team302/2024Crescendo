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

//todo handle optional elements such as followID in a motorcontroller
//todo the range of pdpID for ctre is 0-15, for REV it is 0-19. How to adjust the range allowed in the GUI. If initially REV is used and an id > 15 is used, then user chooses CTRE, what to do?
//todo make mechanism instances separate files so that it is easier for multiple people to work on the robot in parallel
//todo run a sanity check on a click of a button or on every change?
//todo in the treeview, place the "name" nodes at the top
//todo in the robot code, check that an enum belonging to another robot is not used
//todo check naming convention
//todo getDisplayName gets called multiple times when a solenoid name is changed in a mechanism
//todo handle DistanceAngleCalcStruc should this be split into 2 separate structs? one ofr dist , 2nd for angle?
//todo when mechanisms are renamed, the GUIDs get messed up
//todo if a decorator mod file exists, do not write it
//todo show the DataDescription information
//todo target physical units should not be editable in the mechanism instance
//todo add DataDescription for the robot elements
//todo zoom so that the text is larger
//todo handle chassis like a special mechanism

// =================================== Rules =====================================
// A property named __units__ will be converted to the list of physical units
// A property named value__ will not be shown in the tree directly. Its value is shown in the parent node
// Attributes are only allowed on the standard types (uint, int, double, bool) and on doubleParameter, unitParameter, intParameter, boolParameter
// The attribute PhysicalUnitsFamily can only be applied on doubleParameter, uintParameter, intParameter, boolParameter
// A class can only contain one List of a particular type

namespace ApplicationData
{
    #region general enums
    [Serializable()]
    public enum CAN_BUS
    {
        rio,
        canivore
    }

    [Serializable()]
    public enum MotorType
    {

        TALONSRX,

        FALCON,

        BRUSHLESS_SPARK_MAX,

        BRUSHED_SPARK_MAX,

        FALCON500,

        NEOMOTOR,

        NEO500MOTOR,

        CIMMOTOR,

        MINICIMMOTOR,

        BAGMOTOR,

        PRO775,

        ANDYMARK9015,

        ANDYMARKNEVEREST,

        ANDYMARKRS775125,

        ANDYMARKREDLINEA,

        REVROBOTICSHDHEXMOTOR,

        BANEBOTSRS77518V,

        BANEBOTSRS550,

        MODERNROBOTICS12VDCMOTOR,

        JOHNSONELECTRICALGEARMOTOR,

        TETRIXMAXTORQUENADOMOTOR,
    }

    [Serializable()]
    public enum motorFeedbackDevice
    {

        NONE,

        INTERNAL,

        QUADENCODER,

        ANALOG,

        TACHOMETER,

        PULSEWIDTHENCODERPOSITION,

        SENSORSUM,

        SENSORDIFFERENCE,

        REMOTESENSOR0,

        REMOTESENSOR1,

        SOFTWAREEMULATEDSENSOR,
    }
    #endregion

    [Serializable()]
    public class topLevelAppDataElement
    {
        public List<string> robotFiles = new List<string>();

        [DataDescription("The robot definitions")]
        public List<applicationData> Robots { get; set; }

        [DataDescription("The mechanism templates")]
        [DataDescription("How to build a mechanism:")]
        [DataDescription("  1) Define all the motors")]
        [DataDescription("  2) Define the digital inputs")]
        [DataDescription("  3) Define the control datas")]
        [DataDescription("  4) Define all the states")]
        [DataDescription("  5) Select states and push \"Configure States\"")]
        [DataDescription("  6) In each state select the correct control datas")]
        [DataDescription("  7) Specify the transitions")]
        [DataDescription("  8) Generate")]
        [DataDescription("  9) Fill in the isTransition functions")]
        public List<mechanism> Mechanisms { get; set; }

        public topLevelAppDataElement()
        {

            helperFunctions.initializeNullProperties(this);
            helperFunctions.initializeDefaultValues(this);
        }

        public string getDisplayName()
        {
            return "robotBuildDefinition";
        }
    }

    [Serializable()]
    public partial class applicationData
#if !enableTestAutomation
    {

        [DataDescription("One power distribution panel can be configured for a robot")]
        public pdp PowerDistributionPanel { get; set; }

        [DataDescription("A robot can contain multiple pneumatic control modules")]
        public List<pcm> PneumaticControlModules { get; set; }

        [DataDescription("A robot can have one chassis definition")]
        public chassis Chassis { get; set; }

        [DataDescription("A robot can contain multiple mechanism instances")]
        public List<mechanismInstance> mechanismInstances { get; set; }

        [DataDescription("A robot can contain multiple cameras")]
        public List<Camera> Cameras { get; set; }

        [DataDescription("A robot can contain multiple roborios")]
        public List<roborio> Roborios { get; set; }

        [DataDescription("A robot can contain multiple LED setups")]

        public Led LEDs { get; set; }

        [DefaultValue(1u)]
        [Range(typeof(uint), "1", "9999")]
        [DataDescription("The robot number.")]
        [DataDescription("The competition robot should be set to 302")]
        public uintParameter robotID { get; set; }

        public string name { get; set; } = "Example";

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
            //else if (propertyName == "testClass")
            //    return string.Format("{0} ({1}))", propertyName, testClass.name);
            else if (propertyName == "pdp")
                return string.Format("{0} ({1})", propertyName, PowerDistributionPanel.type);
            else if (propertyName == "name")
                return this.name;

            return "robot class - incomplete getDisplayName";
        }

        public string getFullRobotName()
        {
            return string.Format("{0}{1}", name, robotID.value);
        }

        public List<string> generate(string generateFunctionName)
        {
            List<string> sb = new List<string>();

            PropertyInfo[] propertyInfos = this.GetType().GetProperties();
            foreach (PropertyInfo pi in propertyInfos) // add its children
            {
                if (baseDataConfiguration.isACollection(pi.PropertyType))
                {
                    object theObject = pi.GetValue(this);
                    if (theObject != null)
                    {
                        Type elementType = theObject.GetType().GetGenericArguments().Single();
                        ICollection ic = theObject as ICollection;
                        foreach (var v in ic)
                        {
                            if (v != null)
                            {
                                sb.AddRange(generate(v, generateFunctionName));
                            }
                        }
                    }
                }
                else
                {
                    object theObject = pi.GetValue(this);
                    if (theObject != null)
                        sb.AddRange(generate(theObject, generateFunctionName));
                }
            }

            return sb;
        }

        private List<string> generate(object obj, string generateFunctionName)
        {
            MethodInfo mi = obj.GetType().GetMethod(generateFunctionName);
            if (mi != null)
            {
                object[] parameters = new object[] { };
                return (List<string>)mi.Invoke(obj, parameters);
            }

            return new List<string>();
        }
#endif
    }

    [Serializable()]
    public class mechanismInstance
    {
        [XmlIgnore]
        public object theTreeNode = null;

        public string name { get; set; }

        public mechanism mechanism { get; set; }

        public mechanismInstance()
        {
            name = "mechanismInstanceName";
            helperFunctions.initializeNullProperties(this);
            helperFunctions.initializeDefaultValues(this);
        }

        public string getDisplayName(string propertyName, out helperFunctions.RefreshLevel refresh)
        {
            refresh = helperFunctions.RefreshLevel.parentHeader;

            if (propertyName == "")
                return name;

            PropertyInfo pi = this.GetType().GetProperty(propertyName);
            if (pi != null)
            {
                object value = pi.GetValue(this);
                return string.Format("{0} ({1})", propertyName, value.ToString());
            }

            return null;
        }

        public List<string> generate(string generateFunctionName)
        {
            List<string> sb = new List<string>();
            object obj = this.mechanism;

            PropertyInfo[] propertyInfos = obj.GetType().GetProperties();
            foreach (PropertyInfo pi in propertyInfos) // add its children
            {
                if (baseDataConfiguration.isACollection(pi.PropertyType))
                {
                    sb.Add(""); // this creates an empty line in the output file, to increase readability

                    object theObject = pi.GetValue(obj);
                    if (theObject != null)
                    {
                        Type elementType = theObject.GetType().GetGenericArguments().Single();

                        if ((generateFunctionName == "generateIndexedObjectCreation") && (elementType == typeof(state)))
                            continue;

                        ICollection ic = theObject as ICollection;
                        int index = 0;
                        foreach (var v in ic)
                        {
                            if (v != null)
                            {
                                sb.AddRange(generate(v, generateFunctionName, index));
                            }
                            index++;
                        }
                    }
                }
                else
                {
                    object theObject = pi.GetValue(obj);
                    if (theObject != null)
                        sb.AddRange(generate(theObject, generateFunctionName));
                }
            }

            return sb;
        }

        private List<string> generate(object obj, string generateFunctionName, int currentIndex)
        {
            MethodInfo mi = obj.GetType().GetMethod(generateFunctionName);
            ParameterInfo[] pi = mi.GetParameters();

            if (pi.Length == 0)
                return (List<string>)mi.Invoke(obj, new object[] { });
            else if (pi.Length == 1)
            {
                object[] parameters = new object[] { currentIndex };
                return (List<string>)mi.Invoke(obj, parameters);
            }

            return new List<string>();
        }
        private List<string> generate(object obj, string generateFunctionName)
        {
            MethodInfo mi = obj.GetType().GetMethod(generateFunctionName);
            if (mi != null)
            {
                object[] parameters = new object[] { };
                return (List<string>)mi.Invoke(obj, parameters);
            }

            return new List<string>();
        }

        public string getIncludePath()
        {
            return String.Format("mechanisms/{0}/decoratormods/{0}.h", name);
        }
    }

    [Serializable()]
    public partial class mechanism
    {
        [XmlIgnore]
        public object theTreeNode = null;

        public Guid GUID;

        [ConstantInMechInstance]
        public string name { get; set; }

        public override string ToString()
        {
            return name;
        }
#if !enableTestAutomation
        public List<MotorController> MotorControllers { get; set; }
        public List<PIDFZ> closedLoopControlParameters { get; set; }
        public List<solenoid> solenoid { get; set; }
        public List<servo> servo { get; set; }
        public List<analogInput> analogInput { get; set; }
        public List<digitalInput> digitalInput { get; set; }
        // not defined in /hw/Dragon.. public List<colorSensor> colorSensor { get; set; }
        public List<CANcoder> cancoder { get; set; }

        public List<motorControlData> stateMotorControlData { get; set; }
        public List<state> states { get; set; }

        public mechanism()
        {
            if ((GUID == null) || (GUID == new Guid()))
                GUID = Guid.NewGuid();

            helperFunctions.initializeNullProperties(this);

            name = GetType().Name;

            helperFunctions.initializeDefaultValues(this);
        }

        public string getDisplayName(string propertyName, out helperFunctions.RefreshLevel refresh)
        {
            refresh = helperFunctions.RefreshLevel.none;
            if (propertyName == "name")
                refresh = helperFunctions.RefreshLevel.parentHeader;

            return string.Format("{0}", name);
        }

        public List<string> generate(string generateFunctionName)
        {
            List<string> sb = new List<string>();

            PropertyInfo[] propertyInfos = this.GetType().GetProperties();
            foreach (PropertyInfo pi in propertyInfos) // add its children
            {
                if (baseDataConfiguration.isACollection(pi.PropertyType))
                {
                    object theObject = pi.GetValue(this);
                    if (theObject != null)
                    {
                        Type elementType = theObject.GetType().GetGenericArguments().Single();
                        ICollection ic = theObject as ICollection;
                        int index = 0;
                        foreach (var v in ic)
                        {
                            if (v != null)
                            {
                                sb.AddRange(generate(v, generateFunctionName, index));
                            }
                            index++;
                        }
                    }
                }
                else
                {
                    object theObject = pi.GetValue(this);
                    if (theObject != null)
                        sb.AddRange(generate(theObject, generateFunctionName));
                }
            }

            return sb;
        }

        private List<string> generate(object obj, string generateFunctionName, int currentIndex)
        {
            MethodInfo mi = obj.GetType().GetMethod(generateFunctionName);
            ParameterInfo[] pi = mi.GetParameters();

            if (pi.Length == 0)
                return (List<string>)mi.Invoke(obj, new object[] { });
            else if (pi.Length == 2)
            {
                object[] parameters = new object[] { currentIndex };
                return (List<string>)mi.Invoke(obj, parameters);
            }

            return new List<string>();
        }

        private List<string> generate(object obj, string generateFunctionName)
        {
            MethodInfo mi = obj.GetType().GetMethod(generateFunctionName);
            if (mi != null)
            {
                object[] parameters = new object[] { };
                return (List<string>)mi.Invoke(obj, parameters);
            }

            return new List<string>();
        }
#endif
    }

    [Serializable]
    public class CANcoderInstance : baseRobotElementClass
    {

    }

#if !enableTestAutomation
    [Serializable()]
    public class PID : baseRobotElementClass
    {
        [DefaultValue(0D)]
        [DataDescription("The proportional gain of the PIDF controller.")]
        [TunableParameter()]
        public doubleParameter pGain { get; set; }

        [DefaultValue(0D)]
        [DataDescription("The integral gain of the PIDF controller.")]
        [TunableParameter()]
        public doubleParameter iGain { get; set; }

        [DefaultValue(0D)]
        [DataDescription("The differential gain of the PIDF controller.")]
        [TunableParameter()]
        public doubleParameter dGain { get; set; }
        public PID()
        {
        }
    }

    [Serializable()]
    public class PIDF : PID
    {
        [DefaultValue(0D)]
        [DataDescription("The feed forward gain of the PIDF controller.")]
        [TunableParameter()]
        public doubleParameter fGain { get; set; }

        public PIDF()
        {
        }
    }

    [Serializable()]
    public class PIDFslot : PIDF
    {

        [DefaultValue(0D)]
        [DataDescription("The slot to store the PIDF settings.")]
        [Range(0, 3)]
        public intParameter slot { get; set; }

        public PIDFslot()
        {
        }
    }

    [Serializable()]
    public class PIDFZ : PIDF
    {
        [DefaultValue(0D)]
        [TunableParameter()]
        public doubleParameter iZone { get; set; }

        public PIDFZ()
        {
        }
    }

    [Serializable()]
    [DataDescription("The Power distribution panel. A robot can only contain 1 PDP")]
    public class pdp : baseRobotElementClass
    {
        public enum pdptype { CTRE, REV, }

        [DefaultValue(pdptype.REV)]
        public pdptype type { get; set; }

        [DefaultValue(0u)]
        [Range(typeof(uint), "0", "62")]
        [DataDescription("The ID that is used to form the device CAD ID")]
        [DataDescription("ID 0 is normally reserved for the roborio")]
        public uintParameter canID { get; set; }

        public pdp()
        {

        }
    }

    [Serializable()]
    public class pigeon : baseRobotElementClass
    {
        public enum pigeonType
        {
            pigeon1,
            pigeon2,
        }

        public enum pigeonPosition
        {
            CENTER_OF_ROTATION,
        }

        [DefaultValue(0u)]
        [Range(typeof(uint), "0", "62")]
        [DataDescription("The ID that is used to form the device CAD ID")]
        [DataDescription("ID 0 is normally reserved for the roborio")]
        public uintParameter canID { get; set; }

        [DefaultValue(CAN_BUS.rio)]
        public CAN_BUS canBus { get; set; }

        [DefaultValue("0.0")]
        [PhysicalUnitsFamily(physicalUnit.Family.angle)]
        public doubleParameter rotation { get; set; }

        [DefaultValue(pigeonType.pigeon1)]
        public pigeonType type { get; set; }

        [DefaultValue(pigeonPosition.CENTER_OF_ROTATION)]
        public pigeonPosition position { get; set; }

        public pigeon()
        {
        }
    }

    [Serializable()]
    [DataDescription("Multiple pneumatic control modules can be added to a robot")]
    public class pcm : baseRobotElementClass
    {
        [DefaultValue(0u)]
        [Range(typeof(uint), "0", "62")]
        [DataDescription("The ID that is used to form the device CAD ID")]
        [DataDescription("ID 0 is normally reserved for the roborio")]
        public uintParameter canID { get; set; }

        [DefaultValue(95.0)]
        [PhysicalUnitsFamily(physicalUnit.Family.pressure)]
        public doubleParameter minPressure { get; set; }

        [DefaultValue(115.0)]
        [PhysicalUnitsFamily(physicalUnit.Family.pressure)]
        public doubleParameter maxPressure { get; set; }

        public pcm()
        {
        }
    }

    [Serializable()]
    [ImplementationName("DragonAnalogInput")]
    [UserIncludeFile("hw/DragonAnalogInput.h")]
    public class analogInput : baseRobotElementClass
    {
        public enum analogInputType
        {
            ANALOG_GENERAL,
            ANALOG_GYRO,
            POTENTIOMETER,
            PRESSURE_GAUGE,
            ELEVATOR_HEIGHT
        }

        [DefaultValue(analogInputType.PRESSURE_GAUGE)]
        [ConstantInMechInstance]
        public analogInputType type { get; set; }

        [DefaultValue(0u)]
        [Range(typeof(uint), "0", "7")]
        public uintParameter analogId { get; set; }

        [DefaultValue(0D)]
        [ConstantInMechInstance]
        public doubleParameter voltageMin { get; set; }

        [DefaultValue(5D)]
        [ConstantInMechInstance]
        public doubleParameter voltageMax { get; set; }

        [ConstantInMechInstance]
        public doubleParameter outputMin { get; set; }

        [ConstantInMechInstance]
        public doubleParameter outputMax { get; set; }

        public analogInput()
        {
        }

        override public List<string> generateObjectCreation()
        {
            string creation = string.Format("{0} = new {1}(\"{0}\",{1}::ANALOG_SENSOR_TYPE::{2},{3},{4},{5},{6},{7})",
                name,
                getImplementationName(),
                type,
                analogId.value,
                voltageMin.value,
                voltageMax.value,
                outputMin.value,
                outputMax.value
                );

            return new List<string> { creation };
        }

        override public List<string> generateInitialization()
        {
            List<string> initCode = new List<string>()
            {
                string.Format("// {0} : Analog inputs do not have initialization needs", name)
            };

            return initCode;
        }
    }


    [Serializable()]
    public class chassis : baseRobotElementClass
    {
        [DataDescription("Add the drive motor modules here")]
        public List<mechanismInstance> mechanismInstances { get; set; }

        [DataDescription("The Chassis can contain multiple pigeons")]
        public List<pigeon> Pigeons { get; set; }

        /*
        public enum chassisType
        {
            TANK,
            MECANUM,
            SWERVE,
        }
        public enum chassisWheelSpeedCalcOption
        {
            WPI,
            ETHER,
            _2910,
        }
        public enum chassisPoseEstimationOption
        {
            WPI,
            EULERCHASSIS,
            EULERWHEEL,
            POSECHASSIS,
            POSEWHEEL,
        }

        public List<MotorController> motor { get; set; }
        public List<swerveModule> swervemodule { get; set; }

        [DefaultValue(chassisType.TANK)]
        public chassisType type { get; set; }

        [DefaultValue(1.0)]
        [PhysicalUnitsFamily(physicalUnit.Family.length)]
        public doubleParameter wheelDiameter { get; set; }

        [DefaultValue(1.0)]
        [PhysicalUnitsFamily(physicalUnit.Family.length)]
        public doubleParameter wheelBase { get; set; }

        [DefaultValue(1.0)]
        [PhysicalUnitsFamily(physicalUnit.Family.length)]
        public doubleParameter track { get; set; }

        [DefaultValue(chassisWheelSpeedCalcOption.ETHER)]
        public chassisWheelSpeedCalcOption wheelSpeedCalcOption { get; set; }

        [DefaultValue(chassisPoseEstimationOption.EULERCHASSIS)]
        public chassisPoseEstimationOption poseEstimationOption { get; set; }

        [TunableParameter]
        [PhysicalUnitsFamily(physicalUnit.Family.velocity)]
        public doubleParameter maxVelocity { get; set; }

        [TunableParameter]
        [PhysicalUnitsFamily(physicalUnit.Family.angularVelocity)]
        public doubleParameter maxAngularVelocity { get; set; }

        [TunableParameter]
        [PhysicalUnitsFamily(physicalUnit.Family.acceleration)]
        public doubleParameter maxAcceleration { get; set; }

        [TunableParameter]
        [PhysicalUnitsFamily(physicalUnit.Family.angularAcceleration)]
        public doubleParameter maxAngularAcceleration { get; set; }
        */

        public chassis()
        {
            name = this.GetType().Name;

            helperFunctions.initializeNullProperties(this);
            helperFunctions.initializeDefaultValues(this);
        }


        public string getDisplayName(string propertyName, out helperFunctions.RefreshLevel refresh)
        {
            refresh = helperFunctions.RefreshLevel.none;

            if (string.IsNullOrEmpty(propertyName))
                return "Chassis";

            PropertyInfo pi = this.GetType().GetProperty(propertyName);
            if (pi == null)
                return string.Format("chassis.getDisplayName : pi is null for propertyName {0}", propertyName);

            object obj = pi.GetValue(this);
            if (obj == null)
                return string.Format("chassis.getDisplayName : obj is null for propertyName {0}", propertyName);

            if (obj is parameter)
            {
                pi = this.GetType().GetProperty("value");
                obj = pi.GetValue(this);
            }

            return string.Format("{0} ({1})", propertyName, obj.ToString());
        }

        override public List<string> generateElementNames()
        {
            List<string> sb = new List<string>();

            foreach (pigeon p in Pigeons)
            {
                sb.Add(string.Format("{1}::{0}", ToUnderscoreCase(p.name), ToUnderscoreCase(p.GetType().Name)));
            }

            return sb;
        }
    }

    [Serializable()]
    [ImplementationName("DragonDigitalInput")]
    [UserIncludeFile("hw/DragonDigitalInput.h")]
    public class digitalInput : baseRobotElementClass
    {
        [DefaultValue(0u)]
        [Range(typeof(uint), "0", "25")]
        [PhysicalUnitsFamily(physicalUnit.Family.none)]
        public uintParameter digitalId { get; set; }

        [DefaultValue(false)]
        [ConstantInMechInstance]
        public boolParameter reversed { get; set; }

        [DefaultValue(0D)]
        [PhysicalUnitsFamily(physicalUnit.Family.time)]
        [ConstantInMechInstance]
        public doubleParameter debouncetime { get; set; }

        public digitalInput()
        {
        }

        public override List<string> generateIndexedObjectCreation(int index)
        {
            string creation = string.Format("{0} = new {1}(\"{0}\",RobotElementNames::{2},{3},{4},{5}({6}));",
                                            name,
                                            getImplementationName(),
                                            utilities.ListToString(generateElementNames()).ToUpper().Replace("::", "_USAGE::"),
                                            digitalId.value,
                                            reversed.value.ToString().ToLower(),
                                            generatorContext.theGeneratorConfig.getWPIphysicalUnitType(debouncetime.__units__),
                                            debouncetime.value
                                            );

            return new List<string> { creation };
        }

        override public List<string> generateObjectCreation()
        {
            return new List<string>();
        }

        override public List<string> generateInitialization()
        {
            List<string> initCode = new List<string>()
            {
                string.Format("// {0} : Digital inputs do not have initialization needs", name)
            };

            return initCode;
        }
    }

    [Serializable()]
    public class swerveModule : baseRobotElementClass
    {
        public enum swervemoduletype
        {
            LEFT_FRONT,
            RIGHT_FRONT,
            LEFT_BACK,
            RIGHT_BACK,
        }

        public List<MotorController> motor { get; set; }
        public CANcoder cancoder { get; set; }
        [DefaultValue(swervemoduletype.LEFT_FRONT)]
        public swervemoduletype position { get; set; }
        public PIDFZ controlParameters { get; set; }

        [DefaultValue(0.0)]
        [TunableParameter()]
        [PhysicalUnitsFamily(physicalUnit.Family.percent)]
        public doubleParameter turn_nominal_val { get; set; }

        [DefaultValue(0.0)]
        [TunableParameter()]
        [PhysicalUnitsFamily(physicalUnit.Family.percent)]
        public doubleParameter turn_peak_val { get; set; }

        [DefaultValue(0.0)]
        [TunableParameter()]
        [PhysicalUnitsFamily(physicalUnit.Family.angularAcceleration)]
        public doubleParameter turn_max_acc { get; set; }

        [DefaultValue(0.0)]
        [TunableParameter()]
        [PhysicalUnitsFamily(physicalUnit.Family.angularVelocity)]
        public doubleParameter turn_cruise_vel { get; set; }

        [DefaultValue(1.0)]
        public uintParameter countsOnTurnEncoderPerDegreesOnAngleSensor { get; set; }

        public swerveModule()
        {
            motor = new List<MotorController>();
            helperFunctions.initializeNullProperties(this);
            helperFunctions.initializeDefaultValues(this);
        }
    }


    [Serializable()]
    [ImplementationName("DragonCanCoder")]
    [UserIncludeFile("hw/DragonCanCoder.h")]
    public class CANcoder : baseRobotElementClass
    {
        [DefaultValue(0u)]
        [Range(typeof(uint), "0", "62")]
        [DataDescription("The ID that is used to form the device CAD ID")]
        [DataDescription("ID 0 is normally reserved for the roborio")]
        public uintParameter canID { get; set; }

        [DefaultValue(CAN_BUS.rio)]
        public CAN_BUS canBusName { get; set; }

        [DefaultValue(0D)]
        public doubleParameter offset { get; set; }

        [DefaultValue(false)]
        public boolParameter reverse { get; set; }

        public CANcoder()
        {
        }

        override public List<string> generateIndexedObjectCreation(int index)
        {
            string creation = string.Format("{0} = new {1}(\"{0}\",RobotElementNames::{2},{3},\"{4}\",{5},{6});",
                name,
                getImplementationName(),
                utilities.ListToString(generateElementNames()).ToUpper().Replace("::", "_USAGE::"),
                canID.value,
                canBusName,
                offset.value,
                reverse.value.ToString().ToLower()
                );

            return new List<string> { creation };
        }

        override public List<string> generateInitialization()
        {
            List<string> initCode = new List<string>()
            {
                string.Format("// {0} : CANcoder inputs do not have initialization needs", name)
            };

            return initCode;
        }
    }


    [Serializable()]
    [ImplementationName("DragonSolenoid")]
    [UserIncludeFile("hw/DragonSolenoid.h")]
    public class solenoid : baseRobotElementClass
    {
        public enum solenoidtype
        {
            CTREPCM,
            REVPH,
        }

        [DefaultValue(0u)]
        [Range(typeof(uint), "0", "62")]
        public uintParameter CAN_ID { get; set; }

        [DefaultValue(0u)]
        [Range(typeof(uint), "0", "7")]
        public uintParameter channel { get; set; }

        [DefaultValue(false)]
        public boolParameter enableDualChannel { get; set; }

        [DefaultValue(0u)]
        [Range(typeof(uint), "0", "7")]
        public uintParameter forwardChannel { get; set; }

        [DefaultValue(0u)]
        [Range(typeof(uint), "0", "7")]
        public uintParameter reverseChannel { get; set; }


        [DefaultValue(false)]
        public boolParameter reversed { get; set; }

        [DefaultValue(solenoidtype.REVPH)]
        public solenoidtype type { get; set; }

        public solenoid()
        {
        }

        override public List<string> generateObjectCreation()
        {
            string creation = "";

            if (enableDualChannel.value)
            {
                creation = string.Format("{0} = new {1}(\"{0}\",RobotElementNames::{2},{3},frc::PneumaticsModuleType::{4},{5},{6},{7})",
                    name,
                    getImplementationName(),
                    utilities.ListToString(generateElementNames()).ToUpper().Replace("::", "_USAGE::"),
                    CAN_ID.value,
                    type,
                    forwardChannel.value,
                    reverseChannel.value,
                    reversed.value.ToString().ToLower()
                    );
            }
            else
            {
                creation = string.Format("{0} = new {1}(\"{0}\",RobotElementNames::{2},{3},frc::PneumaticsModuleType::{4},{5},{6})",
                    name,
                    getImplementationName(),
                    utilities.ListToString(generateElementNames()).ToUpper().Replace("::", "_USAGE::"),
                    CAN_ID.value,
                    type,
                    channel.value,
                    reversed.value.ToString().ToLower()
                    );
            }

            return new List<string> { creation };
        }

        override public List<string> generateInitialization()
        {
            List<string> initCode = new List<string>()
            {
                string.Format("// {0} : Solenoids do not have initialization needs", name)
            };

            return initCode;
        }

        override public List<string> generateObjectAddToMaps()
        {
            string creation = string.Format("m_solenoidMap[{0}->GetType()] = new BaseMechSolenoid(m_ntName, *{0})",
                name);

            return new List<string> { creation };
        }
    }

    [Serializable()]
    [ImplementationName("DragonServo")]
    [UserIncludeFile("hw/DragonServo.h")]
    public class servo : baseRobotElementClass
    {
        [DefaultValue(0u)]
        [Range(typeof(uint), "0", "19")]
        public uintParameter Id { get; set; }

        [DefaultValue(0.0)]
        [Range(typeof(double), "0", "360")]
        [PhysicalUnitsFamily(physicalUnit.Family.angle)]
        public doubleParameter minAngle { get; set; }

        [DefaultValue(360.0)]
        [Range(typeof(double), "0", "360")]
        [PhysicalUnitsFamily(physicalUnit.Family.angle)]
        public doubleParameter maxAngle { get; set; }

        public servo()
        {
        }

        override public List<string> generateIndexedObjectCreation(int currentIndex)
        {
            string creation = string.Format("{0} = new {1}(RobotElementNames::{2},{3},{4}({5}),{6}({7}))",
                name,
                getImplementationName(),
                utilities.ListToString(generateElementNames()).ToUpper().Replace("::", "_USAGE::"),
                Id.value,
                generatorContext.theGeneratorConfig.getWPIphysicalUnitType(minAngle.__units__),
                minAngle.value,
                generatorContext.theGeneratorConfig.getWPIphysicalUnitType(maxAngle.__units__),
                maxAngle.value
                );

            return new List<string> { creation };
        }

        override public List<string> generateInitialization()
        {
            List<string> initCode = new List<string>()
            {
                string.Format("// {0} : Servos do not have initialization needs", name)
            };

            return initCode;
        }

        override public List<string> generateObjectAddToMaps()
        {
            string creation = string.Format("m_servoMap[{0}->GetUsage()] = new BaseMechServo(m_ntName, {0})",
                name);

            return new List<string> { creation };
        }
    }


    [Serializable()]
    [ImplementationName("DragonColorSensor")]
    [UserIncludeFile("hw/DragonColorSensor.h")]
    public class colorSensor : baseRobotElementClass
    {
        public enum colorSensorPort
        {
            kOnboard,
            kMXP,
        }

        [DefaultValue(colorSensorPort.kOnboard)]
        public colorSensorPort port { get; set; }

        public colorSensor()
        {
        }
    }

    
    [Serializable()]
    public class roborio : baseRobotElementClass
    {
        public enum Orientation
        {
            X_FORWARD_Y_LEFT,
            X_LEFT_Y_BACKWARD,
            X_BACKWARD_Y_RIGHT,
            X_RIGHT_Y_FORWARD,
            X_FORWARD_Y_RIGHT,
            X_LEFT_Y_FORWARD,
            X_BACKWARD_Y_LEFT,
            X_RIGHT_Y_BACKWARD,
            X_UP_Y_LEFT,
            X_LEFT_Y_DOWN,
            X_DOWN_Y_RIGHT,
            X_RIGHT_Y_UP,
            X_UP_Y_RIGHT,
            X_LEFT_Y_UP,
            X_DOWN_Y_LEFT,
            X_RIGHT_Y_DOWN,
        }

        [DefaultValue(Orientation.X_FORWARD_Y_LEFT)]
        public Orientation orientation { get; set; }

        public roborio()
        {
            helperFunctions.initializeNullProperties(this);
            helperFunctions.initializeDefaultValues(this);
        }
    }


    [Serializable()]
    public class talontach : baseRobotElementClass
    {
        [DefaultValue(0u)]
        [Range(typeof(uint), "0", "62")]
        public uintParameter CAN_ID { get; set; }

        [DefaultValue(0u)]
        [Range(typeof(uint), "0", "6")]
        public uintParameter usage { get; set; }

        [DefaultValue(0u)]
        [Range(typeof(uint), "0", "11")]
        public uintParameter generalpin { get; set; }

        public talontach()
        {
        }
    }
#endif

    [Serializable]
    public class baseRobotElementClass
    {
        [ConstantInMechInstance]
        public string name { get; set; }

        public baseRobotElementClass()
        {
            helperFunctions.initializeNullProperties(this, true);
            helperFunctions.initializeDefaultValues(this);
            name = GetType().Name;
        }

        virtual public string getDisplayName(string propertyName, out helperFunctions.RefreshLevel refresh)
        {
            refresh = helperFunctions.RefreshLevel.none;

            if (string.IsNullOrEmpty(propertyName))
                refresh = helperFunctions.RefreshLevel.none;
            else if (propertyName == "name")
                refresh = helperFunctions.RefreshLevel.parentHeader;

            if (string.IsNullOrEmpty(propertyName))
                return name;

            PropertyInfo pi = this.GetType().GetProperty(propertyName);
            if (pi == null)
                return string.Format("baseRobotElementClass.getDisplayName : pi is null for propertyName {0}", propertyName);

            object obj = pi.GetValue(this);
            if (obj == null)
                return string.Format("baseRobotElementClass.getDisplayName : obj is null for propertyName {0}", propertyName);

            if (obj is parameter)
            {
                pi = this.GetType().GetProperty("value");
                obj = pi.GetValue(this);
            }

            return string.Format("{0} ({1})", propertyName, obj.ToString());
        }

        virtual public List<string> generateElementNames()
        {
            Type baseType = GetType();
            while ((baseType.BaseType != typeof(object)) && (baseType.BaseType != typeof(baseRobotElementClass)))
                baseType = baseType.BaseType;

            if (generatorContext.theMechanismInstance != null)
            {
                return new List<string> { string.Format("{2}::{0}_{1}", ToUnderscoreCase(generatorContext.theMechanismInstance.name), ToUnderscoreCase(name), ToUnderscoreCase(baseType.Name)) };
            }
            else if (generatorContext.theMechanism != null)
            {
                return new List<string> { string.Format("{2}::{0}_{1}", ToUnderscoreCase(generatorContext.theMechanism.name), ToUnderscoreCase(name), ToUnderscoreCase(baseType.Name)) };
            }
            else if (generatorContext.theRobot != null)
                return new List<string> { string.Format("{1}::{0}", ToUnderscoreCase(name), ToUnderscoreCase(baseType.Name)) };
            else
                return new List<string> { "generateElementNames got to the else statement...should not be here" };
        }
        public string getImplementationName()
        {
            ImplementationNameAttribute impNameAttr = this.GetType().GetCustomAttribute<ImplementationNameAttribute>();
            if (impNameAttr == null)
                return this.GetType().Name;

            return impNameAttr.name;
        }
        virtual public List<string> generateDefinition()
        {
            return new List<string> { string.Format("{0}* {1};", getImplementationName(), name) }; //todo add m_ in off season
        }

        virtual public List<string> generateDefinitionGetter()
        {
            return new List<string> { string.Format("{0}* get{1}() const {{return {1};}}", getImplementationName(), name) };
        }

        virtual public List<string> generateInitialization()
        {
            return new List<string> { "baseRobotElementClass.generateInitialization needs to be overridden" };
        }
        virtual public List<string> generateObjectCreation()
        {
            return new List<string> { "baseRobotElementClass.generateObjectCreation needs to be overridden" };
        }
        virtual public List<string> generateIndexedObjectCreation(int index)
        {
            return new List<string> { "baseRobotElementClass.generateIndexedObjectCreation(int index) needs to be overridden" };
        }
        virtual public List<string> generateObjectAddToMaps()
        {
            return new List<string> { "baseRobotElementClass.generateObjectAddToMaps needs to be overridden" };
        }

        virtual public List<string> generateIncludes()
        {
            List<string> sb = new List<string>();
            List<UserIncludeFileAttribute> userIncludesAttr = this.GetType().GetCustomAttributes<UserIncludeFileAttribute>().ToList();
            foreach (UserIncludeFileAttribute include in userIncludesAttr)
                sb.Add(string.Format("#include \"{0}\"{1}", include.pathName, Environment.NewLine));

            List<SystemIncludeFileAttribute> sysIncludesAttr = this.GetType().GetCustomAttributes<SystemIncludeFileAttribute>().ToList();
            foreach (SystemIncludeFileAttribute include in sysIncludesAttr)
                sb.Add(string.Format("#include <{0}>{1}", include.pathName, Environment.NewLine));

            return sb;
        }

        virtual public List<string> generateUsings()
        {
            List<string> sb = new List<string>();
            List<UsingAttribute> usingsAttr = this.GetType().GetCustomAttributes<UsingAttribute>().ToList();
            foreach (UsingAttribute u in usingsAttr)
                sb.Add(string.Format("using {0}", u.theUsing));

            return sb;
        }

        internal string ToUnderscoreCase(string str)
        {
            if (str.Contains("_"))
                return str;

            return string.Concat(str.Select((x, i) => i > 0 && char.IsUpper(x) && char.IsLower(str[i - 1]) ? "_" + x.ToString() : x.ToString())).ToLower();
        }

        protected string ListToString(List<string> list, string delimeter, bool discardWhiteSpaceStrings)
        {
            StringBuilder sb = new StringBuilder();
            for (int i = 0; i < list.Count; i++)
            {
                list[i] = list[i].Trim();
                if (!string.IsNullOrWhiteSpace(list[i]))
                    sb.AppendLine(string.Format("{0}{1}", list[i], delimeter));
                else if (!discardWhiteSpaceStrings)
                    sb.AppendLine(string.Format("{0}", list[i]));
            }

            return sb.ToString().Trim();
        }
    }

    [Serializable]
    public class baseDataClass
    {
        protected string defaultDisplayName { get; set; } = "defaultDisplayName";

        virtual public string getDisplayName(string propertyName, out helperFunctions.RefreshLevel refresh)
        {
            refresh = helperFunctions.RefreshLevel.none;

            if (propertyName == "")
                return defaultDisplayName;

            PropertyInfo pi = this.GetType().GetProperty(propertyName);
            if (pi != null)
            {
                object value = pi.GetValue(this);
                return string.Format("{0} ({1})", propertyName, value.ToString());
            }

            return null;
        }

        public baseDataClass()
        {
            helperFunctions.initializeNullProperties(this, true);
            helperFunctions.initializeDefaultValues(this);
        }
    }

    public static class generatorContext
    {
        public enum GenerationStage { Unknown, MechInstanceGen, MechInstanceDecorator }

        public static GenerationStage generationStage { get; set; }

        public const bool singleStateGenFile = true;
        public static mechanism theMechanism { get; set; }
        public static mechanismInstance theMechanismInstance { get; set; }
        public static int stateIndex { get; set; }
        public static applicationData theRobot { get; set; }
        public static topLevelAppDataElement theRobotVariants { get; set; }
        public static toolConfiguration theGeneratorConfig { get; set; }

        public static void clear()
        {
            generationStage = GenerationStage.Unknown;
            theMechanism = null;
            theMechanismInstance = null;
            theRobot = null;
            stateIndex = 0;
        }
    }

    public static class utilities
    {
        public static string ListToString(List<string> list, string delimeter)
        {
            StringBuilder sb = new StringBuilder();
            for (int i = 0; i < list.Count; i++)
            {
                list[i] = list[i].Trim();
                if (!string.IsNullOrWhiteSpace(list[i]))
                    sb.AppendLine(string.Format("{0}{1}", list[i], delimeter));
            }

            return sb.ToString().Trim();
        }
        public static string ListToString(List<string> list)
        {
            return ListToString(list, "");
        }
    }

    /*
     * ====================================================
     * David's interpretation of what is needed -> start
     */

    [Serializable()]
    [ImplementationName("ControlData")]
    public class motorControlData : baseRobotElementClass
    {
        public enum CONTROL_TYPE
        {
            PERCENT_OUTPUT,            /// Open Loop Control - values are between -1.0 and 1.0
            POSITION_INCH,             /// Closed Loop Control - values are displacements measured in inches
            POSITION_ABS_TICKS,        /// Closed Loop Control - values are measured in ticks
            POSITION_DEGREES,          /// Closed Loop Control - values are angles measured in degrees
            POSITION_DEGREES_ABSOLUTE, /// Closed Loop Control - values are angles measured in degrees that don't need to be converted
            VELOCITY_INCH,             /// Closed Loop Control - values are linear velocity measured in inches per second
            VELOCITY_DEGREES,          /// Closed Loop Control - values are angular velocity measured in degrees per second
            VELOCITY_RPS,              /// Closed Loop Control - values are in revolutions per second
            VOLTAGE,                   /// Closed Loop Control - values are in volts
            CURRENT,                   /// Closed Loop Control - values in amps
            TRAPEZOID_ANGULAR_POS,     /// Closed Loop Control - trapezoid profile (e.g. Motion Magic)
            TRAPEZOID_LINEAR_POS,     /// Closed Loop Control - trapezoid profile (e.g. Motion Magic)
            MAX_CONTROL_TYPES
        };

        public enum CONTROL_RUN_LOCS
        {
            MOTOR_CONTROLLER,
            ROBORIO,
            MAX_CONTROL_RUN_LOCS
        };

        public enum FEEDFORWARD_TYPE
        {
            VOLTAGE,
            TORQUE_CURRENT,
            DUTY_CYCLE
        };

        public PIDFZ PID { get; set; }

        [DefaultValue(0)]
        public doubleParameter peakValue { get; set; }
        [DefaultValue(0)]
        public doubleParameter nominalValue { get; set; }
        [DefaultValue(0)]
        public doubleParameter maxAcceleration { get; set; }

        [DefaultValue(0)]
        public doubleParameter cruiseVelocity { get; set; } //todo what does this mean

        [DefaultValue(false)]
        public boolParameter enableFOC { get; set; }

        [DefaultValue(FEEDFORWARD_TYPE.VOLTAGE)]
        public FEEDFORWARD_TYPE feedForwardType { get; set; }

        [DefaultValue(CONTROL_TYPE.PERCENT_OUTPUT)]
        [ConstantInMechInstance]
        public CONTROL_TYPE controlType { get; set; }

        [DefaultValue(CONTROL_RUN_LOCS.MOTOR_CONTROLLER)]
        public CONTROL_RUN_LOCS controlLoopLocation { get; set; }
        public motorControlData()
        {
            name = GetType().Name;
        }

        override public List<string> generateIndexedObjectCreation(int currentIndex)
        {
            /*
        {2}, // ControlModes::CONTROL_TYPE mode
        {3}, // ControlModes::CONTROL_RUN_LOCS server
        {4}, // std::string indentifier
        {5}, // double proportional
        {6}, // double integral
        {7}, // double derivative
        {8}, // double feedforward
        {9}, // FEEDFORWARD_TYPE feedforwadType
        {10}, // double integralZone
        {11}, // double maxAcceleration
        {12}, // double cruiseVelocity
        {13}, // double peakValue
        {14}, // double nominalValue
        {15}  // bool enableFOC"
             */
            string controlTypeStr = controlType.ToString();
            if ((controlType == CONTROL_TYPE.TRAPEZOID_ANGULAR_POS) || (controlType == CONTROL_TYPE.TRAPEZOID_LINEAR_POS))
                controlTypeStr = "TRAPEZOID";

            string creation = string.Format(@"{0} = new {1}(
                                                            ControlModes::CONTROL_TYPE::{2}, // ControlModes::CONTROL_TYPE mode
                                                            ControlModes::CONTROL_RUN_LOCS::{3}, // ControlModes::CONTROL_RUN_LOCS server
                                                            ""{0}"", // std::string indentifier
                                                            {4}, // double proportional
                                                            {5}, // double integral
                                                            {6}, // double derivative
                                                            {7}, // double feedforward
                                                            ControlData::FEEDFORWARD_TYPE::{8}, // FEEDFORWARD_TYPE feedforwadType
                                                            {9}, // double integralZone
                                                            {10}, // double maxAcceleration
                                                            {11}, // double cruiseVelocity
                                                            {12}, // double peakValue
                                                            {13}, // double nominalValue
                                                            {14}  // bool enableFOC
                );",
            name,
                getImplementationName(),
                controlTypeStr,
                controlLoopLocation,
                PID.pGain.value,
                PID.iGain.value,
                PID.dGain.value,
                PID.fGain.value,
                feedForwardType,
                PID.iZone.value,
                maxAcceleration.value,
                cruiseVelocity.value,
                peakValue.value,
                nominalValue.value,
                enableFOC.value.ToString().ToLower()
                /*
                utilities.ListToString(generateElementNames()).ToUpper().Replace("::", "_USAGE::"),
                Id.value,
                generatorContext.theGeneratorConfig.getWPIphysicalUnitType(minAngle.__units__),
                minAngle.value,
                generatorContext.theGeneratorConfig.getWPIphysicalUnitType(maxAngle.__units__),
                maxAngle.value*/
                );

            return new List<string> { creation };
        }

        override public List<string> generateInitialization()
        {
            List<string> initCode = new List<string>()
            {
                string.Format("// {0} : ControlData does not have initialization needs", name)
            };

            return initCode;
        }

        override public List<string> generateObjectAddToMaps()
        {
            List<string> initCode = new List<string>()
            {
                string.Format("// {0} : ControlData is not added to a map", name)
            };

            return initCode;
        }
    }

    [Serializable]
    public class motorControlDataLink : baseRobotElementClass
    {
        [ConstantInMechInstance]
        [DefaultValue("theControlDataName")]
        public string motorControlDataName { get; set; }

        public motorControlDataLink()
        {
        }
    }

    [Serializable]
    public class motorTarget : baseRobotElementClass
    {
        public doubleParameterUserDefinedTunableOnlyValueChangeableInMechInst target { get; set; }

        [ConstantInMechInstance]
        [DataDescription("The name of the motor that this target applies to")]
        public string motorName { get; set; }

        [ConstantInMechInstance]
        [DataDescription("If Enabled is true, this motor target will be set on entry into the state.")]
        public boolParameter Enabled { get; set; }

        [DataDescription("The name of the control data to use in order to reach this target")]
        public string controlDataName { get; set; }

        public motorTarget()
        {
            Enabled.value = true;
            target.name = "Target";
            motorName = "theMotorName";
            controlDataName = "theControlData";
        }
    }

    [Serializable()]
    public class state : baseRobotElementClass
    {
        [ConstantInMechInstance()]
        public List<stringParameterConstInMechInstance> transitionsTo { get; set; }

        public List<motorTarget> motorTargets { get; set; }

        public state()
        {
            name = GetType().Name;
            helperFunctions.initializeNullProperties(this);
            helperFunctions.initializeDefaultValues(this);
        }

        public override List<string> generateIncludes()
        {
            List<string> sb = new List<string>();
            if ((generatorContext.theMechanismInstance != null) && (generatorContext.GenerationStage.MechInstanceDecorator == generatorContext.generationStage))
            {
                sb.Add(string.Format("#include \"mechanisms/{1}/decoratormods/{0}State.h\"",
                    name,
                    generatorContext.theMechanismInstance.name));
            }

            return sb;
        }
        public override List<string> generateIndexedObjectCreation(int index)
        {
            if (generatorContext.theMechanismInstance != null)
            {
                string creation = "";

                if (generatorContext.singleStateGenFile)
                {
                    creation = string.Format("{0}State* {0}StateInst = new {0}State(string(\"{0}\"), {2}, new {1}AllStatesStateGen(m_activeRobotId, string(\"{0}\"), {2}, this), this)",
                    name,
                    generatorContext.theMechanismInstance.name,
                    index);
                }
                else
                {
                    creation = string.Format("{0}State* {0}StateInst = new {0}State(string(\"{0}\"), {2}, new {1}{0}StateGen(string(\"{0}\"), {2}, this), this)",
                    name,
                    generatorContext.theMechanismInstance.name,
                    index);
                }

                List<string> code = new List<string>() { creation };

                code.AddRange(generateObjectAddToMaps());
                code.Add("");

                return code;
            }

            return new List<string>();
        }

        override public List<string> generateInitialization()
        {
            List<string> initCode = new List<string>()
            {
                string.Format("//todo create initialization for {0}", name)
            };

            return initCode;
        }
        override public List<string> generateObjectAddToMaps()
        {
            string creation = string.Format("AddToStateVector({0}StateInst)", name);

            return new List<string> { creation };
        }

        override public List<string> generateElementNames()
        {
            Type baseType = GetType();
            while ((baseType.BaseType != typeof(object)) && (baseType.BaseType != typeof(baseRobotElementClass)))
                baseType = baseType.BaseType;

            if (generatorContext.theMechanismInstance != null)
            {
                return new List<string> { string.Format("{2}_{0}::{0}_{1}", ToUnderscoreCase(generatorContext.theMechanismInstance.name), ToUnderscoreCase(name), ToUnderscoreCase(baseType.Name)) };
            }
            else
                return new List<string> { "generateElementNames got to the else statement...should not be here" };
        }
    }
    /*
 * ====================================================
 * David's interpretation of what is needed -> end
 */



    [Serializable()]
    public class controlData
    {
        public string name { get; set; }

        PIDFZ pid { get; set; }

        public double maxAccel { get; set; }

        public controlData()
        {
            name = GetType().Name;
        }
    }

    [Serializable()]
    public class actuatorTarget
    {
        public string name { get; set; }

        PIDFZ pid { get; set; }

        public double maxAccel { get; set; }

        public actuatorTarget()
        {
            name = GetType().Name;
        }
    }
    [Serializable()]
    public class state_
    {
        public string name { get; set; }

        public List<controlData> controlData { get; set; }

        public state_()
        {
            name = GetType().Name;
            controlData = new List<controlData>();
        }
    }

}
