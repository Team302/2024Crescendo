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
        [DataDescription("The robot definitions")]
        public List<applicationData> Robots { get; set; }

        [DataDescription("The mechanism templates")]
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
    {
#if !enableTestAutomation

        [DataDescription("One power distribution panel can be configured for a robot")]
        public pdp PowerDistributionPanel { get; set; }
        
        [DataDescription("A robot can contain multiple pneumatic control modules")]
        public List<pcm> PneumaticControlModules { get; set; }
      
        [DataDescription("A robot can contain multiple limelights")]
        public List<limelight> Limelights { get; set; }

        [DataDescription("A robot can have one chassis definition")]
        public chassis Chassis { get; set; }

        [DataDescription("A robot can contain multiple mechanism instances")]
        public List<mechanismInstance> mechanismInstances { get; set; }

        [DataDescription("A robot can contain multiple cameras")]
        public List<camera> Cameras { get; set; }

        [DataDescription("A robot can contain multiple roborios")]
        public List<roborio> Roborios { get; set; }

        [DataDescription("A robot can contain multiple LED setups")]

        public List<led> Leds { get; set; }

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
            return string.Format("{0}_{1}", name, robotID.value);
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
    [XmlInclude(typeof(TalonFX))]
    [XmlInclude(typeof(TalonSRX))]
    public class MotorController : baseRobotElementClass
    {
        public enum InvertedValue { CounterClockwise_Positive, Clockwise_Positive }
        public enum NeutralModeValue { Coast, Brake }

        public enum SwitchConfiguration
        {
            NormallyOpen,
            NormallyClosed
        };

        public enum RemoteSensorSource
        {
            Off,
            TalonSRX_SelectedSensor,
            Pigeon_Yaw,
            Pigeon_Pitch,
            Pigeon_Roll,
            CANifier_Quadrature,
            CANifier_PWMInput0,
            CANifier_PWMInput1,
            CANifier_PWMInput2,
            CANifier_PWMInput3,
            GadgeteerPigeon_Yaw,
            GadgeteerPigeon_Pitch,
            GadgeteerPigeon_Roll,
            CANCoder,
            TalonFX_SelectedSensor = TalonSRX_SelectedSensor,
        };

        public enum MOTOR_TYPE
        {
            UNKNOWN_MOTOR = -1,
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
            NONE,
            MAX_MOTOR_TYPES
        };

        [Serializable]
        public class DistanceAngleCalcStruc : baseDataClass
        {
            [DefaultValue(0)]
            [ConstantInMechInstance]
            public intParameter countsPerRev { get; set; }

            [DefaultValue(1.0)]
            [ConstantInMechInstance]
            public doubleParameter gearRatio { get; set; }

            [DefaultValue(1.0)]
            [ConstantInMechInstance]
            [PhysicalUnitsFamily(physicalUnit.Family.length)]
            public doubleParameter diameter { get; set; }

            [DefaultValue(0)]
            [ConstantInMechInstance]
            public doubleParameter countsPerInch { get; set; }

            [DefaultValue(0)]
            [ConstantInMechInstance]
            public doubleParameter countsPerDegree { get; set; }

            public DistanceAngleCalcStruc()
            {
                defaultDisplayName = this.GetType().Name;
            }

            public string getDefinition(string namePrePend)
            {
                string fullName = getName(namePrePend);

                StringBuilder sb = new StringBuilder();
                sb.AppendLine(string.Format("{0} {1};", this.GetType().Name, fullName));

                foreach (PropertyInfo pi in GetType().GetProperties())
                {
                    Object obj = pi.GetValue(this);
                    // PhysicalUnitsFamilyAttribute unitsAttr = this.GetType().GetCustomAttribute<PhysicalUnitsFamilyAttribute>();

                    string rightValue = obj.ToString();
                    if (pi.Name == "diameter")
                    {
                        string units = generatorContext.theGeneratorConfig.getWPIphysicalUnitType(diameter.physicalUnits);
                        rightValue = string.Format("units::length::inch_t({0}({1})).to<double>()", units, rightValue);
                    }

                    sb.AppendLine(string.Format("{0}.{1} = {2} ;", fullName, pi.Name, rightValue));
                }

                return sb.ToString();
            }

            public string getName(string namePrePend)
            {
                return namePrePend + "CalcStruct";
            }
        }
        public DistanceAngleCalcStruc theDistanceAngleCalcInfo { get; set; }

        [Serializable]
        public class VoltageRamping : baseDataClass
        {
            [DefaultValue(0)]
            [PhysicalUnitsFamily(physicalUnit.Family.time)]
            [ConstantInMechInstance]
            public doubleParameter openLoopRampTime { get; set; }

            [DefaultValue(0)]
            [PhysicalUnitsFamily(physicalUnit.Family.time)]
            [ConstantInMechInstance]
            public doubleParameter closedLoopRampTime { get; set; }

            [DefaultValue(false)]
            [ConstantInMechInstance]
            public boolParameter enableClosedLoop { get; set; }

            public VoltageRamping()
            {
                defaultDisplayName = this.GetType().Name;
            }
        }
        public VoltageRamping voltageRamping { get; set; }


        [XmlIgnore]
        [Constant()]
        public string motorControllerType { get; protected set; }

        [DefaultValue(MOTOR_TYPE.UNKNOWN_MOTOR)]
        [ConstantInMechInstance]
        public MOTOR_TYPE motorType { get; set; }

        [DefaultValue(0u)]
        [Range(typeof(uint), "0", "62")]
        [DataDescription("The ID that is used to form the device CAD ID")]
        [DataDescription("ID 0 is normally reserved for the roborio")]
        public uintParameter canID { get; set; }

        [DefaultValue(CAN_BUS.rio)]
        [ConstantInMechInstance]
        public CAN_BUS canBusName { get; set; }

        [DefaultValue(0u)]
        [Range(typeof(uint), "0", "15")]
        public uintParameter pdpID { get; set; }

        [DefaultValue(0u)]
        [Range(typeof(uint), "0", "19")] // REV is 0-19, CTRE 0-15, cannot handle 2 ranges for now
        [ConstantInMechInstance]
        public uintParameter followID { get; set; }

        [DefaultValue(false)]
        [ConstantInMechInstance]
        public boolParameter enableFollowID { get; set; }



        [Serializable]
        public class RemoteSensor : baseDataClass
        {
            [DefaultValue(RemoteSensorSource.Off)]
            [ConstantInMechInstance]
            public RemoteSensorSource Source { get; set; }

            [DefaultValue(0u)]
            [Range(typeof(uint), "0", "62")]
            [DataDescription("The ID that is used to form the device CAD ID")]
            public uintParameter CanID { get; set; }

            public RemoteSensor()
            {
                defaultDisplayName = this.GetType().Name;
            }
        }
        public RemoteSensor remoteSensor { get; set; }


        [Serializable]
        public class FusedCANcoder : baseDataClass
        {
            [DefaultValue(false)]
            [ConstantInMechInstance]
            public boolParameter enable { get; set; }

            [ConstantInMechInstance]
            public CANcoderInstance fusedCANcoder { get; set; }

            [ConstantInMechInstance]
            public doubleParameter sensorToMechanismRatio { get; set; }

            [ConstantInMechInstance]
            public doubleParameter rotorToSensorRatio { get; set; }

            public FusedCANcoder()
            {
                defaultDisplayName = "FusedCANcoder";
            }
        }

        public FusedCANcoder fusedCANcoder { get; set; }

        [DefaultValue(false)]
        [ConstantInMechInstance]
        public boolParameter sensorIsInverted { get; set; }

        public MotorController()
        {
            motorControllerType = this.GetType().Name;
        }

        public override List<string> generateInitialization()
        {
            List<string> initCode = new List<string>();

            initCode.Add(string.Format(@"{0}->SetVoltageRamping( units::time::second_t({1}({2})).to<double>(),
                                                                 units::time::second_t({3}({4})).to<double>() );",
                                                        name,
                                                        generatorContext.theGeneratorConfig.getWPIphysicalUnitType(voltageRamping.openLoopRampTime.physicalUnits),
                                                        voltageRamping.openLoopRampTime.value,
                                                        generatorContext.theGeneratorConfig.getWPIphysicalUnitType(voltageRamping.closedLoopRampTime.physicalUnits),
                                                        voltageRamping.enableClosedLoop.value ? voltageRamping.closedLoopRampTime.value : 0.0));

            initCode.Add(string.Format("{0}->SetSensorInverted( {1});",
                                                        name,
                                                        sensorIsInverted.ToString().ToLower()));

            return initCode;
        }

        override public List<string> generateObjectAddToMaps()
        {
            string creation = string.Format(@"m_motorMap[{0}->GetType()] = new BaseMechMotor(m_ntName, 
                                                                                            *{0}, 
                                                                                            BaseMechMotor::EndOfTravelSensorOption::NONE, 
                                                                                            nullptr, 
                                                                                            BaseMechMotor::EndOfTravelSensorOption::NONE, 
                                                                                            nullptr)",
                                                                                name);

            return new List<string> { creation };
        }

    }

    [Serializable()]
    [ImplementationName("DragonTalonFX")]
    [UserIncludeFile("hw/DragonTalonFX.h")]
    [Using("ctre::phoenixpro::signals::ForwardLimitSourceValue")]
    [Using("ctre::phoenixpro::signals::ForwardLimitTypeValue")]
    [Using("ctre::phoenixpro::signals::ReverseLimitSourceValue")]
    [Using("ctre::phoenixpro::signals::ReverseLimitTypeValue")]
    [Using("ctre::phoenixpro::signals::InvertedValue")]
    [Using("ctre::phoenixpro::signals::NeutralModeValue")]
    [Using("ctre::phoenix::motorcontrol::RemoteSensorSource")]
    public class TalonFX : MotorController
    {
        [Serializable]
        public class CurrentLimits : baseDataClass
        {
            [DefaultValue(false)]
            public boolParameter enableStatorCurrentLimit { get; set; }

            [DefaultValue(0)]
            [Range(typeof(double), "0", "40.0")] //todo choose a valid range
            [PhysicalUnitsFamily(physicalUnit.Family.current)]
            [ConstantInMechInstance]
            public doubleParameter statorCurrentLimit { get; set; }

            [DefaultValue(false)]
            [ConstantInMechInstance]
            public boolParameter enableSupplyCurrentLimit { get; set; }

            [DefaultValue(0)]
            [Range(typeof(double), "0", "50.0")] //todo choose a valid range
            [PhysicalUnitsFamily(physicalUnit.Family.current)]
            [ConstantInMechInstance]
            public doubleParameter supplyCurrentLimit { get; set; }

            [DefaultValue(0)]
            [Range(typeof(double), "0", "40.0")] //todo choose a valid range
            [PhysicalUnitsFamily(physicalUnit.Family.current)]
            [ConstantInMechInstance]
            public doubleParameter supplyCurrentThreshold { get; set; }

            [DefaultValue(0)]
            [Range(typeof(double), "0", "40.0")] //todo choose a valid range
            [PhysicalUnitsFamily(physicalUnit.Family.time)]
            [ConstantInMechInstance]
            public doubleParameter supplyTimeThreshold { get; set; }

            public CurrentLimits()
            {
                defaultDisplayName = "CurrentLimits";
            }
        }
        public CurrentLimits theCurrentLimits { get; set; }

        public List<PIDFslot> PIDFs { get; set; }

        [Serializable]
        public class ConfigHWLimitSW : baseDataClass
        {
            public enum ForwardLimitSourceValue { LimitSwitchPin }
            public enum ForwardLimitTypeValue { NormallyOpen, NormallyClosed }
            public enum ReverseLimitSourceValue { LimitSwitchPin }
            public enum ReverseLimitTypeValue { NormallyOpen, NormallyClosed }

            [ConstantInMechInstance]
            public boolParameter enableForward { get; set; }

            [ConstantInMechInstance]
            public intParameter remoteForwardSensorID { get; set; }

            [ConstantInMechInstance]
            public boolParameter forwardResetPosition { get; set; }

            [ConstantInMechInstance]
            public doubleParameter forwardPosition { get; set; }

            [ConstantInMechInstance]
            public ForwardLimitSourceValue forwardType { get; set; }

            [ConstantInMechInstance]
            public ForwardLimitTypeValue forwardOpenClose { get; set; }

            [ConstantInMechInstance]
            public boolParameter enableReverse { get; set; }

            [ConstantInMechInstance]
            public intParameter remoteReverseSensorID { get; set; }

            [ConstantInMechInstance]
            public boolParameter reverseResetPosition { get; set; }

            [ConstantInMechInstance]
            public doubleParameter reversePosition { get; set; }

            [ConstantInMechInstance]
            public ReverseLimitSourceValue revType { get; set; }

            [ConstantInMechInstance]
            public ReverseLimitTypeValue revOpenClose { get; set; }

            public ConfigHWLimitSW()
            {
                defaultDisplayName = "ConfigHWLimitSW";
            }
        }
        public ConfigHWLimitSW theConfigHWLimitSW { get; set; }

        [Serializable]
        public class ConfigMotorSettings : baseDataClass
        {
            [DefaultValue(0)]
            [Range(typeof(double), "0", "100")]
            [PhysicalUnitsFamily(physicalUnit.Family.percent)]
            [ConstantInMechInstance]
            public doubleParameter deadbandPercent { get; set; }

            [DefaultValue(0)]
            [Range(typeof(double), "0", "1.0")]
            [PhysicalUnitsFamily(physicalUnit.Family.none)]
            [ConstantInMechInstance]
            public doubleParameter peakForwardDutyCycle { get; set; }

            [DefaultValue(0)]
            [Range(typeof(double), "-1.0", "0.0")]
            [PhysicalUnitsFamily(physicalUnit.Family.none)]
            [ConstantInMechInstance]
            public doubleParameter peakReverseDutyCycle { get; set; }

            [DefaultValue(InvertedValue.CounterClockwise_Positive)]
            [ConstantInMechInstance]
            public InvertedValue inverted { get; set; }

            [DefaultValue(NeutralModeValue.Coast)]
            [ConstantInMechInstance]
            public NeutralModeValue mode { get; set; }

            public ConfigMotorSettings()
            {
                int index = this.GetType().Name.IndexOf("_");
                if (index > 0)
                    defaultDisplayName = this.GetType().Name.Substring(0, index);
                else
                    defaultDisplayName = this.GetType().Name;
            }
        }
        public ConfigMotorSettings theConfigMotorSettings { get; set; }

        [PhysicalUnitsFamily (physicalUnit.Family.length)]
        [ConstantInMechInstance]
        public doubleParameter diameter { get; set; }

        public TalonFX()
        {
        }

        override public List<string> generateInitialization()
        {
            List<string> initCode = new List<string>();

            initCode.Add(string.Format(@"{0}->SetCurrentLimits({1},
                                            {2}({3}),
                                            {4},
                                            {5}({6}),
                                            {7}({8}),
                                            {9}({10}));",
                                            name, theCurrentLimits.enableStatorCurrentLimit.value.ToString().ToLower(),
                                            generatorContext.theGeneratorConfig.getWPIphysicalUnitType(theCurrentLimits.statorCurrentLimit.__units__), theCurrentLimits.statorCurrentLimit.value,
                                            theCurrentLimits.enableSupplyCurrentLimit.value.ToString().ToLower(),
                                            generatorContext.theGeneratorConfig.getWPIphysicalUnitType(theCurrentLimits.supplyCurrentLimit.__units__), theCurrentLimits.supplyCurrentLimit.value,
                                            generatorContext.theGeneratorConfig.getWPIphysicalUnitType(theCurrentLimits.supplyCurrentThreshold.__units__), theCurrentLimits.supplyCurrentThreshold.value,
                                            generatorContext.theGeneratorConfig.getWPIphysicalUnitType(theCurrentLimits.supplyTimeThreshold.__units__), theCurrentLimits.supplyTimeThreshold.value
                                            ));

            foreach (PIDFslot pIDFslot in PIDFs)
            {
                initCode.Add(string.Format(@"{0}->SetPIDConstants({1}, // slot
                                                                    {2}, // P
                                                                    {3}, // I
                                                                    {4}, // D
                                                                    {5}); // F",
                                            name,
                                            pIDFslot.slot.value,
                                            pIDFslot.pGain.value,
                                            pIDFslot.iGain.value,
                                            pIDFslot.dGain.value,
                                            pIDFslot.fGain.value
                                            ));
            }

            initCode.Add(string.Format(@"{0}->ConfigHWLimitSW({1}, // enableForward
                                            {2}, // remoteForwardSensorID                  
                                            {3}, // forwardResetPosition                 
                                            {4}, // forwardPosition                 
                                            {5}::{6}, // forwardType
                                            {7}::{8}, // forwardOpenClose
                                            {9}, // enableReverse
                                            {10}, // remoteReverseSensorID
                                            {11}, // reverseResetPosition
                                            {12}, // reversePosition
                                            {13}::{14}, // revType
                                            {15}::{16} ); // revOpenClose"
                                            ,
                                            name,
                                            theConfigHWLimitSW.enableForward.value.ToString().ToLower(),
                                            theConfigHWLimitSW.remoteForwardSensorID.value,
                                            theConfigHWLimitSW.forwardResetPosition.value.ToString().ToLower(),
                                            theConfigHWLimitSW.forwardPosition.value,

                                            theConfigHWLimitSW.forwardType.GetType().Name,
                                            theConfigHWLimitSW.forwardType,
                                            theConfigHWLimitSW.forwardOpenClose.GetType().Name,
                                            theConfigHWLimitSW.forwardOpenClose,

                                            theConfigHWLimitSW.enableReverse.value.ToString().ToLower(),
                                            theConfigHWLimitSW.remoteReverseSensorID.value,
                                            theConfigHWLimitSW.reverseResetPosition.value.ToString().ToLower(),
                                            theConfigHWLimitSW.reversePosition.value,

                                            theConfigHWLimitSW.revType.GetType().Name,
                                            theConfigHWLimitSW.revType,
                                            theConfigHWLimitSW.revOpenClose.GetType().Name,
                                            theConfigHWLimitSW.revOpenClose
                                           ));

            initCode.Add(string.Format(@"{0}->ConfigMotorSettings(ctre::phoenixpro::signals::{1}::{2}, // ctre::phoenixpro::signals::InvertedValue
                                            ctre::phoenixpro::signals::{3}::{4}, // ctre::phoenixpro::signals::NeutralModeValue                  
                                            {5}, // deadbandPercent                 
                                            {6}, // peakForwardDutyCycle                 
                                            {7} ); // peakReverseDutyCycle"
                                            ,
                                            name,

                                            theConfigMotorSettings.inverted.GetType().Name,
                                            theConfigMotorSettings.inverted,

                                            theConfigMotorSettings.mode.GetType().Name,
                                            theConfigMotorSettings.mode,

                                            theConfigMotorSettings.deadbandPercent.value,
                                            theConfigMotorSettings.peakForwardDutyCycle.value,
                                            theConfigMotorSettings.peakReverseDutyCycle.value
                                           ));

            initCode.Add(string.Format(@"{0}->SetAsFollowerMotor({1} ); // masterCANID",
                                            name,
                                            followID.value
                                            ));

            initCode.Add(string.Format(@"{0}->SetRemoteSensor({1}, // canID
                                                              {2}::{2}_{3} ); // ctre::phoenix::motorcontrol::RemoteSensorSource",
                                            name,
                                            remoteSensor.CanID.value,
                                            remoteSensor.Source.GetType().Name,
                                            remoteSensor.Source
                                            ));

            if (fusedCANcoder.enable.value == true)
            {
                initCode.Add(string.Format(@"{0}->FuseCancoder(*{1}, // DragonCanCoder &cancoder
                                                               {2}, // sensorToMechanismRatio
                                                               {3} ); // rotorToSensorRatio",
                                            name,
                                            fusedCANcoder.fusedCANcoder.name,
                                            fusedCANcoder.sensorToMechanismRatio.value,
                                            fusedCANcoder.rotorToSensorRatio.value
                                            ));
            }

            initCode.Add(string.Format(@"{0}->SetDiameter({1} ); // double diameter",
                                name,
                                diameter.value
                                ));

            initCode.AddRange(base.generateInitialization());

            return initCode;
        }

        override public List<string> generateIndexedObjectCreation(int currentIndex)
        {
            string creation = string.Format("{0} = new {1}(\"{0}\",RobotElementNames::{2},{3},\"{4}\")",
                name,
                getImplementationName(),
                utilities.ListToString(generateElementNames()).ToUpper().Replace("::", "_USAGE::"),
                canID.value.ToString(),
                canBusName.ToString());

            List<string> code = new List<string>() { "", theDistanceAngleCalcInfo.getDefinition(name), creation };

            code.AddRange(generateObjectAddToMaps());
            code.Add("");

            return code;
        }
    }

    [Serializable]
    public class FeedbackSensorConfigBase : baseRobotElementClass
    {
        [DefaultValue(0)]
        [Range(typeof(int), "0", "3")]
        [ConstantInMechInstance]
        public intParameter pidSlotId { get; set; }

        [DefaultValue(0)]
        [PhysicalUnitsFamily(physicalUnit.Family.time)]
        [ConstantInMechInstance]
        public doubleParameter timeOut { get; set; }

        public FeedbackSensorConfigBase()
        {
        }
    }

    [Serializable]
    public class FeedbackSensorConfig_SRX : FeedbackSensorConfigBase
    {
        public enum TalonSRXFeedbackDevice
        {
            /**
             * Quadrature encoder
             */
            QuadEncoder = 0,
            //1
            /**
             * Analog potentiometer/encoder
             */
            Analog = 2,
            //3
            /**
             * Tachometer
             */
            Tachometer = 4,
            /**
             * CTRE Mag Encoder in Absolute mode or
             * any other device that uses PWM to encode its output
             */
            PulseWidthEncodedPosition = 8,
            /**
             * Sum0 + Sum1
             */
            SensorSum = 9,
            /**
             * Diff0 - Diff1
             */
            SensorDifference = 10,
            /**
             * Sensor configured in RemoteFilter0
             */
            RemoteSensor0 = 11,
            /**
             * Sensor configured in RemoteFilter1
             */
            RemoteSensor1 = 12,
            //13
            /**
             * Position and velocity will read 0.
             */
            None = 14,
            /**
             * Motor Controller will fake a sensor based on applied motor output.
             */
            SoftwareEmulatedSensor = 15,
            /**
             * CTR mag encoder configured in absolute, is the same
             * as a PWM sensor.
             */
            CTRE_MagEncoder_Absolute = PulseWidthEncodedPosition,
            /**
             * CTR mag encoder configured in relative, is the same
             * as an quadrature encoder sensor.
             */
            CTRE_MagEncoder_Relative = QuadEncoder,
        }

        [ConstantInMechInstance]
        public TalonSRXFeedbackDevice device { get; set; }

        public FeedbackSensorConfig_SRX()
        {
        }
    }

    [Serializable]
    public class RemoteFeedbackSensorConfig_SRX : FeedbackSensorConfigBase
    {
        public enum RemoteFeedbackDevice
        {
            /**
             * Use Sum0 + Sum1
             */
            SensorSum = 9,
            /**
             * Use Diff0 - Diff1
             */
            SensorDifference = 10,

            /**
             * Use the sensor configured
             * in filter0
             */
            RemoteSensor0 = 11,
            /**
             * [[deprecated("Use RemoteSensor1 instead.")]]
             * Use the sensor configured
             * in filter1
             */
            RemoteSensor1 = 12,
            /**
             * Position and velocity will read 0.
             */
            None = 14,
            /**
             * Motor Controller will fake a sensor based on applied motor output.
             */
            SoftwareEmulatedSensor = 15,
        };

        [ConstantInMechInstance]
        public RemoteFeedbackDevice device { get; set; }

        public RemoteFeedbackSensorConfig_SRX()
        {
            device = RemoteFeedbackDevice.None;
        }
    }

    [Serializable()]
    [ImplementationName("DragonTalonSRX")]
    [UserIncludeFile("hw/DragonTalonSRX.h")]
    public class TalonSRX : MotorController
    {
        [Serializable]
        public class LimitSwitches : baseDataClass
        {
            [DefaultValue(SwitchConfiguration.NormallyOpen)]
            [ConstantInMechInstance]
            public SwitchConfiguration ForwardLimitSwitch { get; set; }

            [DefaultValue(SwitchConfiguration.NormallyOpen)]
            [ConstantInMechInstance]
            public SwitchConfiguration ReverseLimitSwitch { get; set; }

            [DefaultValue(false)]
            [ConstantInMechInstance]
            public boolParameter LimitSwitchesEnabled { get; set; }

            public LimitSwitches()
            {
                defaultDisplayName = this.GetType().Name;
            }
        }
        public LimitSwitches limitSwitches { get; set; }

        [Serializable]
        public class CurrentLimits_SRX : baseDataClass
        {
            [DefaultValue(false)]
            [ConstantInMechInstance]
            public boolParameter EnableCurrentLimits { get; set; }

            [DefaultValue(0)]
            [PhysicalUnitsFamily(physicalUnit.Family.current)]
            [ConstantInMechInstance]
            public intParameter PeakCurrentLimit { get; set; }

            [DefaultValue(0)]
            [PhysicalUnitsFamily(physicalUnit.Family.time)]
            [ConstantInMechInstance]
            public intParameter PeakCurrentLimitTimeout { get; set; }

            [DefaultValue(0)]
            [PhysicalUnitsFamily(physicalUnit.Family.time)]
            [ConstantInMechInstance]
            public intParameter PeakCurrentDuration { get; set; }

            [DefaultValue(0)]
            [PhysicalUnitsFamily(physicalUnit.Family.time)]
            [ConstantInMechInstance]
            public intParameter PeakCurrentDurationTimeout { get; set; }


            [DefaultValue(0)]
            [PhysicalUnitsFamily(physicalUnit.Family.current)]
            [ConstantInMechInstance]
            public intParameter ContinuousCurrentLimit { get; set; }

            [DefaultValue(0)]
            [PhysicalUnitsFamily(physicalUnit.Family.time)]
            [ConstantInMechInstance]
            public intParameter ContinuousCurrentLimitTimeout { get; set; }

            public CurrentLimits_SRX()
            {
                int index = this.GetType().Name.IndexOf("_");
                if (index > 0)
                    defaultDisplayName = this.GetType().Name.Substring(0, index);
                else
                    defaultDisplayName = this.GetType().Name;
            }
        }
        public CurrentLimits_SRX currentLimits { get; set; }

        [Serializable]
        public class ConfigMotorSettings_SRX : baseDataClass
        {
            [DefaultValue(InvertedValue.CounterClockwise_Positive)]
            [ConstantInMechInstance]
            public InvertedValue inverted { get; set; }

            [DefaultValue(NeutralModeValue.Coast)]
            [ConstantInMechInstance]
            public NeutralModeValue mode { get; set; }

            public ConfigMotorSettings_SRX()
            {
                int index = this.GetType().Name.IndexOf("_");
                if (index > 0)
                    defaultDisplayName = this.GetType().Name.Substring(0, index);
                else
                    defaultDisplayName = this.GetType().Name;
            }
        }
        public ConfigMotorSettings_SRX theConfigMotorSettings { get; set; }

        public List<FeedbackSensorConfig_SRX> feedbackSensorConfig { get; set; }
        public List<RemoteFeedbackSensorConfig_SRX> remoteFeedbackSensorConfig { get; set; }

        public TalonSRX()
        {
        }

        override public List<string> generateInitialization()
        {
            List<string> initCode = new List<string>();

            foreach (RemoteFeedbackSensorConfig_SRX config in remoteFeedbackSensorConfig)
            {

                initCode.Add(string.Format(@"{0}->ConfigSelectedFeedbackSensor({1}::{2},
                                                                                {3}, 
                                                                                units::time::millisecond_t({4}({5})).to<double>());",
                                                                                    name,
                                                                                    "ctre::phoenix::motorcontrol::RemoteFeedbackDevice",
                                                                                    config.device,
                                                                                    config.pidSlotId,
                                                                                    generatorContext.theGeneratorConfig.getWPIphysicalUnitType(config.timeOut.physicalUnits),
                                                                                    config.timeOut
                                                                                    ));
            }

            foreach (FeedbackSensorConfig_SRX config in feedbackSensorConfig)
            {

                initCode.Add(string.Format(@"{0}->ConfigSelectedFeedbackSensor({1}::{2},
                                                                                {3}, 
                                                                                units::time::millisecond_t({4}({5})).to<double>());",
                                                                                    name,
                                                                                    "ctre::phoenix::motorcontrol::FeedbackDevice",
                                                                                    config.device,
                                                                                    config.pidSlotId,
                                                                                    generatorContext.theGeneratorConfig.getWPIphysicalUnitType(config.timeOut.physicalUnits),
                                                                                    config.timeOut
                                                                                    ));
            }

            initCode.Add(string.Format(@"{0}->SetRemoteSensor({1},
                                                              ctre::phoenix::motorcontrol::{2}::{2}_{3} );",
                                                                    name,
                                                                    remoteSensor.CanID.value,
                                                                    remoteSensor.Source.GetType().Name,
                                                                    remoteSensor.Source
                                                                    ));

            initCode.Add(string.Format("{0}->Invert( {1});",
                                                                    name,
                                                                    (theConfigMotorSettings.inverted == InvertedValue.CounterClockwise_Positive).ToString().ToLower()));

            initCode.Add(string.Format("{0}->EnableBrakeMode( {1});",
                                                                    name,
                                                                    (theConfigMotorSettings.mode == NeutralModeValue.Brake).ToString().ToLower()));

            initCode.Add(string.Format("{0}->EnableCurrentLimiting( {1});",
                                                                    name,
                                                                    currentLimits.EnableCurrentLimits.value.ToString().ToLower()));

            initCode.Add(string.Format(@"{0}->ConfigPeakCurrentLimit(units::current::ampere_t ( {1}({2})).to<int>(), 
                                                                     units::time::millisecond_t({3}({4})).to<int>() );",      //todo check return code
                                                                    name,
                                                                    generatorContext.theGeneratorConfig.getWPIphysicalUnitType(currentLimits.PeakCurrentLimit.physicalUnits),
                                                                    currentLimits.PeakCurrentLimit.value,
                                                                    generatorContext.theGeneratorConfig.getWPIphysicalUnitType(currentLimits.PeakCurrentLimitTimeout.physicalUnits),
                                                                    currentLimits.PeakCurrentLimitTimeout.value));

            initCode.Add(string.Format(@"{0}->ConfigPeakCurrentDuration(units::time::millisecond_t ( {1}({2})).to<int>(), 
                                                                        units::time::millisecond_t({3}({4})).to<int>() );",      //todo check return code
                                                                    name,
                                                                    generatorContext.theGeneratorConfig.getWPIphysicalUnitType(currentLimits.PeakCurrentDuration.physicalUnits),
                                                                    currentLimits.PeakCurrentDuration.value,
                                                                    generatorContext.theGeneratorConfig.getWPIphysicalUnitType(currentLimits.PeakCurrentDurationTimeout.physicalUnits),
                                                                    currentLimits.PeakCurrentDurationTimeout.value));

            initCode.Add(string.Format(@"{0}->ConfigContinuousCurrentLimit(units::current::ampere_t ( {1}({2})).to<int>(), 
                                                                           units::time::millisecond_t({3}({4})).to<int>() );",      //todo check return code
                                                                    name,
                                                                    generatorContext.theGeneratorConfig.getWPIphysicalUnitType(currentLimits.ContinuousCurrentLimit.physicalUnits),
                                                                    currentLimits.ContinuousCurrentLimit.value,
                                                                    generatorContext.theGeneratorConfig.getWPIphysicalUnitType(currentLimits.ContinuousCurrentLimitTimeout.physicalUnits),
                                                                    currentLimits.ContinuousCurrentLimitTimeout.value));

            initCode.Add(string.Format("{0}->SetDiameter(units::length::inch_t ( {1}({2})).to<double>());",      //todo Should SetDiameter(double) be called within the constructor, since the diameter is inside the calcStruct that is passed to the constructor?
                                                                    name,
                                                                    generatorContext.theGeneratorConfig.getWPIphysicalUnitType(theDistanceAngleCalcInfo.diameter.physicalUnits),
                                                                    theDistanceAngleCalcInfo.diameter.value));

            initCode.Add(string.Format("{0}->EnableDisableLimitSwitches( {1});",
                                                                    name,
                                                                    limitSwitches.LimitSwitchesEnabled.value.ToString().ToLower()));

            initCode.Add(string.Format("{0}->SetForwardLimitSwitch( {1});",
                                                                    name,
                                                                    (limitSwitches.ForwardLimitSwitch == SwitchConfiguration.NormallyOpen).ToString().ToLower()));

            initCode.Add(string.Format("{0}->SetReverseLimitSwitch( {1});",
                                                                    name,
                                                                    (limitSwitches.ReverseLimitSwitch == SwitchConfiguration.NormallyOpen).ToString().ToLower()));

            if (enableFollowID.value)
            {
                initCode.Add(string.Format("{0}->SetAsFollowerMotor( {1} );",
                                                                        name,
                                                                        followID.value));
            }
            else
                initCode.Add(string.Format("// {0} : Follower motor mode is not enabled", name));

            initCode.AddRange(base.generateInitialization());

            initCode.Add(Environment.NewLine);

            //todo finish the TalonSRX initialization

            return initCode;
        }

        override public List<string> generateIndexedObjectCreation(int currentIndex)
        {
            string creation = string.Format(@"{0} = new {1}(""{0}"",
                                                                RobotElementNames::{2},
                                                                {3},
                                                                {4},
                                                                {5}, 
                                                                IDragonMotorController::MOTOR_TYPE::{6})",
                name,
                getImplementationName(),
                utilities.ListToString(generateElementNames()).ToUpper().Replace("::", "_USAGE::"),
                canID.value.ToString(),
                pdpID.value.ToString(),
                theDistanceAngleCalcInfo.getName(name),
                motorType
                );

            List<string> code = new List<string>() { "", theDistanceAngleCalcInfo.getDefinition(name), creation };

            code.AddRange(generateObjectAddToMaps());
            code.Add("");

            return code;
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
    public class limelight : baseRobotElementClass
    {
        public enum limelightRotation
        {
            Angle_0_deg = 0,
            Angle_90_deg = 90,
            Angle_180_deg = 180,
            Angle_270_deg = 270,
        }
        public enum limelightDefaultLedMode
        {
            currentPipeline,
            off,
            blink,
            on,
        }
        public enum limelightDefaultCamMode
        {
            vision,
            driverCamera,
        }
        public enum limelightStreamMode
        {
            sideBySide,
            pipMain,
            pipSecondary,
        }
        public enum limelightSnapshots
        {
            off,
            twoPerSec,
        }

        [DefaultValue(0.0)]
        [PhysicalUnitsFamily(physicalUnit.Family.length)]
        public doubleParameter mountingheight { get; set; }

        [DefaultValue(0.0)]
        [PhysicalUnitsFamily(physicalUnit.Family.length)]
        public doubleParameter horizontaloffset { get; set; }

        [DefaultValue(0.0)]
        [PhysicalUnitsFamily(physicalUnit.Family.angle)]
        public doubleParameter mountingangle { get; set; }

        [DefaultValue(limelightRotation.Angle_0_deg)]
        [PhysicalUnitsFamily(physicalUnit.Family.angle)]
        public limelightRotation rotation { get; set; }

        public List<doubleParameterUserDefinedTunable> tunableParameters { get; set; }

        [DefaultValue(limelightDefaultLedMode.currentPipeline)]
        public limelightDefaultLedMode defaultledmode { get; set; }

        [DefaultValue(limelightDefaultCamMode.vision)]
        public limelightDefaultCamMode defaultcammode { get; set; }

        [DefaultValue(limelightStreamMode.sideBySide)]
        public limelightStreamMode streammode { get; set; }

        [DefaultValue(limelightSnapshots.off)]
        public limelightSnapshots snapshots { get; set; }

        public limelight()
        {
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

            foreach(pigeon p in Pigeons)
            {
                sb.Add(string.Format("{1}::{0}",ToUnderscoreCase(p.name), ToUnderscoreCase(p.GetType().Name)));
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

        override public List<string> generateObjectCreation()
        {
            string creation = string.Format("{0} = new {1}(\"{0}\",RobotElementNames::{2},{3},{4},{5}({6}))",
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
            string creation = string.Format("{0} = new {1}(\"{0}\",RobotElementNames::{2},{3},\"{4}\",{5},{6})",
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
    public class camera : baseRobotElementClass
    {
        public enum cameraformat
        {
            KMJPEG,
            KYUYV,
            KRGB565,
            KBGR,
            KGRAY,
        }

        [DefaultValue("0")]
        public uintParameter id { get; set; }

        [DefaultValue(cameraformat.KMJPEG)]
        public cameraformat format { get; set; }

        [DefaultValue(640)]
        public uintParameter width { get; set; }

        [DefaultValue(480)]
        public uintParameter height { get; set; }

        [DefaultValue(30)]
        public uintParameter fps { get; set; }

        [DefaultValue(false)]
        public boolParameter thread { get; set; }

        public camera()
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
    public class led : baseRobotElementClass
    {
        [DefaultValue(0u)]
        [Range(typeof(uint), "0", "19")]
        public uintParameter Id { get; set; }

        [DefaultValue(0u)]
        public uintParameter count { get; set; }

        public led()
        {
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
            return new List<string> { string.Format("{0}* {1};", getImplementationName(), name) };
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
        public static mechanism theMechanism { get; set; }
        public static mechanismInstance theMechanismInstance { get; set; }
        public static int stateIndex { get; set; }
        public static applicationData theRobot { get; set; }
        public static toolConfiguration theGeneratorConfig { get; set; }

        public static void clear()
        {
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
                )",
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

    [Serializable()]
    public class state : baseRobotElementClass
    {
        public List<motorControlDataLink> motorControlDataLinks { get; set; }

        [ConstantInMechInstance()]
        public List<stringParameterConstInMechInstance> transitionsTo { get; set; }

        public List<doubleParameterUserDefinedTunableOnlyValueChangeableInMechInst> doubleTargets { get; set; }
        public List<boolParameterUserDefinedTunableOnlyValueChangeableInMechInst> booleanTargets { get; set; }

        public state()
        {
            name = GetType().Name;
            helperFunctions.initializeNullProperties(this);
            helperFunctions.initializeDefaultValues(this);
        }

        public override List<string> generateIncludes()
        {
            List<string> sb = new List<string>();
            if (generatorContext.theMechanismInstance != null)
            {
                sb.Add(string.Format("#include \"mechanisms/{1}/decoratormods/{1}_{0}_State.h\"",
                    name,
                    generatorContext.theMechanismInstance.name));
            }

            return sb;
        }
        public override List<string> generateIndexedObjectCreation(int index)
        {
            if (generatorContext.theMechanismInstance != null)
            {
                string creation = string.Format("{1}{0}State* {0}State = new {1}{0}State(string(\"{0}\"), {2}, new {1}{0}StateGen(string(\"{0}\"), {2}, *this))",
                name,
                generatorContext.theMechanismInstance.name,
                index);

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
            string creation = string.Format("AddToStateVector({0}State)", name);

            return new List<string> { creation };
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
