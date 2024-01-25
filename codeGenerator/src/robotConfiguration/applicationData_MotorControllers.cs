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
    [Serializable()]
    [XmlInclude(typeof(TalonFX))]
    [XmlInclude(typeof(TalonSRX))]
    [XmlInclude(typeof(SparkMax))]
    [XmlInclude(typeof(SparkFlex))]
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
    [Using("ctre::phoenix6::signals::ForwardLimitSourceValue")]
    [Using("ctre::phoenix6::signals::ForwardLimitTypeValue")]
    [Using("ctre::phoenix6::signals::ReverseLimitSourceValue")]
    [Using("ctre::phoenix6::signals::ReverseLimitTypeValue")]
    [Using("ctre::phoenix6::signals::InvertedValue")]
    [Using("ctre::phoenix6::signals::NeutralModeValue")]
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

        [PhysicalUnitsFamily(physicalUnit.Family.length)]
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

            initCode.Add(string.Format(@"{0}->ConfigMotorSettings(ctre::phoenix6::signals::{1}::{2}, // ctre::phoenixpro::signals::InvertedValue
                                            ctre::phoenix6::signals::{3}::{4}, // ctre::phoenixpro::signals::NeutralModeValue                  
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
            string creation = string.Format("{0} = new {1}(\"{0}\",RobotElementNames::{2},{3}, {4}, IDragonMotorController::MOTOR_TYPE::{5}, \"{6}\")",
                name,
                getImplementationName(),
                utilities.ListToString(generateElementNames()).ToUpper().Replace("::", "_USAGE::"),
                canID.value.ToString(),
                theDistanceAngleCalcInfo.getName(name),
                motorType,
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

    [Serializable]
    [ImplementationName("DragonTalonSRX")]
    [UserIncludeFile("hw/DragonTalonSRX.h")]
    public class    TalonSRX : MotorController
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



    [Serializable]
    [NotUserAddable]
    public class SparkController : MotorController
    {
        public enum Type { kBrushed = 0, kBrushless = 1 };
        public enum SensorType { kNoSensor = 0, kHallSensor = 1, kQuadrature = 2 }
        public enum ControlType
        {
            kDutyCycle = 0,
            kVelocity = 1,
            kVoltage = 2,
            kPosition = 3,
            kSmartMotion = 4,
            kCurrent = 5,
            kSmartVelocity = 6
        }

        public enum ParameterStatus
        {
            kOK = 0,
            kInvalidID = 1,
            kMismatchType = 2,
            kAccessMode = 3,
            kInvalid = 4,
            kNotImplementedDeprecated = 5,
        }

        public enum PeriodicFrame
        {
            kStatus0 = 0,
            kStatus1 = 1,
            kStatus2 = 2,
            kStatus3 = 3,
            kStatus4 = 4,
            kStatus5 = 5,
            kStatus6 = 6,
            kStatus7 = 7,
        }

        [Serializable]
        public class CurrentLimits_SparkController : baseDataClass
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

            public CurrentLimits_SparkController()
            {
                int index = this.GetType().Name.IndexOf("_");
                if (index > 0)
                    defaultDisplayName = this.GetType().Name.Substring(0, index);
                else
                    defaultDisplayName = this.GetType().Name;
            }
        }
        public CurrentLimits_SparkController currentLimits { get; set; }

        [Serializable]
        public class ConfigMotorSettings_SparkController : baseDataClass
        {
            [DefaultValue(InvertedValue.CounterClockwise_Positive)]
            [ConstantInMechInstance]
            public InvertedValue inverted { get; set; }

            [DefaultValue(NeutralModeValue.Coast)]
            [ConstantInMechInstance]
            public NeutralModeValue mode { get; set; }

            public ConfigMotorSettings_SparkController()
            {
                int index = this.GetType().Name.IndexOf("_");
                if (index > 0)
                    defaultDisplayName = this.GetType().Name.Substring(0, index);
                else
                    defaultDisplayName = this.GetType().Name;
            }
        }
        public ConfigMotorSettings_SparkController theConfigMotorSettings { get; set; }

        [DefaultValue(1.0)]
        [ConstantInMechInstance]
        public doubleParameter RotationOffset { get; set; }


        public Type motorBrushType { get; set; }

        public SensorType sensorType { get; set; }
    }


    [Serializable]
    [ImplementationName("DragonSparkMax")]
    [UserIncludeFile("hw/DragonSparkMax.h")]
    public class SparkMax : SparkController
    {
        override public List<string> generateIndexedObjectCreation(int currentIndex)
        {
            string creation = string.Format("{0} = new {1}({2},RobotElementNames::{3},rev::CANSparkMax::MotorType::{4},rev::SparkRelativeEncoder::Type::{5},{6})",
                name,
                getImplementationName(),
                canID.value.ToString(),
                utilities.ListToString(generateElementNames()).ToUpper().Replace("::", "_USAGE::"),
                motorBrushType,
                sensorType,
                theDistanceAngleCalcInfo.gearRatio); //todo: install notepad++ :)


            List<string> code = new List<string>() { "", theDistanceAngleCalcInfo.getDefinition(name), creation };

            code.AddRange(generateObjectAddToMaps());
            code.Add("");

            return code;
        }

        override public List<string> generateInitialization()
        {
            List<string> initCode = new List<string>();



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

            //initCode.Add(string.Format(@"{0}->ConfigPeakCurrentLimit(units::current::ampere_t ( {1}({2})).to<int>(), 
            //                                                         units::time::millisecond_t({3}({4})).to<int>() );",      //todo check return code
            //                                                        name,
            //                                                        generatorContext.theGeneratorConfig.getWPIphysicalUnitType(currentLimits.PeakCurrentLimit.physicalUnits),
            //                                                        currentLimits.PeakCurrentLimit.value,
            //                                                        generatorContext.theGeneratorConfig.getWPIphysicalUnitType(currentLimits.PeakCurrentLimitTimeout.physicalUnits),
            //                                                        currentLimits.PeakCurrentLimitTimeout.value));

            //initCode.Add(string.Format(@"{0}->ConfigPeakCurrentDuration(units::time::millisecond_t ( {1}({2})).to<int>(), 
            //                                                            units::time::millisecond_t({3}({4})).to<int>() );",      //todo check return code
            //                                                        name,
            //                                                        generatorContext.theGeneratorConfig.getWPIphysicalUnitType(currentLimits.PeakCurrentDuration.physicalUnits),
            //                                                        currentLimits.PeakCurrentDuration.value,
            //                                                        generatorContext.theGeneratorConfig.getWPIphysicalUnitType(currentLimits.PeakCurrentDurationTimeout.physicalUnits),
            //                                                        currentLimits.PeakCurrentDurationTimeout.value));

            //initCode.Add(string.Format(@"{0}->ConfigContinuousCurrentLimit(units::current::ampere_t ( {1}({2})).to<int>(), 
            //                                                               units::time::millisecond_t({3}({4})).to<int>() );",      //todo check return code
            //                                                        name,
            //                                                        generatorContext.theGeneratorConfig.getWPIphysicalUnitType(currentLimits.ContinuousCurrentLimit.physicalUnits),
            //                                                        currentLimits.ContinuousCurrentLimit.value,
            //                                                        generatorContext.theGeneratorConfig.getWPIphysicalUnitType(currentLimits.ContinuousCurrentLimitTimeout.physicalUnits),
            //                                                        currentLimits.ContinuousCurrentLimitTimeout.value));

            initCode.Add(string.Format("{0}->SetDiameter(units::length::inch_t ( {1}({2})).to<double>());",      //todo Should SetDiameter(double) be called within the constructor, since the diameter is inside the calcStruct that is passed to the constructor?
                                                                    name,
                                                                    generatorContext.theGeneratorConfig.getWPIphysicalUnitType(theDistanceAngleCalcInfo.diameter.physicalUnits),
                                                                    theDistanceAngleCalcInfo.diameter.value));

            //initCode.Add(string.Format("{0}->EnableDisableLimitSwitches( {1});",
            //                                                        name,
            //                                                        limitSwitches.LimitSwitchesEnabled.value.ToString().ToLower()));

            //initCode.Add(string.Format("{0}->SetForwardLimitSwitch( {1});",
            //                                                        name,
            //                                                        (limitSwitches.ForwardLimitSwitch == SwitchConfiguration.NormallyOpen).ToString().ToLower()));

            //initCode.Add(string.Format("{0}->SetReverseLimitSwitch( {1});",
            //                                                        name,
            //                                                        (limitSwitches.ReverseLimitSwitch == SwitchConfiguration.NormallyOpen).ToString().ToLower()));

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


            return initCode;
        }
    }

    [Serializable]
    [ImplementationName("DragonSparkFlex")]
    [UserIncludeFile("hw/DragonSparkFlex.h")]
    public class SparkFlex : SparkController
    {
        override public List<string> generateIndexedObjectCreation(int currentIndex)
        {
            string creation = string.Format("{0} = new {1}({2},RobotElementNames::{3},rev::CANSparkFlex::MotorType::{4},rev::SparkRelativeEncoder::Type::{5},{6})",
                name,
                getImplementationName(),
                canID.value.ToString(),
                utilities.ListToString(generateElementNames()).ToUpper().Replace("::", "_USAGE::"),
                motorBrushType,
                sensorType,
                theDistanceAngleCalcInfo.gearRatio); //todo: install notepad++ :)


            List<string> code = new List<string>() { "", theDistanceAngleCalcInfo.getDefinition(name), creation };

            code.AddRange(generateObjectAddToMaps());
            code.Add("");

            return code;
        }

        override public List<string> generateInitialization()
        {
            List<string> initCode = new List<string>();



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

            //initCode.Add(string.Format(@"{0}->ConfigPeakCurrentLimit(units::current::ampere_t ( {1}({2})).to<int>(), 
            //                                                         units::time::millisecond_t({3}({4})).to<int>() );",      //todo check return code
            //                                                        name,
            //                                                        generatorContext.theGeneratorConfig.getWPIphysicalUnitType(currentLimits.PeakCurrentLimit.physicalUnits),
            //                                                        currentLimits.PeakCurrentLimit.value,
            //                                                        generatorContext.theGeneratorConfig.getWPIphysicalUnitType(currentLimits.PeakCurrentLimitTimeout.physicalUnits),
            //                                                        currentLimits.PeakCurrentLimitTimeout.value));

            //initCode.Add(string.Format(@"{0}->ConfigPeakCurrentDuration(units::time::millisecond_t ( {1}({2})).to<int>(), 
            //                                                            units::time::millisecond_t({3}({4})).to<int>() );",      //todo check return code
            //                                                        name,
            //                                                        generatorContext.theGeneratorConfig.getWPIphysicalUnitType(currentLimits.PeakCurrentDuration.physicalUnits),
            //                                                        currentLimits.PeakCurrentDuration.value,
            //                                                        generatorContext.theGeneratorConfig.getWPIphysicalUnitType(currentLimits.PeakCurrentDurationTimeout.physicalUnits),
            //                                                        currentLimits.PeakCurrentDurationTimeout.value));

            //initCode.Add(string.Format(@"{0}->ConfigContinuousCurrentLimit(units::current::ampere_t ( {1}({2})).to<int>(), 
            //                                                               units::time::millisecond_t({3}({4})).to<int>() );",      //todo check return code
            //                                                        name,
            //                                                        generatorContext.theGeneratorConfig.getWPIphysicalUnitType(currentLimits.ContinuousCurrentLimit.physicalUnits),
            //                                                        currentLimits.ContinuousCurrentLimit.value,
            //                                                        generatorContext.theGeneratorConfig.getWPIphysicalUnitType(currentLimits.ContinuousCurrentLimitTimeout.physicalUnits),
            //                                                        currentLimits.ContinuousCurrentLimitTimeout.value));

            initCode.Add(string.Format("{0}->SetDiameter(units::length::inch_t ( {1}({2})).to<double>());",      //todo Should SetDiameter(double) be called within the constructor, since the diameter is inside the calcStruct that is passed to the constructor?
                                                                    name,
                                                                    generatorContext.theGeneratorConfig.getWPIphysicalUnitType(theDistanceAngleCalcInfo.diameter.physicalUnits),
                                                                    theDistanceAngleCalcInfo.diameter.value));

            //initCode.Add(string.Format("{0}->EnableDisableLimitSwitches( {1});",
            //                                                        name,
            //                                                        limitSwitches.LimitSwitchesEnabled.value.ToString().ToLower()));

            //initCode.Add(string.Format("{0}->SetForwardLimitSwitch( {1});",
            //                                                        name,
            //                                                        (limitSwitches.ForwardLimitSwitch == SwitchConfiguration.NormallyOpen).ToString().ToLower()));

            //initCode.Add(string.Format("{0}->SetReverseLimitSwitch( {1});",
            //                                                        name,
            //                                                        (limitSwitches.ReverseLimitSwitch == SwitchConfiguration.NormallyOpen).ToString().ToLower()));

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


            return initCode;
        }
    }
}
