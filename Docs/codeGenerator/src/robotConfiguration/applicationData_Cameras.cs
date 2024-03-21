using ApplicationData;
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
using static ApplicationData.Limelight;
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
    [XmlInclude(typeof(Limelight))]
    [XmlInclude(typeof(PhotonCam))]
    public class Camera : baseRobotElementClass
    {
        public enum cameraPipeline
        {
            OFF,
            UNKNOWN,
            MACHINE_LEARNING,
            APRIL_TAG,
            COLOR_THRESHOLD
        }



        [DefaultValue(cameraPipeline.OFF)]
        public cameraPipeline pipeline { get; set; }

        [DefaultValue(0)]
        [PhysicalUnitsFamily(physicalUnit.Family.length)]
        public doubleParameter mountingXOffset { get; set; }

        [DefaultValue(0)]
        [PhysicalUnitsFamily(physicalUnit.Family.length)]
        public doubleParameter mountingYOffset { get; set; }

        [DefaultValue(0)]
        [PhysicalUnitsFamily(physicalUnit.Family.length)]
        public doubleParameter mountingZOffset { get; set; }

        [DefaultValue(0.0)]
        [PhysicalUnitsFamily(physicalUnit.Family.angle)]
        public doubleParameter pitch { get; set; }

        [DefaultValue(0.0)]
        [PhysicalUnitsFamily(physicalUnit.Family.angle)]
        public doubleParameter yaw { get; set; }

        [DefaultValue(0.0)]
        [PhysicalUnitsFamily(physicalUnit.Family.angle)]
        public doubleParameter roll { get; set; }

        public Camera()
        {
        }
    }
    
    [Serializable()]
    [ImplementationName("DragonLimelight")]
    [UserIncludeFile("DragonVision/DragonLimelight.h")]
    [UserIncludeFile("DragonVision/DragonVision.h")]
    public class Limelight : Camera
    {
        public enum ledMode
        {
            LED_DEFAULT,
            LED_OFF,
            LED_BLINK,
            LED_ON
        }

        public enum camMode
        {
            CAM_VISION,
            CAM_DRIVER
        }

        public enum streamMode
        {
            STREAM_DEFAULT,
            STREAM_MAIN_AND_SECOND,
            STREAM_SECOND_AND_MAIN
        }

        public enum snapshotMode
        {
            SNAP_OFF,
            SNAP_ON
        }

        [DefaultValue(ledMode.LED_OFF)]
        public ledMode LedMode { get; set; }

        [DefaultValue(camMode.CAM_VISION)]
        public camMode CamMode { get; set; }

        [DefaultValue(streamMode.STREAM_DEFAULT)]
        public streamMode StreamMode { get; set; }

        [DefaultValue(snapshotMode.SNAP_OFF)]
        public snapshotMode SnapshotMode { get; set; }

        public override List<string> generateIndexedObjectCreation(int index)
        {
            string creation = string.Format(@"{0} = new {1} ( ""limelight-{19}"", //std::string name,                      /// <I> - network table name
                                                            DragonCamera::PIPELINE::{2}, //PIPELINE initialPipeline,              /// <I> enum for starting pipeline
                                                           {4}({3}), //units::length::inch_t mountingXOffset, /// <I> x offset of cam from robot center (forward relative to robot)
                                                            {6}({5}), //units::length::inch_t mountingYOffset, /// <I> y offset of cam from robot center (left relative to robot)
                                                            {8}({7}), //units::length::inch_t mountingZOffset, /// <I> z offset of cam from robot center (up relative to robot)
                                                            {10}({9}), //units::angle::degree_t pitch,          /// <I> - Pitch of camera
                                                            {12}({11}), //units::angle::degree_t yaw,            /// <I> - Yaw of camera
                                                            {14}({13}), //units::angle::degree_t roll,           /// <I> - Roll of camera
                                                            DragonLimelight::LED_MODE::{15}, //LED_MODE ledMode,
                                                            DragonLimelight::CAM_MODE::{16}, //CAM_MODE camMode,
                                                            DragonLimelight::STREAM_MODE::{17}, //STREAM_MODE streamMode,
                                                            DragonLimelight::SNAPSHOT_MODE::{18});//SNAPSHOT_MODE snapMode);",
                                            name,
                                            getImplementationName(),
                                            pipeline.ToString(),
                                            mountingXOffset.value,
                                            generatorContext.theGeneratorConfig.getWPIphysicalUnitType(mountingXOffset.__units__),
                                            mountingYOffset.value,
                                            generatorContext.theGeneratorConfig.getWPIphysicalUnitType(mountingYOffset.__units__),
                                            mountingZOffset.value,
                                            generatorContext.theGeneratorConfig.getWPIphysicalUnitType(mountingZOffset.__units__),
                                            pitch.value,
                                            generatorContext.theGeneratorConfig.getWPIphysicalUnitType(pitch.__units__),
                                            yaw.value,
                                            generatorContext.theGeneratorConfig.getWPIphysicalUnitType(yaw.__units__),
                                            roll.value,
                                            generatorContext.theGeneratorConfig.getWPIphysicalUnitType(roll.__units__),
                                            LedMode.ToString(),
                                            CamMode.ToString(),
                                            StreamMode.ToString(),
                                            SnapshotMode.ToString(),
                                            name.ToLower()
                                            );
            string addCamera = string.Format(@"DragonVision::GetDragonVision()->AddCamera({0}, RobotElementNames::CAMERA_USAGE::{1});",name,ToUnderscoreCase( name).ToUpper());
            return new List<string> { creation, addCamera, Environment.NewLine };
        }
    }
    
    [Serializable()]
    [ImplementationName("DragonPhotonCam")]
    [UserIncludeFile("DragonVision/DragonPhotonCam.h")]
    [UserIncludeFile("DragonVision/DragonVision.h")]
    public class PhotonCam : Camera
    {
        public override List<string> generateIndexedObjectCreation(int index)
        {
            string creation = string.Format(@"{0} = new {1} ( ""{0}"", //std::string name,                      /// <I> - network table name
                                                            DragonCamera::PIPELINE::{2}, //PIPELINE initialPipeline,              /// <I> enum for starting pipeline
                                                           {4}({3}), //units::length::inch_t mountingXOffset, /// <I> x offset of cam from robot center (forward relative to robot)
                                                           {6}({5}), //units::length::inch_t mountingYOffset, /// <I> y offset of cam from robot center (left relative to robot)
                                                           {8}({7}), //units::length::inch_t mountingZOffset, /// <I> z offset of cam from robot center (up relative to robot)
                                                           {10}({9}), //units::angle::degree_t pitch,          /// <I> - Pitch of camera
                                                           {12}({11}), //units::angle::degree_t yaw,            /// <I> - Yaw of camera
                                                           {14}({13})); //units::angle::degree_t roll,           /// <I> - Roll of camera
",
                                            name,
                                            getImplementationName(),
                                            pipeline.ToString(),
                                            mountingXOffset.value,
                                            generatorContext.theGeneratorConfig.getWPIphysicalUnitType(mountingXOffset.__units__),
                                            mountingYOffset.value,
                                            generatorContext.theGeneratorConfig.getWPIphysicalUnitType(mountingYOffset.__units__),
                                            mountingZOffset.value,
                                            generatorContext.theGeneratorConfig.getWPIphysicalUnitType(mountingZOffset.__units__),
                                            pitch.value,
                                            generatorContext.theGeneratorConfig.getWPIphysicalUnitType(pitch.__units__),
                                            yaw.value,
                                            generatorContext.theGeneratorConfig.getWPIphysicalUnitType(yaw.__units__),
                                            roll.value,
                                            generatorContext.theGeneratorConfig.getWPIphysicalUnitType(roll.__units__)
                                            );
            string addCamera = string.Format(@"DragonVision::GetDragonVision()->AddCamera({0}, RobotElementNames::CAMERA_USAGE::{1});", name, ToUnderscoreCase(name).ToUpper());
            return new List<string> { creation, addCamera, Environment.NewLine };
        }
    }
}