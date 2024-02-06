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

[Serializable()]
public class camera : baseRobotElementClass
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
    public uintParameter mountingXOffset { get; set; }

    [DefaultValue(0)]
    [PhysicalUnitsFamily(physicalUnit.Family.length)]
    public uintParameter mountingYOffset { get; set; }

    [DefaultValue(0)]
    [PhysicalUnitsFamily(physicalUnit.Family.length)]
    public uintParameter mountingZOffset { get; set; }

    [DefaultValue(0.0)]
    [PhysicalUnitsFamily(physicalUnit.Family.angle)]
    public doubleParameter pitch { get; set; }

    [DefaultValue(0.0)]
    [PhysicalUnitsFamily(physicalUnit.Family.angle)]
    public doubleParameter yaw { get; set; }

    [DefaultValue(0.0)]
    [PhysicalUnitsFamily(physicalUnit.Family.angle)]
    public doubleParameter roll { get; set; }

    public camera()
    {
    }
}
[Serializable()]
public class limelight : camera
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
}
[Serializable()]
public class PhotonCam : camera
{

}