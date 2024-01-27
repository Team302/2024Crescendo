using applicationConfiguration;
using ApplicationData;
using Configuration;
using DataConfiguration;
using System;
using System.CodeDom.Compiler;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Reflection;
using System.Text;
using System.Threading.Tasks;
using static ApplicationData.motorControlData;

namespace CoreCodeGenerator
{
    internal class MechanismInstanceGenerator : baseGenerator
    {
        internal MechanismInstanceGenerator(string codeGeneratorVersion, applicationDataConfig theRobotConfiguration, toolConfiguration theToolConfiguration, bool cleanMode, bool cleanDecoratorModFolders, showMessage displayProgress)
        : base(codeGeneratorVersion, theRobotConfiguration, theToolConfiguration, cleanMode, cleanDecoratorModFolders)
        {
            setProgressCallback(displayProgress);
        }



        internal void generate()
        {
            addProgress((cleanMode ? "Erasing" : "Writing") + " mechanism instance files...");
            List<string> mechMainFiles = new List<string>();
            //List<string> mechStateFiles = new List<string>();
            //List<string> mechStateMgrFiles = new List<string>();
            List<string> mechInstanceNames = new List<string>();
            foreach (applicationData robot in theRobotConfiguration.theRobotVariants.Robots)
            {
                generatorContext.theRobot = robot;

                List<mechanismInstance> mechInstances = new List<mechanismInstance>();
                mechInstances.AddRange(robot.mechanismInstances);
                mechInstances.AddRange(robot.Chassis.mechanismInstances);

                int index = 0;
                foreach (mechanismInstance mi in mechInstances)
                {
                    if (!mechInstanceNames.Exists(n => n == mi.name))
                    {
                        mechInstanceNames.Add(mi.name);

                        generatorContext.theMechanismInstance = mi;
                        generatorContext.theMechanism = mi.mechanism;

                        string filePathName;
                        string resultString;
                        codeTemplateFile cdf;
                        string template;

                        string mechanismName = mi.name;


                        #region the generated files
                        createMechanismFolder(mechanismName, true);
                        if (index < robot.mechanismInstances.Count)
                            createMechanismFolder(mechanismName, false);

                        #region Generate Cpp File
                        cdf = theToolConfiguration.getTemplateInfo("MechanismInstance_gen_cpp");
                        template = loadTemplate(cdf.templateFilePathName);

                        resultString = template;

                        #region clean up unused stuff
                        if (index < robot.mechanismInstances.Count)
                        {
                            resultString = resultString.Replace("_STATE_MANAGER_START_", "");
                            resultString = resultString.Replace("_STATE_MANAGER_END_", "");
                        }
                        else
                            resultString = Remove(resultString, "_STATE_MANAGER_START_", "_STATE_MANAGER_END_");
                        #endregion

                        resultString = resultString.Replace("$$_MECHANISM_TYPE_NAME_$$", ToUnderscoreCase(mi.name).ToUpper());
                        resultString = resultString.Replace("$$_MECHANISM_NAME_$$", mi.mechanism.name);
                        resultString = resultString.Replace("$$_MECHANISM_INSTANCE_NAME_$$", mi.name);
                        resultString = resultString.Replace("$$_OBJECT_CREATION_$$", ListToString(generateMethod(mi, "generateIndexedObjectCreation"), ";"));

                        List<string> theUsings = generateMethod(mi, "generateUsings").Distinct().ToList();
                        resultString = resultString.Replace("$$_USING_DIRECTIVES_$$", ListToString(theUsings, ";"));

                        List<string> initCode = new List<string>
                        {
                            "if(false){}"
                        };

                        foreach (applicationData r in theRobotConfiguration.theRobotVariants.Robots)
                        {
                            mechanismInstance mis = mechInstances.Find(m => m.name == mi.name);
                            if (mis != null)
                            {
                                initCode.Add(string.Format("else if(RobotConfigMgr::RobotIdentifier::{0} == robotFullName)", ToUnderscoreDigit(ToUnderscoreCase(r.getFullRobotName())).ToUpper()));
                                initCode.Add("{");
                                initCode.AddRange(generateMethod(mis, "generateInitialization"));
                                initCode.Add("}");
                            }
                        }
                        resultString = resultString.Replace("$$_ELEMENT_INITIALIZATION_$$", ListToString(initCode));

                        filePathName = getMechanismFullFilePathName(mechanismName, cdf.outputFilePathName.Replace("MECHANISM_INSTANCE_NAME", mechanismName), true);
                        copyrightAndGenNoticeAndSave(filePathName, resultString);
                        #endregion

                        #region Generate H File
                        cdf = theToolConfiguration.getTemplateInfo("MechanismInstance_gen_h");
                        template = loadTemplate(cdf.templateFilePathName);

                        resultString = template;

                        #region clean up unused stuff
                        if (index < robot.mechanismInstances.Count)
                        {
                            resultString = resultString.Replace("_STATE_MANAGER_START_", "");
                            resultString = resultString.Replace("_STATE_MANAGER_END_", "");
                        }
                        else
                            resultString = Remove(resultString, "_STATE_MANAGER_START_", "_STATE_MANAGER_END_");
                        #endregion

                        resultString = resultString.Replace("$$_MECHANISM_NAME_$$", mi.mechanism.name);
                        resultString = resultString.Replace("$$_MECHANISM_INSTANCE_NAME_$$", mi.name);

                        List<string> enumList = new List<string>();
                        foreach (state s in mi.mechanism.states)
                        {
                            enumList.Add(String.Format("STATE_{0}", ToUnderscoreCase(s.name).ToUpper()));
                        }

                        resultString = resultString.Replace("$$_STATE_NAMES_$$", ListToString(enumList, ", "));

                        filePathName = getMechanismFullFilePathName(mechanismName, cdf.outputFilePathName.Replace("MECHANISM_INSTANCE_NAME", mechanismName), true);
                        copyrightAndGenNoticeAndSave(filePathName, resultString);
                        #endregion

                        if (index < robot.mechanismInstances.Count)
                        {
                            #region Generate CPP baseStateGen File
                            cdf = theToolConfiguration.getTemplateInfo("BaseStateGen_cpp");
                            template = loadTemplate(cdf.templateFilePathName);

                            resultString = template;

                            resultString = resultString.Replace("$$_MECHANISM_NAME_$$", mi.mechanism.name);
                            resultString = resultString.Replace("$$_MECHANISM_INSTANCE_NAME_$$", mi.name);

                            filePathName = getMechanismFullFilePathName(mechanismName, cdf.outputFilePathName.Replace("MECHANISM_INSTANCE_NAME", mechanismName), true);
                            copyrightAndGenNoticeAndSave(filePathName, resultString);
                            #endregion

                            #region Generate H baseStateGen File
                            cdf = theToolConfiguration.getTemplateInfo("BaseStateGen_h");
                            template = loadTemplate(cdf.templateFilePathName);

                            resultString = template;

                            resultString = resultString.Replace("$$_MECHANISM_NAME_$$", mi.mechanism.name);
                            resultString = resultString.Replace("$$_MECHANISM_INSTANCE_NAME_$$", mi.name);

                            filePathName = getMechanismFullFilePathName(mechanismName, cdf.outputFilePathName.Replace("MECHANISM_INSTANCE_NAME", mechanismName), true);
                            copyrightAndGenNoticeAndSave(filePathName, resultString);
                            #endregion

                            #region Generate H StateGen Files
                            foreach (state s in mi.mechanism.states)
                            {
                                cdf = theToolConfiguration.getTemplateInfo("stateGen_h");
                                template = loadTemplate(cdf.templateFilePathName);

                                resultString = template;

                                resultString = resultString.Replace("$$_MECHANISM_NAME_$$", mi.mechanism.name);
                                resultString = resultString.Replace("$$_MECHANISM_INSTANCE_NAME_$$", mi.name);
                                resultString = resultString.Replace("$$_STATE_NAME_$$", s.name);

                                //List<string> targetDecl = new List<string>();
                                //foreach (doubleParameterUserDefinedTunableOnlyValueChangeableInMechInst d in s.doubleTargets)
                                //{
                                //    string varType = "double";
                                //    if (d.unitsFamily != physicalUnit.Family.none)
                                //        varType = generatorContext.theGeneratorConfig.getWPIphysicalUnitType(d.__units__);

                                //    targetDecl.Add(string.Format("{0} {1} = {0}({2})", varType, d.name, d.value));
                                //}
                                //foreach (boolParameterUserDefinedTunableOnlyValueChangeableInMechInst d in s.booleanTargets)
                                //{
                                //    targetDecl.Add(string.Format("bool {0} = {1}", d.name, d.value.ToString().ToLower()));
                                //}
                                //resultString = resultString.Replace("$$_TARGET_DECLARATIONS_$$", ListToString(targetDecl, ";"));
                                resultString = resultString.Replace("$$_TARGET_DECLARATIONS_$$", "");

                                filePathName = getMechanismFullFilePathName(mechanismName,
                                                                            cdf.outputFilePathName.Replace("MECHANISM_INSTANCE_NAME", mechanismName).Replace("STATE_NAME", s.name)
                                                                            , true);
                                copyrightAndGenNoticeAndSave(filePathName, resultString);
                            }
                            #endregion

                            #region Generate CPP StateGen Files
                            foreach (state s in mi.mechanism.states)
                            {
                                cdf = theToolConfiguration.getTemplateInfo("stateGen_cpp");
                                template = loadTemplate(cdf.templateFilePathName);

                                resultString = template;

                                resultString = resultString.Replace("$$_MECHANISM_NAME_$$", mi.mechanism.name);
                                resultString = resultString.Replace("$$_MECHANISM_INSTANCE_NAME_$$", mi.name);
                                resultString = resultString.Replace("$$_STATE_NAME_$$", s.name);

                                List<string> motorTargets = new List<string>();
                                foreach (motorTarget mT in s.motorTargets)
                                {
                                    // find the corresponding motor control Data link
                                    motorControlData mcd = mi.mechanism.stateMotorControlData.Find(cd => cd.name == mT.controlDataName);
                                    if (mcd == null)
                                        addProgress(string.Format("In mechanism {0}, cannot find a Motor control data called {1}, referenced in state {2}", mi.name, mT.controlDataName, s.name));
                                    else
                                    {
                                        //void SetTargetControl(RobotElementNames::MOTOR_CONTROLLER_USAGE identifier, double percentOutput);
                                        //void SetTargetControl(RobotElementNames::MOTOR_CONTROLLER_USAGE identifier, ControlData &controlConst, units::angle::degree_t angle );
                                        //void SetTargetControl(RobotElementNames::MOTOR_CONTROLLER_USAGE identifier, ControlData &controlConst, units::angular_velocity::revolutions_per_minute_t angVel );
                                        //void SetTargetControl(RobotElementNames::MOTOR_CONTROLLER_USAGE identifier, ControlData &controlConst, units::length::inch_t position );
                                        //void SetTargetControl(RobotElementNames::MOTOR_CONTROLLER_USAGE identifier, ControlData &controlConst, units::velocity::feet_per_second_t velocity );

                                        string targetUnitsType = "";
                                        if (mcd.controlType == CONTROL_TYPE.PERCENT_OUTPUT) { }
                                        else if (mcd.controlType == CONTROL_TYPE.POSITION_INCH) { targetUnitsType = "units::length::inch_t"; }
                                        else if (mcd.controlType == CONTROL_TYPE.POSITION_ABS_TICKS) { addProgress("How should we handle POSITION_ABS_TICKS"); }
                                        else if (mcd.controlType == CONTROL_TYPE.POSITION_DEGREES) { targetUnitsType = "units::angle::degree_t"; }
                                        else if (mcd.controlType == CONTROL_TYPE.POSITION_DEGREES_ABSOLUTE) { targetUnitsType = "units::angle::degree_t"; }
                                        else if (mcd.controlType == CONTROL_TYPE.VELOCITY_INCH) { targetUnitsType = "units::velocity::feet_per_second_t"; }
                                        else if (mcd.controlType == CONTROL_TYPE.VELOCITY_DEGREES) { targetUnitsType = "units::angular_velocity::revolutions_per_minute_t"; }
                                        else if (mcd.controlType == CONTROL_TYPE.VELOCITY_RPS) { targetUnitsType = "units::angular_velocity::revolutions_per_minute_t"; }
                                        else if (mcd.controlType == CONTROL_TYPE.VOLTAGE) { targetUnitsType = "units::angular_velocity::revolutions_per_minute_t"; }
                                        else if (mcd.controlType == CONTROL_TYPE.CURRENT) { addProgress("How should we handle CURRENT"); }
                                        else if (mcd.controlType == CONTROL_TYPE.TRAPEZOID_LINEAR_POS) { targetUnitsType = "units::length::inch_t"; }
                                        else if (mcd.controlType == CONTROL_TYPE.TRAPEZOID_ANGULAR_POS) { targetUnitsType = "units::angle::degree_t"; }

                                        MotorController mc = mi.mechanism.MotorControllers.Find(m => m.name == mT.motorName);
                                        if (mc == null)
                                        {
                                            addProgress(string.Format("In mechanism {0}, cannot find a Motor controller called {1}, referenced in state {2}, target {3}", mi.name, mT.motorName, s.name, mT.name));
                                        }
                                        else
                                        {
                                            string motorEnumName =String.Format("RobotElementNames::{0}", ListToString(mc.generateElementNames(),"").Trim().Replace("::","_USAGE::").ToUpper());
                                            if (targetUnitsType == "")
                                                motorTargets.Add(String.Format("SetTargetControl({0}, {1})", motorEnumName, mT.target.value));
                                            else
                                                motorTargets.Add(String.Format("SetTargetControl({0}, *(Get{1}()->{2}), {5}({3}({4})))",
                                                    motorEnumName,
                                                    mi.name,
                                                    mcd.name,
                                                    generatorContext.theGeneratorConfig.getWPIphysicalUnitType(mT.target.physicalUnits),
                                                    mT.target.value,
                                                    targetUnitsType));
                                        }
                                    }
                                }

                                resultString = resultString.Replace("$$_SET_TARGET_CONTROL_$$", ListToString(motorTargets, ";"));

                                filePathName = getMechanismFullFilePathName(mechanismName,
                                                                            cdf.outputFilePathName.Replace("MECHANISM_INSTANCE_NAME", mechanismName).Replace("STATE_NAME", s.name)
                                                                            , true);
                                copyrightAndGenNoticeAndSave(filePathName, resultString);
                            }
                            #endregion

                            #endregion

                            #region The decorator mod files
                            createMechanismFolder(mechanismName, false);

                            #region Generate Cpp File
                            cdf = theToolConfiguration.getTemplateInfo("MechanismInstance_cpp");
                            template = loadTemplate(cdf.templateFilePathName);

                            resultString = template;

                            resultString = resultString.Replace("$$_MECHANISM_INSTANCE_NAME_$$", mi.name);

                            List<string> stateTransitions = new List<string>();
                            List<string> statesCreation = new List<string>();
                            int stateIndex = 0;
                            foreach (state s in mi.mechanism.states)
                            {
                                statesCreation.AddRange(s.generateIndexedObjectCreation(stateIndex));
                                stateIndex++;

                                if (s.transitionsTo.Count > 0)
                                {
                                    foreach (stringParameterConstInMechInstance transition in s.transitionsTo)
                                    {
                                        stateTransitions.Add(String.Format("{0}State->RegisterTransitionState({1}State)", s.name, transition.value));
                                    }
                                }
                                else
                                {
                                    stateTransitions.Add(String.Format("{0}State->RegisterTransitionState({0}State)", s.name));
                                }
                            }
                            resultString = resultString.Replace("$$_OBJECT_CREATION_$$", ListToString(statesCreation, ";"));
                            resultString = resultString.Replace("$$_STATE_TRANSITION_REGISTRATION_$$", ListToString(stateTransitions, ";"));

                            resultString = resultString.Replace("$$_STATE_CLASSES_INCLUDES_$$", ListToString(generateMethod(mi, "generateIncludes"), ""));

                            filePathName = getMechanismFullFilePathName(mechanismName, cdf.outputFilePathName.Replace("MECHANISM_INSTANCE_NAME", mechanismName), false);
                            copyrightAndGenNoticeAndSave(filePathName, resultString, true);
                            #endregion

                            #region Generate H File
                            cdf = theToolConfiguration.getTemplateInfo("MechanismInstance_h");
                            template = loadTemplate(cdf.templateFilePathName);

                            resultString = template;

                            resultString = resultString.Replace("$$_MECHANISM_INSTANCE_NAME_$$", mi.name);

                            filePathName = getMechanismFullFilePathName(mechanismName, cdf.outputFilePathName.Replace("MECHANISM_INSTANCE_NAME", mechanismName), false);
                            copyrightAndGenNoticeAndSave(filePathName, resultString, true);
                            #endregion

                            #region Generate H StateGen_Decorator Files
                            foreach (state s in mi.mechanism.states)
                            {
                                cdf = theToolConfiguration.getTemplateInfo("stateGen_Decorator_h");
                                template = loadTemplate(cdf.templateFilePathName);

                                resultString = template;

                                resultString = resultString.Replace("$$_MECHANISM_NAME_$$", mi.mechanism.name);
                                resultString = resultString.Replace("$$_MECHANISM_INSTANCE_NAME_$$", mi.name);
                                resultString = resultString.Replace("$$_STATE_NAME_$$", s.name);

                                filePathName = getMechanismFullFilePathName(mechanismName,
                                                                            cdf.outputFilePathName.Replace("MECHANISM_INSTANCE_NAME", mechanismName).Replace("STATE_NAME", s.name)
                                                                            , false);
                                copyrightAndGenNoticeAndSave(filePathName, resultString, true);
                            }
                            #endregion

                            #region Generate CPP StateGen_Decorator Files
                            foreach (state s in mi.mechanism.states)
                            {
                                cdf = theToolConfiguration.getTemplateInfo("stateGen_Decorator_cpp");
                                template = loadTemplate(cdf.templateFilePathName);

                                resultString = template;

                                resultString = resultString.Replace("$$_MECHANISM_NAME_$$", mi.mechanism.name);
                                resultString = resultString.Replace("$$_MECHANISM_INSTANCE_NAME_$$", mi.name);
                                resultString = resultString.Replace("$$_STATE_NAME_$$", s.name);

                                filePathName = getMechanismFullFilePathName(mechanismName,
                                                                            cdf.outputFilePathName.Replace("MECHANISM_INSTANCE_NAME", mechanismName).Replace("STATE_NAME", s.name)
                                                                            , false);
                                copyrightAndGenNoticeAndSave(filePathName, resultString, true);
                            }
                            #endregion

                            #endregion
                        }

                        if (cleanMode)
                        {
                            Directory.Delete(getMechanismOutputPath(mechanismName, true));
                            if (cleanDecoratorModFolders)
                            {
                                Directory.Delete(getMechanismOutputPath(mechanismName, false));
                                Directory.Delete(Path.Combine(getMechanismOutputPath(mechanismName, true), ".."));
                            }
                        }
                    }

                    index++;
                }
            }
        }

        internal string getIncludePath(string mechanismName, bool generated)
        {
            return getMechanismOutputPath(mechanismName, generated).Replace(theToolConfiguration.rootOutputFolder, "").Replace(@"\", "/").TrimStart('/');
        }

        internal void createMechanismFolder(string mechanismName, bool generated)
        {
            Directory.CreateDirectory(getMechanismOutputPath(mechanismName, generated));
        }

        internal string getMechanismFullFilePathName(string mechanismName, string templateFilePath, bool generated)
        {
            string filename = Path.GetFileName(templateFilePath);

            filename = filename.Replace("MECHANISM_NAME", mechanismName);

            return Path.Combine(getMechanismOutputPath(mechanismName, generated), filename);
        }

        internal string getMechanismOutputPath(string mechanismName, bool generated)
        {
            return Path.Combine(theToolConfiguration.rootOutputFolder, "mechanisms", mechanismName, generated ? "generated" : "decoratormods");
        }

    }
}
