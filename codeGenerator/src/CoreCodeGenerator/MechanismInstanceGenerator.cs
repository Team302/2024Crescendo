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
using System.Security.Cryptography.X509Certificates;
using System.Text;
using System.Threading.Tasks;
//using ApplicationData.motorControlData;

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
                generatorContext.theRobotVariants = theRobotConfiguration.theRobotVariants;
                generatorContext.theRobot = robot;

                List<mechanismInstance> mechInstances = new List<mechanismInstance>();
                mechInstances.AddRange(robot.mechanismInstances);
                mechInstances.AddRange(robot.Chassis.mechanismInstances);

                int index = 0;
                foreach (mechanismInstance mi in mechInstances)
                {
                    generatorContext.generationStage = generatorContext.GenerationStage.MechInstanceGen;

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

                        string createFunctionDeclarations;
                        resultString = resultString.Replace("$$_CREATE_FUNCTIONS_$$", GenerateCreateFunctions(mi, out createFunctionDeclarations));
                        string initializationFunctionDeclarations;
                        resultString = resultString.Replace("$$_INITIALZATION_FUNCTIONS_$$", GenerateInitializationFunctions(mi, out initializationFunctionDeclarations));

                        resultString = resultString.Replace("$$_MECHANISM_TYPE_NAME_$$", ToUnderscoreCase(mi.name).ToUpper());
                        resultString = resultString.Replace("$$_MECHANISM_NAME_$$", mi.mechanism.name);
                        resultString = resultString.Replace("$$_MECHANISM_INSTANCE_NAME_$$", mi.name);
                        resultString = resultString.Replace("$$_MECHANISM_INSTANCE_NAME_UPPER_CASE_$$", ToUnderscoreCase(mi.name).ToUpper());


                        List<string> theUsings = generateMethod(mi, "generateUsings").Distinct().ToList();
                        resultString = resultString.Replace("$$_USING_DIRECTIVES_$$", ListToString(theUsings, ";").Trim());


                        List<string> enumMapList = new List<string>();
                        foreach (state s in mi.mechanism.states)
                        {
                            enumMapList.Add(String.Format("{{\"STATE_{0}\", {1}Gen::STATE_NAMES::STATE_{0}}}", ToUnderscoreCase(s.name).ToUpper(), mi.name));
                        }
                        resultString = resultString.Replace("$$_STATE_MAP_$$", ListToString(enumMapList, ",").Trim());


                        #region Tunable Parameters
                        string allParameterReading = "";
                        string allParameterWriting = "";

                        foreach (motorControlData mcd in mi.mechanism.stateMotorControlData)
                        {
                            if (mcd.controlType != motorControlData.CONTROL_TYPE.PERCENT_OUTPUT)
                            {
                                object obj = mcd.PID;
                                Type objType = obj.GetType();

                                PropertyInfo[] propertyInfos = objType.GetProperties();

                                foreach (PropertyInfo pi in propertyInfos)
                                {
                                    bool skip = (pi.Name == "name");
                                    if (!skip)
                                    {
                                        string setItem = pi.Name == "iZone" ? "IZone" : pi.Name.Replace("Gain", "").ToUpper();
                                        allParameterReading += string.Format("{0}->Set{4}( m_table.get()->GetNumber(\"{0}_{1}\", {2}));{3}", mcd.name, pi.Name, pi.GetValue(obj), Environment.NewLine, setItem);
                                        allParameterWriting += string.Format(" m_table.get()->PutNumber(\"{0}_{1}\", {0}->Get{4}());{3}", mcd.name, pi.Name, pi.GetValue(obj), Environment.NewLine, setItem);
                                    }
                                }

                            }
                        }

                        resultString = resultString.Replace("$$_READ_TUNABLE_PARAMETERS_$$", allParameterReading);
                        resultString = resultString.Replace("$$_PUSH_TUNABLE_PARAMETERS_$$", allParameterWriting);

                        #endregion



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

                        resultString = resultString.Replace("$$_CREATE_FUNCTIONS_$$", createFunctionDeclarations);
                        resultString = resultString.Replace("$$_INITIALZATION_FUNCTIONS_$$", initializationFunctionDeclarations);

                        resultString = resultString.Replace("$$_MECHANISM_NAME_$$", mi.mechanism.name);
                        resultString = resultString.Replace("$$_MECHANISM_INSTANCE_NAME_$$", mi.name);
                        resultString = resultString.Replace("$$_MECHANISM_INSTANCE_NAME_UPPER_CASE_$$", ToUnderscoreCase(mi.name).ToUpper());

                        List<string> mechElementsGetters = generateMethod(mi.mechanism, "generateDefinitionGetter").FindAll(me => !me.StartsWith("state* "));
                        resultString = resultString.Replace("$$_MECHANISM_ELEMENTS_GETTERS_$$", ListToString(mechElementsGetters.Distinct().ToList()));

                        List<string> mechElements = generateMethod(mi.mechanism, "generateDefinition").FindAll(me => !me.StartsWith("state* "));
                        resultString = resultString.Replace("$$_MECHANISM_ELEMENTS_$$", ListToString(mechElements));
                        resultString = resultString.Replace("$$_INCLUDE_FILES_$$", ListToString(generateMethod(mi.mechanism, "generateIncludes").Distinct().ToList()));

                        //closed loop parameters
                        string allParameters = "";
                        resultString = resultString.Replace("$$_TUNABLE_PARAMETERS_$$", allParameters);

                        List<string> enumList = new List<string>();
                        foreach (state s in mi.mechanism.states)
                        {
                            enumList.Add(String.Format("STATE_{0}", ToUnderscoreCase(s.name).ToUpper()));
                        }

                        resultString = resultString.Replace("$$_STATE_NAMES_$$", ListToString(enumList, ", ").TrimEnd(new char[] { ',', ' ' }));

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

                            string stateName = "AllStates";

                            StringBuilder setTargetFunctionDeclerations = new StringBuilder();
                            if (generatorContext.singleStateGenFile)
                            {
                                #region Generate CPP StateGen Files
                                cdf = theToolConfiguration.getTemplateInfo("stateGen_cpp");
                                template = loadTemplate(cdf.templateFilePathName.Replace(".cpp", "_singleStateGenFile.cpp"));

                                resultString = template;

                                resultString = resultString.Replace("$$_MECHANISM_NAME_$$", mi.mechanism.name);
                                resultString = resultString.Replace("$$_MECHANISM_INSTANCE_NAME_$$", mi.name);
                                resultString = resultString.Replace("$$_STATE_NAME_$$", stateName);



                                StringBuilder setTargetFunctionCalls = new StringBuilder();
                                StringBuilder setTargetFunctionDefinitions = new StringBuilder();
                                foreach (applicationData r in theRobotConfiguration.theRobotVariants.Robots)
                                {
                                    mechanismInstance theMi = r.mechanismInstances.Find(m => m.mechanism.GUID == mi.mechanism.GUID);
                                    //if this mechanism exists in this robot
                                    if (theMi != null)
                                    {
                                        StringBuilder stateTargets = new StringBuilder();
                                        foreach (state s in theMi.mechanism.states)
                                        {
                                            List<string> motorTargets = new List<string>();
                                            foreach (motorTarget mT in s.motorTargets)
                                            {
                                                if (mT.Enabled.value)
                                                {
                                                    // find the corresponding motor control Data link
                                                    motorControlData mcd = theMi.mechanism.stateMotorControlData.Find(cd => cd.name == mT.controlDataName);
                                                    if (mcd == null)
                                                        addProgress(string.Format("In mechanism {0}, cannot find a Motor control data called {1}, referenced in state {2}", theMi.name, mT.controlDataName, s.name));
                                                    else
                                                    {
                                                        //void SetTargetControl(RobotElementNames::MOTOR_CONTROLLER_USAGE identifier, double percentOutput);
                                                        //void SetTargetControl(RobotElementNames::MOTOR_CONTROLLER_USAGE identifier, ControlData &controlConst, units::angle::degree_t angle );
                                                        //void SetTargetControl(RobotElementNames::MOTOR_CONTROLLER_USAGE identifier, ControlData &controlConst, units::angular_velocity::revolutions_per_minute_t angVel );
                                                        //void SetTargetControl(RobotElementNames::MOTOR_CONTROLLER_USAGE identifier, ControlData &controlConst, units::length::inch_t position );
                                                        //void SetTargetControl(RobotElementNames::MOTOR_CONTROLLER_USAGE identifier, ControlData &controlConst, units::velocity::feet_per_second_t velocity );

                                                        string targetUnitsType = "";
                                                        if (mcd.controlType == motorControlData.CONTROL_TYPE.PERCENT_OUTPUT) { }
                                                        else if (mcd.controlType == motorControlData.CONTROL_TYPE.POSITION_INCH) { targetUnitsType = "units::length::inch_t"; }
                                                        else if (mcd.controlType == motorControlData.CONTROL_TYPE.POSITION_ABS_TICKS) { addProgress("How should we handle POSITION_ABS_TICKS"); }
                                                        else if (mcd.controlType == motorControlData.CONTROL_TYPE.POSITION_DEGREES) { targetUnitsType = "units::angle::degree_t"; }
                                                        else if (mcd.controlType == motorControlData.CONTROL_TYPE.POSITION_DEGREES_ABSOLUTE) { targetUnitsType = "units::angle::degree_t"; }
                                                        else if (mcd.controlType == motorControlData.CONTROL_TYPE.VELOCITY_INCH) { targetUnitsType = "units::velocity::feet_per_second_t"; }
                                                        else if (mcd.controlType == motorControlData.CONTROL_TYPE.VELOCITY_DEGREES) { targetUnitsType = "units::angular_velocity::revolutions_per_minute_t"; }
                                                        else if (mcd.controlType == motorControlData.CONTROL_TYPE.VELOCITY_RPS) { targetUnitsType = "units::angular_velocity::revolutions_per_minute_t"; }
                                                        else if (mcd.controlType == motorControlData.CONTROL_TYPE.VOLTAGE) { targetUnitsType = "units::angular_velocity::revolutions_per_minute_t"; }
                                                        else if (mcd.controlType == motorControlData.CONTROL_TYPE.CURRENT) { addProgress("How should we handle CURRENT"); }
                                                        else if (mcd.controlType == motorControlData.CONTROL_TYPE.TRAPEZOID_LINEAR_POS) { targetUnitsType = "units::length::inch_t"; }
                                                        else if (mcd.controlType == motorControlData.CONTROL_TYPE.TRAPEZOID_ANGULAR_POS) { targetUnitsType = "units::angle::degree_t"; }

                                                        MotorController mc = theMi.mechanism.MotorControllers.Find(m => m.name == mT.motorName);
                                                        if (mc == null)
                                                        {
                                                            addProgress(string.Format("In mechanism {0}, cannot find a Motor controller called {1}, referenced in state {2}, target {3}", theMi.name, mT.motorName, s.name, mT.name));
                                                        }
                                                        else
                                                        {
                                                            string motorEnumName = String.Format("RobotElementNames::{0}", ListToString(mc.generateElementNames(), "").Trim().Replace("::", "_USAGE::").ToUpper());
                                                            if (targetUnitsType == "")
                                                            {
                                                                if (mc.GetType().IsSubclassOf(typeof(SparkController)))
                                                                {
                                                                    motorTargets.Add(String.Format("Get{0}()->get{1}()->SetControlConstants({2},*Get{0}()->get{3}())", theMi.name, mc.name, 0 /*slot number fixed to 0*/, mT.controlDataName));
                                                                }

                                                                motorTargets.Add(String.Format("SetTargetControl({0}, {1})", motorEnumName, mT.target.value));
                                                            }
                                                            else
                                                                motorTargets.Add(String.Format("SetTargetControl({0}, Get{1}()->get{2}(), {5}({3}({4})))",
                                                                    motorEnumName,
                                                                    theMi.name,
                                                                    mcd.name,
                                                                    generatorContext.theGeneratorConfig.getWPIphysicalUnitType(mT.target.physicalUnits),
                                                                    mT.target.value,
                                                                    targetUnitsType));
                                                        }
                                                    }
                                                }
                                            }

                                            stateTargets.AppendLine(string.Format("else if( Get{2}()->GetCurrentState() == {2}Gen::STATE_NAMES::STATE_{1})", ToUnderscoreCase(generatorContext.theMechanismInstance.name).ToUpper(), ToUnderscoreCase(s.name).ToUpper(), generatorContext.theMechanismInstance.name));
                                            stateTargets.AppendLine("{");
                                            stateTargets.AppendLine(ListToString(motorTargets, ";"));
                                            stateTargets.AppendLine("}");
                                        }

                                        setTargetFunctionCalls.AppendLine(string.Format("else if(m_RobotId == {0})",r.robotID));
                                        setTargetFunctionCalls.AppendLine(String.Format(" Init{0}();", r.getFullRobotName()));

                                        setTargetFunctionDeclerations.AppendLine(String.Format("void Init{0}();", r.getFullRobotName()));

                                        setTargetFunctionDefinitions.AppendLine(String.Format("void {0}AllStatesStateGen::Init{1}()", theMi.name, r.getFullRobotName()));
                                        setTargetFunctionDefinitions.AppendLine("{");
                                        setTargetFunctionDefinitions.AppendLine(stateTargets.ToString().Trim().Substring(5));
                                        setTargetFunctionDefinitions.AppendLine("}");
                                        setTargetFunctionDefinitions.AppendLine();
                                    }
                                }

                                resultString = resultString.Replace("$$_SET_TARGET_CONTROL_FUNCTIONS_$$", setTargetFunctionDefinitions.ToString().Trim());
                                resultString = resultString.Replace("$$_SET_TARGET_CONTROL_$$", setTargetFunctionCalls.ToString().Trim().Substring(5));

                                filePathName = getMechanismFullFilePathName(mechanismName,
                                                                            cdf.outputFilePathName.Replace("MECHANISM_INSTANCE_NAME", mechanismName).Replace("STATE_NAME", stateName)
                                                                            , true);
                                copyrightAndGenNoticeAndSave(filePathName, resultString);
                                #endregion
                            }
                            else
                            {
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
                                            if (mcd.controlType == motorControlData.CONTROL_TYPE.PERCENT_OUTPUT) { }
                                            else if (mcd.controlType == motorControlData.CONTROL_TYPE.POSITION_INCH) { targetUnitsType = "units::length::inch_t"; }
                                            else if (mcd.controlType == motorControlData.CONTROL_TYPE.POSITION_ABS_TICKS) { addProgress("How should we handle POSITION_ABS_TICKS"); }
                                            else if (mcd.controlType == motorControlData.CONTROL_TYPE.POSITION_DEGREES) { targetUnitsType = "units::angle::degree_t"; }
                                            else if (mcd.controlType == motorControlData.CONTROL_TYPE.POSITION_DEGREES_ABSOLUTE) { targetUnitsType = "units::angle::degree_t"; }
                                            else if (mcd.controlType == motorControlData.CONTROL_TYPE.VELOCITY_INCH) { targetUnitsType = "units::velocity::feet_per_second_t"; }
                                            else if (mcd.controlType == motorControlData.CONTROL_TYPE.VELOCITY_DEGREES) { targetUnitsType = "units::angular_velocity::revolutions_per_minute_t"; }
                                            else if (mcd.controlType == motorControlData.CONTROL_TYPE.VELOCITY_RPS) { targetUnitsType = "units::angular_velocity::revolutions_per_minute_t"; }
                                            else if (mcd.controlType == motorControlData.CONTROL_TYPE.VOLTAGE) { targetUnitsType = "units::angular_velocity::revolutions_per_minute_t"; }
                                            else if (mcd.controlType == motorControlData.CONTROL_TYPE.CURRENT) { addProgress("How should we handle CURRENT"); }
                                            else if (mcd.controlType == motorControlData.CONTROL_TYPE.TRAPEZOID_LINEAR_POS) { targetUnitsType = "units::length::inch_t"; }
                                            else if (mcd.controlType == motorControlData.CONTROL_TYPE.TRAPEZOID_ANGULAR_POS) { targetUnitsType = "units::angle::degree_t"; }

                                            MotorController mc = mi.mechanism.MotorControllers.Find(m => m.name == mT.motorName);
                                            if (mc == null)
                                            {
                                                addProgress(string.Format("In mechanism {0}, cannot find a Motor controller called {1}, referenced in state {2}, target {3}", mi.name, mT.motorName, s.name, mT.name));
                                            }
                                            else
                                            {
                                                string motorEnumName = String.Format("RobotElementNames::{0}", ListToString(mc.generateElementNames(), "").Trim().Replace("::", "_USAGE::").ToUpper());
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
                            }

                            if (generatorContext.singleStateGenFile)
                            {
                                #region Generate H StateGen Files
                                cdf = theToolConfiguration.getTemplateInfo("stateGen_h");
                                template = loadTemplate(cdf.templateFilePathName.Replace(".h", "_singleStateGenFile.h"));

                                resultString = template;
                                //

                                resultString = resultString.Replace("$$_SET_TARGET_CONTROL_FUNCTION_DECLS_$$", setTargetFunctionDeclerations.ToString());
                                resultString = resultString.Replace("$$_MECHANISM_NAME_$$", mi.mechanism.name);
                                resultString = resultString.Replace("$$_MECHANISM_INSTANCE_NAME_$$", mi.name);
                                resultString = resultString.Replace("$$_STATE_NAME_$$", stateName);

                                resultString = resultString.Replace("$$_TARGET_DECLARATIONS_$$", "");

                                filePathName = getMechanismFullFilePathName(mechanismName,
                                                                            cdf.outputFilePathName.Replace("MECHANISM_INSTANCE_NAME", mechanismName).Replace("STATE_NAME", stateName)
                                                                            , true);
                                copyrightAndGenNoticeAndSave(filePathName, resultString);
                                #endregion
                            }
                            else
                            {
                                #region Generate H StateGen Files
                                foreach (state s in mi.mechanism.states)
                                {
                                    cdf = theToolConfiguration.getTemplateInfo("stateGen_h");
                                    template = loadTemplate(cdf.templateFilePathName);

                                    resultString = template;

                                    resultString = resultString.Replace("$$_MECHANISM_NAME_$$", mi.mechanism.name);
                                    resultString = resultString.Replace("$$_MECHANISM_INSTANCE_NAME_$$", mi.name);
                                    resultString = resultString.Replace("$$_STATE_NAME_$$", s.name);

                                    resultString = resultString.Replace("$$_TARGET_DECLARATIONS_$$", "");

                                    filePathName = getMechanismFullFilePathName(mechanismName,
                                                                                cdf.outputFilePathName.Replace("MECHANISM_INSTANCE_NAME", mechanismName).Replace("STATE_NAME", s.name)
                                                                                , true);
                                    copyrightAndGenNoticeAndSave(filePathName, resultString);
                                }
                                #endregion
                            }


                            #endregion

                            generatorContext.generationStage = generatorContext.GenerationStage.MechInstanceDecorator;

                            #region The decorator mod files
                            createMechanismFolder(mechanismName, false);

                            #region Generate Cpp File
                            cdf = theToolConfiguration.getTemplateInfo("MechanismInstance_cpp");
                            template = loadTemplate(cdf.templateFilePathName);

                            resultString = template;

                            resultString = resultString.Replace("$$_MECHANISM_INSTANCE_NAME_$$", mi.name);

                            List<string> includeList = generateMethod(mi, "generateIncludes").Distinct().ToList();
                            resultString = resultString.Replace("$$_STATE_CLASSES_INCLUDES_$$", ListToString(includeList, ""));

                            filePathName = getMechanismFullFilePathName(mechanismName, cdf.outputFilePathName.Replace("MECHANISM_INSTANCE_NAME", mechanismName), false);
                            copyrightAndGenNoticeAndSave(filePathName, resultString, true);
                            #endregion

                            #region Generate fixed decorator Cpp File
                            cdf = theToolConfiguration.getTemplateInfo("MechanismInstance_decorated_cpp");
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
                                        stateTransitions.Add(String.Format("{0}StateInst->RegisterTransitionState({1}StateInst)", s.name, transition.value));
                                    }
                                }
                                else
                                {
                                    stateTransitions.Add(String.Format("{0}StateInst->RegisterTransitionState({0}StateInst)", s.name));
                                }
                            }
                            resultString = resultString.Replace("$$_OBJECT_CREATION_$$", ListToString(statesCreation, ";"));
                            resultString = resultString.Replace("$$_STATE_TRANSITION_REGISTRATION_$$", ListToString(stateTransitions, ";"));

                            includeList = generateMethod(mi, "generateIncludes").Distinct().ToList();
                            resultString = resultString.Replace("$$_STATE_CLASSES_INCLUDES_$$", ListToString(includeList, ""));

                            filePathName = getMechanismFullFilePathName(mechanismName, cdf.outputFilePathName.Replace("MECHANISM_INSTANCE_NAME", mechanismName), true);
                            copyrightAndGenNoticeAndSave(filePathName, resultString, false);
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
                                if (generatorContext.singleStateGenFile)
                                    template = loadTemplate(cdf.templateFilePathName.Replace(".h", "_singleStateGenFile.h"));
                                else
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
                                if (generatorContext.singleStateGenFile)
                                    template = loadTemplate(cdf.templateFilePathName.Replace(".cpp", "_singleStateGenFile.cpp"));
                                else
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

        private string GenerateCreateFunctions(mechanismInstance mi, out string functionDeclarations)
        {
            string createFunctionTemplate =
                            @"void $$_MECHANISM_INSTANCE_NAME_$$Gen::Create$$_ROBOT_ID_$$()
                                {
                                    m_ntName = ""$$_MECHANISM_INSTANCE_NAME_$$"";
                                    $$_OBJECT_CREATION_$$

                                    m_table = nt::NetworkTableInstance::GetDefault().GetTable(m_ntName);
                                    m_tuningIsEnabledStr = ""Enable Tuning for "" + m_ntName; // since this string is used every loop, we do not want to create the string every time
                                    m_table.get()->PutBoolean(m_tuningIsEnabledStr, m_tuning);
                                }";

            string createFunctionDeclarationTemplate = "void Create$$_ROBOT_ID_$$()";

            List<string> createCode = new List<string>();
            List<string> createDeclarationCode = new List<string>();
            foreach (applicationData r in theRobotConfiguration.theRobotVariants.Robots)
            {
                mechanismInstance mis = r.mechanismInstances.Find(m => m.name == mi.name);
                if (mis != null)
                {
                    string temp = createFunctionTemplate;
                    temp = temp.Replace("$$_OBJECT_CREATION_$$", ListToString(generateMethod(mis, "generateIndexedObjectCreation")));
                    temp = temp.Replace("$$_ROBOT_ID_$$", r.getFullRobotName());
                    createCode.Add(temp);

                    string tempDecl = createFunctionDeclarationTemplate;
                    createDeclarationCode.Add(tempDecl.Replace("$$_ROBOT_ID_$$", r.getFullRobotName()));
                }
            }

            functionDeclarations = ListToString(createDeclarationCode, ";");

            return ListToString(createCode, Environment.NewLine);
        }

        private string GenerateInitializationFunctions(mechanismInstance mi, out string functionDeclarations)
        {
            string createFunctionTemplate =
                            @"void $$_MECHANISM_INSTANCE_NAME_$$Gen::Initialize$$_ROBOT_ID_$$()
                                {
                                    $$_ELEMENT_INITIALIZATION_$$
                                }";

            string createFunctionDeclarationTemplate = "void Initialize$$_ROBOT_ID_$$()";

            List<string> createCode = new List<string>();
            List<string> initDeclarationCode = new List<string>();
            foreach (applicationData r in theRobotConfiguration.theRobotVariants.Robots)
            {
                mechanismInstance mis = r.mechanismInstances.Find(m => m.name == mi.name);
                if (mis != null)
                {
                    string temp = createFunctionTemplate;
                    temp = temp.Replace("$$_ELEMENT_INITIALIZATION_$$", ListToString(generateMethod(mis, "generateInitialization")));
                    temp = temp.Replace("$$_ROBOT_ID_$$", r.getFullRobotName());
                    createCode.Add(temp);

                    string tempDecl = createFunctionDeclarationTemplate;
                    initDeclarationCode.Add(tempDecl.Replace("$$_ROBOT_ID_$$", r.getFullRobotName()));
                }
            }

            functionDeclarations = ListToString(initDeclarationCode, ";");

            return ListToString(createCode, Environment.NewLine);
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
