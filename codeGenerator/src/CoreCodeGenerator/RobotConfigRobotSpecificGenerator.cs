using applicationConfiguration;
using ApplicationData;
using Configuration;
using DataConfiguration;
using System;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Reflection;
using System.Text;
using System.Threading.Tasks;

namespace CoreCodeGenerator
{
    internal class RobotConfigRobotSpecificGenerator : baseGenerator
    {
        internal RobotConfigRobotSpecificGenerator(string codeGeneratorVersion, applicationDataConfig theRobotConfiguration, toolConfiguration theToolConfiguration, bool cleanMode, showMessage displayProgress)
        : base(codeGeneratorVersion, theRobotConfiguration, theToolConfiguration, cleanMode)
        {
            setProgressCallback(displayProgress);
        }



        internal void generate()
        {
            string resultString;
            codeTemplateFile cdf;
            string template;

            addProgress((cleanMode ? "Erasing" : "Writing") + " Robot specific configuration files...");

            #region Generate H File
            cdf = theToolConfiguration.getTemplateInfo("RobotConfigRobotSpecific_h");
            template = loadTemplate(cdf.templateFilePathName);

            generatorContext.clear();
            StringBuilder sb = new StringBuilder();
            foreach (applicationData robot in theRobotConfiguration.theRobotVariants.Robots)
            {
                generatorContext.theRobot = robot;
                resultString = template.Replace("$$_ROBOT_NAME_$$", ToUnderscoreDigit(robot.getFullRobotName()));

                StringBuilder includes = new StringBuilder();
                sb.Clear();

                foreach (mechanismInstance mi in robot.mechanismInstances)
                {
                    sb.AppendLine(string.Format("{0}* m_the{0} = nullptr;", mi.name));
                    includes.AppendLine(String.Format("#include \"{0}\"", mi.getIncludePath()));
                }
                resultString = resultString.Replace("$$_MECHANISM_PTR_DECLARATIONS_$$", sb.ToString().Trim());

                sb.Clear();
                List<string> incs = new List<string>();
                foreach (Camera cam in robot.Cameras)
                {
                    sb.AppendLine(ListToString(cam.generateDefinition()));
                    incs.AddRange(cam.generateIncludes());
                }

                includes.AppendLine(ListToString(incs.Distinct().ToList(), ""));

                resultString = resultString.Replace("$$_CAMERA_PTR_DECLARATIONS_$$", sb.ToString().Trim());

                resultString = resultString.Replace("$$_MECHANISM_INCLUDE_FILES_$$", includes.ToString().Trim());

                copyrightAndGenNoticeAndSave(getOutputFileFullPath(cdf.outputFilePathName).Replace("$$_ROBOT_NAME_$$", ToUnderscoreDigit(robot.getFullRobotName())), resultString);
            }

            #endregion

            #region Generate CPP File
            cdf = theToolConfiguration.getTemplateInfo("RobotConfigRobotSpecific_cpp");
            template = loadTemplate(cdf.templateFilePathName);

            string mechInstDef =
                @"Logger::GetLogger()->LogData ( LOGGER_LEVEL::PRINT, string ( ""Initializing mechanism"" ), string ( ""$$_MECHANISM_INSTANCE_NAME_$$"" ), """" );
                  $$_MECHANISM_INSTANCE_NAME_$$Gen* $$_MECHANISM_INSTANCE_NAME_$$GenMech = new $$_MECHANISM_INSTANCE_NAME_$$Gen(RobotConfigMgr::RobotIdentifier::$$_ROBOT_ENUM_NAME_$$);
                  m_the$$_MECHANISM_INSTANCE_NAME_$$ = new $$_MECHANISM_INSTANCE_NAME_$$($$_MECHANISM_INSTANCE_NAME_$$GenMech, RobotConfigMgr::RobotIdentifier::$$_ROBOT_ENUM_NAME_$$);
                  m_the$$_MECHANISM_INSTANCE_NAME_$$->Create$$_ROBOT_FULL_NAME_$$();
                  m_the$$_MECHANISM_INSTANCE_NAME_$$->Initialize$$_ROBOT_FULL_NAME_$$();
                  m_the$$_MECHANISM_INSTANCE_NAME_$$->CreateAndRegisterStates();
                  ";

            string mechInstDefState =
                @"m_the$$_MECHANISM_INSTANCE_NAME_$$->Init(m_the$$_MECHANISM_INSTANCE_NAME_$$);
                  m_mechanismMap[MechanismTypes::MECHANISM_TYPE::$$_MECHANISM_INSTANCE_NAME_UPPERCASE_$$] = m_the$$_MECHANISM_INSTANCE_NAME_$$;
                 ";

            string LEDinitializationTemplate = @"DragonLeds::GetInstance()->Initialize($$_LED_PWM_ID_$$, $$_TOTAL_LED_$$);";

            generatorContext.clear();
            foreach (applicationData robot in theRobotConfiguration.theRobotVariants.Robots)
            {
                sb.Clear();
                resultString = "";
                generatorContext.theRobot = robot;

                foreach (mechanismInstance mi in robot.mechanismInstances)
                {
                    resultString = mechInstDef.Replace("$$_MECHANISM_INSTANCE_NAME_$$", mi.name);
                    resultString += mechInstDefState.Replace("$$_MECHANISM_INSTANCE_NAME_$$", mi.name);
                    resultString = resultString.Replace("$$_MECHANISM_INSTANCE_NAME_UPPERCASE_$$", ToUnderscoreCase(mi.name).ToUpper());

                    sb.AppendLine(resultString);
                }

                resultString = template.Replace("$$_MECHANISMS_INITIALIZATION_$$", sb.ToString().Trim());
                resultString = resultString.Replace("$$_ROBOT_FULL_NAME_$$", robot.getFullRobotName());
                resultString = resultString.Replace("$$_ROBOT_NAME_$$", ToUnderscoreDigit(robot.getFullRobotName()));
                resultString = resultString.Replace("$$_ROBOT_ENUM_NAME_$$", ToUnderscoreDigit(ToUnderscoreCase(robot.getFullRobotName())).ToUpper());

                sb.Clear();
                List<string> list = new List<string>();
                List<string> includeList = new List<string>();
                foreach (Camera cam in robot.Cameras)
                {
                    list.AddRange(cam.generateIndexedObjectCreation(0));
                    includeList.AddRange(cam.generateIncludes());
                }
                resultString = resultString.Replace("$$_CAMERAS_INITIALIZATION_$$", ListToString(list).Trim());

                sb.Clear();
                uint numberOfLeds = 0;
                foreach (LedSegment ls in robot.LEDs.Segments)
                    numberOfLeds += ls.Count.value;

                string LEDinitialization = "";
                if (numberOfLeds > 0)
                {
                    LEDinitialization = LEDinitializationTemplate;
                    LEDinitialization = LEDinitialization.Replace("$$_TOTAL_LED_$$", numberOfLeds.ToString());
                    LEDinitialization = LEDinitialization.Replace("$$_LED_PWM_ID_$$", robot.LEDs.PwmId.ToString());
                    includeList.AddRange(robot.LEDs.generateIncludes());
                }

                resultString = resultString.Replace("$$_LED_INITIALIZATION_$$", LEDinitialization);

                includeList = includeList.Distinct().ToList();
                resultString = resultString.Replace("$$_INCLUDE_$$", ListToString(includeList));

                copyrightAndGenNoticeAndSave(getOutputFileFullPath(cdf.outputFilePathName).Replace("$$_ROBOT_NAME_$$", ToUnderscoreDigit(robot.getFullRobotName())), resultString);
            }
            #endregion
        }
    }
}
