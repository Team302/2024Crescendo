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

                foreach (camera cam in robot.Cameras)
                {
                    sb.AppendLine(ListToString(cam.generateDefinition()));
                    includes.AppendLine(ListToString(cam.generateIncludes(),";"));
                }
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
                  $$_MECHANISM_INSTANCE_NAME_$$Gen* $$_MECHANISM_INSTANCE_NAME_$$GenMech = new $$_MECHANISM_INSTANCE_NAME_$$Gen();
                  m_the$$_MECHANISM_INSTANCE_NAME_$$ = new $$_MECHANISM_INSTANCE_NAME_$$($$_MECHANISM_INSTANCE_NAME_$$GenMech);
                  m_the$$_MECHANISM_INSTANCE_NAME_$$->Create();
                  m_the$$_MECHANISM_INSTANCE_NAME_$$->Initialize(RobotConfigMgr::RobotIdentifier::$$_ROBOT_ENUM_NAME_$$);
                  m_the$$_MECHANISM_INSTANCE_NAME_$$->CreateAndRegisterStates();
                  ";

            string mechInstDefState =
                @"m_the$$_MECHANISM_INSTANCE_NAME_$$->Init(m_the$$_MECHANISM_INSTANCE_NAME_$$);
                  ";

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

                    sb.AppendLine(resultString);
                }

                resultString = template.Replace("$$_MECHANISMS_INITIALIZATION_$$", sb.ToString().Trim());
                resultString = resultString.Replace("$$_ROBOT_NAME_$$", ToUnderscoreDigit(robot.getFullRobotName()));
                resultString = resultString.Replace("$$_ROBOT_ENUM_NAME_$$", ToUnderscoreDigit(ToUnderscoreCase(robot.getFullRobotName())).ToUpper());

                copyrightAndGenNoticeAndSave(getOutputFileFullPath(cdf.outputFilePathName).Replace("$$_ROBOT_NAME_$$", ToUnderscoreDigit(robot.getFullRobotName())), resultString);
            }
            #endregion
        }
    }
}
