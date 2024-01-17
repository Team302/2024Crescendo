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
    internal class RobotConfigManagerGenerator : baseGenerator
    {
        internal RobotConfigManagerGenerator(string codeGeneratorVersion, applicationDataConfig theRobotConfiguration, toolConfiguration theToolConfiguration, bool cleanMode, showMessage displayProgress)
        : base(codeGeneratorVersion, theRobotConfiguration, theToolConfiguration, cleanMode)
        {
            setProgressCallback(displayProgress);
        }



        internal void generate()
        {
            codeTemplateFile cdf;
            string template;

            addProgress((cleanMode ? "Erasing" : "Writing") + " Robot Configuration Manager files...");

            #region Generate CPP File
            cdf = theToolConfiguration.getTemplateInfo("RobotConfigMgr_cpp");
            template = loadTemplate(cdf.templateFilePathName);

            generatorContext.clear();
            StringBuilder sb = new StringBuilder();
            StringBuilder includes = new StringBuilder();
            foreach (applicationData robot in theRobotConfiguration.theRobotVariants.Robots)
            {
                generatorContext.theRobot = robot;
                sb.AppendLine(string.Format(@"case RobotIdentifier::{0}:
                                                Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string(""Initializing robot ""), string(""{0}""), string(""""));
                                                m_config = new RobotConfig{0}();
                                                break;", robot.getFullRobotName()));
                includes.AppendLine(string.Format("#include \"configs/RobotConfig{0}.h\"", robot.getFullRobotName()));
            }
            template = template.Replace("$$_ROBOT_CONFIGURATION_CREATION_$$", sb.ToString());
            template = template.Replace("$$_ROBOT_CONFIG_INCLUDES_$$", includes.ToString());

            copyrightAndGenNoticeAndSave(getOutputFileFullPath(cdf.outputFilePathName), template);
            #endregion

            #region Generate H File
            cdf = theToolConfiguration.getTemplateInfo("RobotConfigMgr_h");
            template = loadTemplate(cdf.templateFilePathName);

            generatorContext.clear();
            sb = new StringBuilder();
            foreach (applicationData robot in theRobotConfiguration.theRobotVariants.Robots)
            {
                generatorContext.theRobot = robot;
                sb.AppendLine(string.Format("{0} = {1},", robot.getFullRobotName(), robot.robotID.value));
            }
            template = template.Replace("$$_ROBOT_CONFIGURATIONS_NAMES_ENUMS_$$", sb.ToString());

            copyrightAndGenNoticeAndSave(getOutputFileFullPath(cdf.outputFilePathName), template);
            #endregion
        }
    }
}
