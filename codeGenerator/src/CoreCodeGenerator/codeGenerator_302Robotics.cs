using Configuration;
using DataConfiguration;
using ApplicationData;
using applicationConfiguration;
using System.Collections.Generic;
using System.IO;
using System.Text;
using System.Linq;
using System;

namespace CoreCodeGenerator
{
    public class codeGenerator_302Robotics : baseReportingClass
    {
        string codeGeneratorVersion = "";
        public bool cleanDecoratorModFolders { get; set; } = false;

        public enum MECHANISM_FILE_TYPE { MAIN, STATE, STATE_MGR }

        private applicationDataConfig theRobotConfiguration = new applicationDataConfig();
        private toolConfiguration theToolConfiguration = new toolConfiguration();

        public void generate(string codeGenVersion, applicationDataConfig theRobotConfig, toolConfiguration generatorConfig)
        {
            generate(codeGenVersion, theRobotConfig, generatorConfig, false);
        }
        public void clean(string codeGenVersion, applicationDataConfig theRobotConfig, toolConfiguration generatorConfig)
        {
            generate(codeGenVersion, theRobotConfig, generatorConfig, true);
        }
        private void generate(string codeGenVersion, applicationDataConfig theRobotConfig, toolConfiguration generatorConfig, bool cleanMode)
        {
            codeGeneratorVersion = codeGenVersion;
            theRobotConfiguration = theRobotConfig;
            theToolConfiguration = generatorConfig;

            string rootFolder = generatorConfig.rootOutputFolder;

            addProgress("Output will be placed at " + rootFolder);

            if (!Directory.Exists(rootFolder))
            {
                Directory.CreateDirectory(rootFolder);
                addProgress("Created output directory " + rootFolder);
            }
            else
            {
                addProgress("Output directory " + rootFolder + " already exists and therefore was not created.");
            }

            new MiscellaneousGenerator(codeGenVersion, theRobotConfig, generatorConfig, cleanMode, addProgress).generate();
            new MechanismInstanceGenerator(codeGenVersion, theRobotConfig, generatorConfig, cleanMode, cleanDecoratorModFolders, addProgress).generate();
            new RobotConfigManagerGenerator(codeGenVersion, theRobotConfig, generatorConfig, cleanMode, addProgress).generate();
            new RobotConfigRobotSpecificGenerator(codeGenVersion, theRobotConfig, generatorConfig, cleanMode, addProgress).generate();
            //new ChassisGenerator(codeGenVersion, theRobotConfig, generatorConfig, cleanMode, addProgress).generate();

        }
    }
}
