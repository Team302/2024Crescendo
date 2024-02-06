using applicationConfiguration;
using ApplicationData;
using Configuration;
using DataConfiguration;
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Reflection;
using System.Text;
using System.Threading.Tasks;
using System.Xml.Linq;

namespace CoreCodeGenerator
{
    internal class MiscellaneousGenerator : baseGenerator
    {
        internal MiscellaneousGenerator(string codeGeneratorVersion, applicationDataConfig theRobotConfiguration, toolConfiguration theToolConfiguration, bool cleanMode, showMessage displayProgress)
                : base(codeGeneratorVersion, theRobotConfiguration, theToolConfiguration, cleanMode)
        {
            setProgressCallback(displayProgress);
        }
        internal void generate()
        {
            addProgress((cleanMode ? "Erasing" : "Writing") + " general files...");
            generate_RobotElementNames();
            generate_MechanismNames();
        }

        internal void generate_RobotElementNames()
        {
            addProgress("Writing RobotElementNames...");
            codeTemplateFile cdf = theToolConfiguration.getTemplateInfo("RobotElementNames");
            string template = loadTemplate(cdf.templateFilePathName);

            List<string> names = new List<string>();

            generatorContext.clear();
            foreach (applicationData robot in theRobotConfiguration.theRobotVariants.Robots)
            {
                List<mechanismInstance> instances = new List<mechanismInstance>();
                instances.AddRange(robot.mechanismInstances);
                instances.AddRange(robot.Chassis.mechanismInstances);

                foreach (mechanismInstance mechInst in instances)
                {
                    generatorContext.theRobot = robot;
                    generatorContext.theMechanismInstance = mechInst;
                    generatorContext.theMechanism = mechInst.mechanism;
                    names.AddRange(generateMethod(mechInst, "generateElementNames"));
                }
            }

            generatorContext.clear();
            foreach (applicationData robot in theRobotConfiguration.theRobotVariants.Robots)
            {
                generatorContext.theRobot = robot;
                names.AddRange(generateMethod(robot, "generateElementNames").Distinct().ToList());
            }
            names = names.Distinct().ToList();

            List<Type> theTypeList = Assembly.GetAssembly(typeof(baseRobotElementClass)).GetTypes()
                  .Where(t => (t.BaseType == typeof(baseRobotElementClass))).ToList();

            //todo remove once chassis is auto generated
            names.Add("pigeon::PIGEON_ROBOT_CENTER");

            StringBuilder sb = new StringBuilder();
            foreach (Type type in theTypeList)
            {
                string enumName = ToUnderscoreCase(type.Name);

                if (type != typeof(state))
                {
                    sb.AppendLine(string.Format("enum {0}_USAGE", enumName.ToUpper()));
                    sb.AppendLine("{");
                    sb.AppendLine(string.Format("UNKNOWN_{0} = -1,", enumName.ToUpper()));

                    string startingChars = enumName + "::";
                    foreach (string s in names.Where(t => t.StartsWith(startingChars)))
                    {
                        sb.AppendLine(string.Format("{0},", s.Substring(startingChars.Length).ToUpper()));
                    }
                    sb.AppendLine(string.Format("MAX_{0}", enumName.ToUpper()));
                    sb.AppendLine("};");
                    sb.AppendLine();
                }
                else if (false) // disable the enum creation for states
                {
                    string startingChars = enumName + "_";
                    List<string> thestates = names.Where(t => t.StartsWith(startingChars)).ToList();
                    for (int i = 0; i < thestates.Count; i++)
                    {
                        int index = thestates[i].IndexOf("::");
                        if (index > 0)
                        {
                            thestates[i] = thestates[i].Substring(0,index);
                        }
                    }

                    List<string> mechStateEnumNames = thestates.Distinct().ToList();
                    foreach(string stateEnumName in mechStateEnumNames)
                    {
                        sb.AppendLine(string.Format("enum {0}_USAGE", stateEnumName.ToUpper()));
                        sb.AppendLine("{");
                        sb.AppendLine(string.Format("UNKNOWN_{0} = -1,", stateEnumName.ToUpper()));

                        startingChars = stateEnumName + "::";
                        foreach (string s in names.Where(t => t.StartsWith(startingChars)))
                        {
                            sb.AppendLine(string.Format("{0},", s.Substring(startingChars.Length).ToUpper()));
                        }

                        sb.AppendLine(string.Format("MAX_{0}", stateEnumName.ToUpper()));
                        sb.AppendLine("};");
                        sb.AppendLine();
                    }
                }

            }

            template = template.Replace("$$_ROBOT_ELEMENT_NAMES_ENUMS_$$", sb.ToString().Trim());

            copyrightAndGenNoticeAndSave(getOutputFileFullPath(cdf.outputFilePathName), template);
        }

        internal void generate_MechanismNames()
        {
            addProgress("Writing MechanismNames...");
            codeTemplateFile cdf = theToolConfiguration.getTemplateInfo("MechanismTypes");
            string template = loadTemplate(cdf.templateFilePathName);

            List<string> mechNames = new List<string>();
            foreach (applicationData robot in theRobotConfiguration.theRobotVariants.Robots)
            {
                List<mechanismInstance> instances = new List<mechanismInstance>();
                instances.AddRange(robot.mechanismInstances);
                instances.AddRange(robot.Chassis.mechanismInstances);

                foreach (mechanismInstance mi in instances)
                {
                    mechNames.Add(string.Format("{0}", getMechanismInstanceName(mi)));
                }
            }

            template = template.Replace("$$_MECHANISM_NAMES_ENUMS_$$", ListToString(mechNames.Distinct().ToList(), ",").ToUpper());

            copyrightAndGenNoticeAndSave(getOutputFileFullPath(cdf.outputFilePathName), template);
        }

        internal string getMechanismElementName(MotorController mc)
        {
            return string.Format("{0}", ToUnderscoreCase(mc.name));
        }

        internal string getMechanismInstanceName(mechanismInstance mi)
        {
            return string.Format("{0}", ToUnderscoreCase(mi.name));
        }
    }
}
