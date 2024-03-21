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
    internal class MechanismGenerator : baseGenerator
    {
        internal MechanismGenerator(string codeGeneratorVersion, applicationDataConfig theRobotConfiguration, toolConfiguration theToolConfiguration, bool cleanMode, showMessage displayProgress)
        : base(codeGeneratorVersion, theRobotConfiguration, theToolConfiguration, cleanMode)
        {
            setProgressCallback(displayProgress);
        }



        internal void generate()
        {
            addProgress((cleanMode ? "Erasing" : "Writing") + " mechanism files...");
            List<string> mechMainFiles = new List<string>();
            //List<string> mechStateFiles = new List<string>();
            //List<string> mechStateMgrFiles = new List<string>();
            foreach (mechanism mech in theRobotConfiguration.theRobotVariants.Mechanisms)
            {
                generatorContext.theMechanism = mech;

                string filePathName;
                string resultString;
                codeTemplateFile cdf;
                string template;

                string mechanismName = mech.name;

                createMechanismFolder(mechanismName);

                #region Generate Cpp File
                cdf = theToolConfiguration.getTemplateInfo("MechanismDefinition_cpp");
                template = loadTemplate(cdf.templateFilePathName);

                resultString = template;

                resultString = resultString.Replace("$$_INCLUDE_PATH_$$", getIncludePath(mechanismName));
                resultString = resultString.Replace("$$_MECHANISM_NAME_$$", mechanismName);
                resultString = resultString.Replace("$$_OBJECT_CREATION_$$", ListToString(generateMethod(mech, "generateObjectCreation")));
                resultString = resultString.Replace("$$_ELEMENT_INITIALIZATION_$$", ListToString(generateMethod(mech, "generateInitialization")));


                #region Tunable Parameters
                string allParameterReading = "";
#if david
                foreach (closedLoopControlParameters cLCParams in mech.closedLoopControlParameters)
                {
                    Type objType = cLCParams.GetType();

                    PropertyInfo[] propertyInfos = objType.GetProperties();

                    foreach (PropertyInfo pi in propertyInfos)
                    {
                        bool skip = (pi.Name == "name");
                        if (!skip)
                            allParameterReading += string.Format("{0}_{1} = m_table.get()->GetNumber(\"{0}_{1}\", {2});{3}", cLCParams.name, pi.Name, pi.GetValue(cLCParams), Environment.NewLine);
                    }

                }
#endif
                resultString = resultString.Replace("$$_READ_TUNABLE_PARAMETERS_$$", allParameterReading);

                string allParameterWriting = "";
#if david
                foreach (closedLoopControlParameters cLCParams in mech.closedLoopControlParameters)
                {
                    Type objType = cLCParams.GetType();

                    PropertyInfo[] propertyInfos = objType.GetProperties();

                    foreach (PropertyInfo pi in propertyInfos)
                    {
                        bool skip = (pi.Name == "name");
                        if (!skip)
                            allParameterWriting += string.Format("{0}_{1} = m_table.get()->PutNumber(\"{0}_{1}\", {0}_{1});{2}", cLCParams.name, pi.Name, Environment.NewLine);
                    }

                }
#endif
                resultString = resultString.Replace("$$_PUSH_TUNABLE_PARAMETERS_$$", allParameterWriting);

                #endregion

                filePathName = getMechanismFullFilePathName(mechanismName, cdf.outputFilePathName.Replace("MECHANISM_NAME", mechanismName));
                copyrightAndGenNoticeAndSave(filePathName, resultString);
                #endregion


                #region Generate H File
                cdf = theToolConfiguration.getTemplateInfo("MechanismDefinition_h");
                template = loadTemplate(cdf.templateFilePathName);

                resultString = template;

                resultString = resultString.Replace("$$_MECHANISM_NAME_$$", mechanismName);
                resultString = resultString.Replace("$$_MECHANISM_ELEMENTS_$$", ListToString(generateMethod(mech, "generateDefinition")).Replace("state*", "//state*"));
                resultString = resultString.Replace("$$_INCLUDE_FILES_$$", ListToString(generateMethod(mech, "generateIncludes").Distinct().ToList()));

                //closed loop parameters
                string allParameters = "";
                resultString = resultString.Replace("$$_TUNABLE_PARAMETERS_$$", allParameters);

                filePathName = getMechanismFullFilePathName(mechanismName, cdf.outputFilePathName.Replace("MECHANISM_NAME", mechanismName));
                copyrightAndGenNoticeAndSave(filePathName, resultString);
                #endregion

                if (cleanMode)
                {
                    Directory.Delete(getMechanismOutputPath(mechanismName), true);
                    Directory.Delete(Path.Combine(getMechanismOutputPath(mechanismName), ".."), true);
                }
            }
        }

        internal string getIncludePath(string mechanismName)
        {
            return getMechanismOutputPath(mechanismName).Replace(theToolConfiguration.rootOutputFolder, "").Replace(@"\", "/").TrimStart('/');
        }

        internal void createMechanismFolder(string mechanismName)
        {
            Directory.CreateDirectory(getMechanismOutputPath(mechanismName));
        }

        internal string getMechanismFullFilePathName(string mechanismName, string templateFilePath)
        {
            string filename = Path.GetFileName(templateFilePath);

            filename = filename.Replace("MECHANISM_NAME", mechanismName);

            return Path.Combine(getMechanismOutputPath(mechanismName), filename);
        }

        internal string getMechanismOutputPath(string mechanismName)
        {
            return Path.Combine(theToolConfiguration.rootOutputFolder, "mechanisms", mechanismName, "generated");
        }

    }
}
