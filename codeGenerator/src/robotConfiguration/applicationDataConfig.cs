using ApplicationData;
using Configuration;
using DataConfiguration;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.ComponentModel;
using System.ComponentModel.DataAnnotations;
using System.IO;
using System.Linq;
using System.Reflection;
using System.Runtime.Serialization.Formatters.Binary;
using System.Security.Cryptography;
using System.Xml;
using System.Xml.Serialization;

namespace applicationConfiguration
{
    public class applicationDataConfig : baseDataConfiguration
    {
        public topLevelAppDataElement theRobotVariants = new topLevelAppDataElement();

        override public void load(string theRobotConfigFullPathFileName)
        {
            try
            {
                string rootRobotConfigFolder = Path.GetDirectoryName(theRobotConfigFullPathFileName);

                addProgress("Loading robot configuration " + theRobotConfigFullPathFileName);
                theRobotVariants = loadRobotConfiguration(theRobotConfigFullPathFileName);

                // switch off validation... maybe switch it on once we understand how it works
                //foreach (applicationData theRobot in theRobotVariants.robot)
                //{
                //    ValidationContext context = new ValidationContext(theRobot.pdp);
                //    IList<ValidationResult> errors = new List<ValidationResult>();

                //    addProgress("Validating Robot with ID " + theRobot.robotID);
                //    if (!Validator.TryValidateObject(theRobot.pdp, context, errors, true))
                //    {
                //        addProgress("Error(s) found ");
                //        //todo should the error be "fixed" without user intervention?
                //        foreach (ValidationResult result in errors)
                //            addProgress(result.ErrorMessage);
                //    }
                //    else
                //        addProgress("Validation passed");
                //}
            }
            catch (Exception ex)
            {
                progressCallback(ex.Message);
            }
        }

        override public void save(string theRobotConfigFullPathFileName)
        {
            try
            {
                string rootRobotConfigFolder = Path.GetDirectoryName(theRobotConfigFullPathFileName);

                addProgress("Saving robot configuration " + theRobotConfigFullPathFileName);
                saveRobotConfiguration(theRobotConfigFullPathFileName);
            }
            catch (Exception ex)
            {
                progressCallback(ex.Message);
            }
        }

        private topLevelAppDataElement loadRobotConfiguration(string fullPathName)
        {
            topLevelAppDataElement theRobotVariants;

            var mySerializer = new XmlSerializer(typeof(topLevelAppDataElement));
            using (var myFileStream = new FileStream(fullPathName, FileMode.Open))
            {
                theRobotVariants = (topLevelAppDataElement)mySerializer.Deserialize(myFileStream);
            }

            for (int m = 0; m < theRobotVariants.Mechanisms.Count; m++)
            {
                string mechanismFullPath = Path.Combine(Path.GetDirectoryName(fullPathName), theRobotVariants.Mechanisms[m].name + ".xml");

                addProgress("Loading mechanism configuration " + mechanismFullPath);
                mySerializer = new XmlSerializer(typeof(mechanism));
                using (var myFileStream = new FileStream(mechanismFullPath, FileMode.Open))
                {
                    theRobotVariants.Mechanisms[m] = (mechanism)mySerializer.Deserialize(myFileStream);
                }
            }

            //try loading any additional mechanisms in cofiguration directory
            string[] files = Directory.GetFiles(Path.GetDirectoryName(fullPathName), "*.xml");
            foreach (string file in files)
            {
                if (theRobotVariants.Mechanisms.Any(p => p.name == Path.GetFileNameWithoutExtension(file)))
                {
                    //if we have previously loaded the mechanism, don't load it again
                    continue;
                }
                else
                {
                    string mechanismFullPath = Path.Combine(Path.GetDirectoryName(fullPathName), file);

                    string tempFile = File.ReadAllText(mechanismFullPath);

                    mechanism tempMech;

                    //ignore configuration files
                    if (!tempFile.Contains("topLevelAppDataElement") && !tempFile.Contains("toolConfiguration"))
                    {
                        mySerializer = new XmlSerializer(typeof(mechanism));

                        using (var myFileStream = new FileStream(mechanismFullPath, FileMode.Open))
                        {
                            tempMech = mySerializer.Deserialize(myFileStream) as mechanism;
                        }

                        //if we have two versions of a mechanism with the same name, append a number to the end of the newest one
                        int numberOfSameNamedMechs = theRobotVariants.Mechanisms.Where(p => p.name == tempMech.name).Count();
                        if (numberOfSameNamedMechs > 0)
                        {
                            tempMech.name += numberOfSameNamedMechs;
                        }

                        theRobotVariants.Mechanisms.Add(tempMech);
                    }
                }
            }

            foreach (applicationData theRobot in theRobotVariants.Robots)
            {
                foreach (mechanismInstance mi in theRobot.mechanismInstances)
                {
                    MergeMechanismParametersIntoStructure(loadMechanism(fullPathName, mi.mechanism.name), mi.mechanism);
                }

                helperFunctions.initializeNullProperties(theRobot, true);
            }

            initializeData(null, theRobotVariants, "theRobotVariants", new List<Attribute>());


            //foreach (robot theRobot in theRobotVariants.robot)
            //{
            //    for (int i = 0; i < theRobot.mechanismInstance.Count; i++)
            //    {
            //        mechanism mech = theRobot.mechanismInstance[i].mechanism;

            //        mySerializer = new XmlSerializer(typeof(mechanism));
            //        string mechanismFullPath = Path.Combine(Path.GetDirectoryName(fullPathName), mech.name + ".xml");

            //        addProgress("Loading mechanism configuration " + mechanismFullPath);
            //        using (var myFileStream = new FileStream(mechanismFullPath, FileMode.Open))
            //        {
            //            mech = (mechanism)mySerializer.Deserialize(myFileStream);
            //        }
            //    }
            //}
            return theRobotVariants;
        }

        public void initializeData(object parent, object obj, string objectName, List<Attribute> attributes)
        {
            if (obj == null)
                return;

            PropertyInfo[] PIs = obj.GetType().GetProperties(BindingFlags.Instance | BindingFlags.Static | BindingFlags.Public | BindingFlags.NonPublic);

            foreach (PropertyInfo pi in PIs)
            {
                if (pi.Name == "name")
                {
                    pi.SetValue(obj, objectName);
                }
                else if (isACollection(pi.PropertyType))
                {
                    ICollection listObject = pi.GetValue(obj) as ICollection;
                    int index = 0;
                    foreach (object o in listObject)
                    {
                        initializeData(listObject, o, string.Format("{0}[{1}]", pi.Name, index), null);
                    }
                }
                else
                {
                    if (obj is baseElement)
                    {
                        baseElement beObj = (baseElement)obj;
                        beObj.parent = parent;

                        PropertyInfo namePI = obj.GetType().GetProperty("name");
                        if (namePI != null)
                            namePI.SetValue(obj, objectName);

                        if (attributes != null)
                        {
                            beObj.isTunable = attributes.Find(a => a.GetType() == typeof(TunableParameterAttribute)) != null;
                            beObj.isConstant = (attributes.Find(a => a.GetType() == typeof(ConstantAttribute)) != null) ||
                                attributes.Find(a => a.GetType() == typeof(ConstantInMechInstanceAttribute)) != null;

                            PhysicalUnitsFamilyAttribute phyUnitsFamilyAttr = (PhysicalUnitsFamilyAttribute)attributes.Find(a => a.GetType() == typeof(PhysicalUnitsFamilyAttribute));
                            if (phyUnitsFamilyAttr != null)
                            {
                                beObj.unitsFamily = phyUnitsFamilyAttr.family;
                                beObj.unitsFamilyDefinedByAttribute = true;
                            }

                            RangeAttribute rangeAttr = (RangeAttribute)attributes.Find(a => a.GetType() == typeof(RangeAttribute));
                            if (rangeAttr != null)
                            {
                                beObj.range.maxRange = Convert.ToDouble(rangeAttr.Maximum);
                                beObj.range.minRange = Convert.ToDouble(rangeAttr.Minimum);
                            }

                            DefaultValueAttribute defaultValueAttr = (DefaultValueAttribute)attributes.Find(a => a.GetType() == typeof(DefaultValueAttribute));
                            if (defaultValueAttr != null)
                                beObj.theDefault.value = defaultValueAttr.Value;

                            DataDescriptionAttribute descriptionAttr = (DataDescriptionAttribute)attributes.Find(a => a.GetType() == typeof(DataDescriptionAttribute));
                            if (descriptionAttr != null)
                                beObj.description = descriptionAttr.description;
                        }

                        if (beObj.unitsFamily != physicalUnit.Family.none)
                        {
                            PropertyInfo unitsInfo = obj.GetType().GetProperty("__units__");

                            if (unitsInfo != null)
                            {
                                object unitsObj = unitsInfo.GetValue(obj);
                                if ((unitsObj == null) || (unitsObj.ToString() == ""))
                                {
                                    physicalUnit pu = physicalUnits.Find(u => u.family == beObj.unitsFamily);
                                    unitsInfo.SetValue(obj, pu.shortName);
                                }
                            }
                        }

                        break;
                    }
                    else if (pi.PropertyType != typeof(string))
                    {
                        List<Attribute> theAttributes = pi.GetCustomAttributes().ToList();
                        object theObj = pi.GetValue(obj);
                        initializeData(obj, theObj, pi.Name, theAttributes);
                    }
                    else
                    {

                    }
                }
            }
        }

        public void initializeUnits(object obj, physicalUnit.Family family)
        {
            //PropertyInfo unitsInfo = obj.GetType().GetProperty("__units__");

            //physicalUnit.Family theUnitFamily = physicalUnit.Family.unitless;
            //PropertyInfo unitFamilyInfo = obj.GetType().GetProperty("unitsFamily");
            //if (unitFamilyInfo != null)
            //    theUnitFamily = (physicalUnit.Family)unitFamilyInfo.GetValue(obj);

            //if (unitsInfo != null)
            //{
            //    string theUnits = (string)unitsInfo.GetValue(obj);

            //    theUnitFamily = family != physicalUnit.Family.unitless ? family : theUnitFamily;

            //    if (string.IsNullOrEmpty(theUnits) && (theUnitFamily != physicalUnit.Family.unitless))
            //    {
            //        physicalUnit pu = physicalUnits.Find(u => u.family == theUnitFamily);
            //        unitsInfo.SetValue(obj, pu.shortName, null);
            //    }
            //}
            //else
            //{
            //    PropertyInfo[] PIs = obj.GetType().GetProperties();
            //    foreach (PropertyInfo pi in PIs)
            //    {
            //        if (isACollection(pi.PropertyType))
            //        { 
            //            ICollection listObject = pi.GetValue(obj) as ICollection;
            //            foreach(object o in listObject)
            //                initializeUnits(o, physicalUnit.Family.unitless);
            //        }
            //        else
            //        {
            //            if (pi.PropertyType != typeof(string))
            //            {
            //                PhysicalUnitsFamilyAttribute theFamily = pi.GetCustomAttribute<PhysicalUnitsFamilyAttribute>();
            //                object theObj = pi.GetValue(obj);
            //                initializeUnits(theObj, theFamily==null ? physicalUnit.Family.unitless : theFamily.family);
            //            }
            //        }
            //    }
            //}
        }

        private mechanism loadMechanism(string fullPathName, string mechanismName)
        {
            var mySerializer = new XmlSerializer(typeof(mechanism));

            string mechanismFullPath = Path.Combine(Path.GetDirectoryName(fullPathName), mechanismName + ".xml");

            using (var myFileStream = new FileStream(mechanismFullPath, FileMode.Open))
            {
                mechanism m = (mechanism)mySerializer.Deserialize(myFileStream);
                return m;
            }
        }



        /// <summary>
        /// Merges the structure and default values from structureSource to parametersSource
        /// </summary>
        /// <param name="structureSource"></param>
        /// <param name="parametersSource"></param>
        public void MergeMechanismParametersIntoStructure(object structureSource, object parametersSource)
        {
            if ((structureSource != null) && (parametersSource != null))
            {
                if (isACollection(structureSource))
                {
                    ICollection ics = structureSource as IList;
                    ICollection icp = parametersSource as IList;
                    foreach (var v in ics)
                    {
                        PropertyInfo[] propertyInfos = v.GetType().GetProperties();
                        PropertyInfo pi = propertyInfos.ToList().Find(p => p.Name == "name");
                        if (pi != null)
                        {
                            string structureName = pi.GetValue(v).ToString(); ;

                            foreach (var vParam in icp)
                            {
                                string s = pi.GetValue(vParam).ToString();
                                if ((s != null) && (s == structureName))
                                {
                                    MergeMechanismParametersIntoStructure(v, vParam);
                                }
                            }
                        }
                    }
                }
                else
                {
                    Type objType = structureSource.GetType();

                    PropertyInfo[] propertyInfos = objType.GetProperties();

                    //if (isATunableParameterType(objType.FullName))
                    //{
                    //    PropertyInfo pi = propertyInfos.ToList().Find(p => p.Name == "value");
                    //    if (pi != null)
                    //    {
                    //        pi.SetValue(structureSource, pi.GetValue(parametersSource));
                    //    }
                    //}
                    //else if (isAParameterType(objType.FullName))
                    //{
                    //    PropertyInfo pi = propertyInfos.ToList().Find(p => p.Name == "value");
                    //    if (pi != null)
                    //    {
                    //        pi.SetValue(structureSource, pi.GetValue(parametersSource));
                    //    }
                    //}
                    //else
                    if ((objType.FullName == "System.String") || (objType.FullName == "System.DateTime"))
                    {

                    }
                    else if (isABasicSystemType(structureSource))
                    {
                        structureSource = parametersSource;
                    }
                    else
                    {
                        foreach (PropertyInfo pi in propertyInfos)
                        {
                            if (pi.Name != "parent")
                            {
                                object theStructureObj = pi.GetValue(structureSource);
                                object theParametersObj = pi.GetValue(parametersSource);

                                if (isABasicSystemType(theStructureObj))
                                {
                                    if (pi.GetCustomAttribute<ConstantInMechInstanceAttribute>() == null)
                                        pi.SetValue(structureSource, pi.GetValue(parametersSource));
                                }
                                else if ((theStructureObj != null) && (theParametersObj != null))
                                {
                                    MergeMechanismParametersIntoStructure(theStructureObj, theParametersObj);
                                }
                            }
                        }
                    }

                    //if (!isAParameterType(objType.FullName) && (objType.FullName != "System.String") && (propertyInfos.Length > 0))
                    //{
                    //    // add its children
                    //    string previousName = "";
                    //    foreach (PropertyInfo pi in propertyInfos)
                    //    {
                    //        object theObj = pi.GetValue(obj);

                    //        //strings have to have some extra handling
                    //        if (pi.PropertyType.FullName == "System.String")
                    //        {
                    //            if (theObj == null)
                    //            {
                    //                theObj = "";
                    //                pi.SetValue(obj, "");
                    //            }
                    //        }

                    //        if (theObj != null)
                    //        {
                    //                AddNode(tn, theObj, pi.Name);
                    //                previousName = pi.Name;
                    //        }
                    //    }
                    //}
                    //else
                    //{
                    //    // this means that this is a leaf node
                    //    leafNodeTag lnt = new leafNodeTag(obj.GetType(), nodeName, obj);
                    //    tn.Tag = lnt;
                    //}
                }
            }
        }

        bool isABasicSystemType(object obj)
        {
            if (obj is double) return true;
            if (obj is float) return true;
            if (obj is int) return true;
            if (obj is uint) return true;
            if (obj is bool) return true;
            if (obj is Enum) return true;
            if (obj is string) return true;
            if (obj is DateTime) return true;

            return false;
        }
        public static mechanism DeepClone(mechanism obj)
        {
            using (var ms = new MemoryStream())
            {
                var formatter = new BinaryFormatter();
                formatter.Serialize(ms, obj);
                ms.Position = 0;

                return (mechanism)formatter.Deserialize(ms);
            }
        }

        private void saveRobotConfiguration(string fullPathName)
        {
            XmlWriterSettings xmlWriterSettings = new XmlWriterSettings();
            xmlWriterSettings.NewLineOnAttributes = true;
            xmlWriterSettings.Indent = true;


            foreach (mechanism mech in theRobotVariants.Mechanisms)
            {
                string mechanismFullPath = Path.Combine(Path.GetDirectoryName(fullPathName), mech.name + ".xml");

                addProgress("Writing mechanism file " + mechanismFullPath);
                try
                {
                    using (XmlWriter mechtw = XmlWriter.Create(mechanismFullPath, xmlWriterSettings))
                    {
                        var mechSerializer = new XmlSerializer(typeof(mechanism));
                        mechSerializer.Serialize(mechtw, mech);
                    }
                }
                catch (Exception ex)
                {
                    addProgress("Problem encountered while writing mechanism file " + mechanismFullPath);
                    addProgress(ex.ToString());
                }
            }

            // after saving the mechanisms into separate files, clear the list of mechanisms, except for the name
            // so that the mechanism xml is blank in the robot config file...will not lead to conflicts when  multiple people change
            // different mechanisms. Restore the list after saving to xml
            List<mechanism> tempList = new List<mechanism>();
            foreach (mechanism mech in theRobotVariants.Mechanisms)
                tempList.Add(mech);

            theRobotVariants.Mechanisms.Clear();

            foreach (mechanism mech in tempList)
            {
                mechanism temp = new mechanism();
                temp.name = mech.name;
                theRobotVariants.Mechanisms.Add(temp);
            }

            var robotSerializer = new XmlSerializer(typeof(topLevelAppDataElement));
            XmlWriter tw = XmlWriter.Create(fullPathName, xmlWriterSettings);
            robotSerializer.Serialize(tw, theRobotVariants);

            tw.Close();

            theRobotVariants.Mechanisms.Clear();

            foreach (mechanism mech in tempList)
            {
                theRobotVariants.Mechanisms.Add(mech);
            }
        }
    }
}
