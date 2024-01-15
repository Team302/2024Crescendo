using applicationConfiguration;
using ApplicationData;
using Configuration;
using CoreCodeGenerator;
using DataConfiguration;
using NetworkTablesUtils;
using System;
using System.CodeDom;
using System.Collections;
using System.Collections.Generic;
using System.ComponentModel;
using System.ComponentModel.DataAnnotations;
using System.Drawing;
using System.IO;
using System.Linq;
using System.Reflection;
using System.Text;
using System.Windows.Forms;
using System.Xml.Linq;
using System.Xml.Serialization;
using static Configuration.physicalUnit;

namespace FRCrobotCodeGen302
{
    public partial class MainForm : Form
    {
        toolConfiguration generatorConfig = new toolConfiguration();
        applicationDataConfig theAppDataConfiguration = new applicationDataConfig();
        codeGenerator_302Robotics codeGenerator = new codeGenerator_302Robotics();
        NTViewer viewer;
        bool needsSaving = false;
        bool loadRobotConfig = false;
        readonly string configurationCacheFile = Path.GetTempPath() + "DragonsCodeGeneratorCache.txt";
        bool automationEnabled = false;

        const int treeIconIndex_lockedPadlock = 0;
        const int treeIconIndex_unlockedPadlock = 1;
        const int treeIconIndex_gear = 2;
        const int treeIconIndex_wrench = 3;

        public MainForm()
        {
            InitializeComponent();

            string[] args = Environment.GetCommandLineArgs();
            if (args.Length > 1)
            {
                if (args[1] == "enableAutomation")
                {
                    automationEnabled = true;
                    selectNodeButton.Enabled = true;
                    selectNodeButton.Visible = true;
                    getCheckBoxListItemsButton.Enabled = true;
                    getCheckBoxListItemsButton.Visible = true;
                    checkCheckBoxListItemButton.Enabled = true;
                    checkCheckBoxListItemButton.Visible = true;
                    getSelectedTreeElementPathButton.Enabled = true;
                    getSelectedTreeElementPathButton.Visible = true;

                    infoIOtextBox.Enabled = true;
                    infoIOtextBox.Visible = true;
                    selectedNodePathTextBox.Enabled = true;
                    selectedNodePathTextBox.Visible = true;
                }
            }

            codeGenerator.setProgressCallback(addProgress);
            theAppDataConfiguration.setProgressCallback(addProgress);
            clearNeedsSaving();

            splitContainer1.SplitterDistance = splitContainer1.Width - 180;

            valueNumericUpDown.Width -= physicalUnitsComboBox.Width;
            valueComboBox.Width = valueNumericUpDown.Width;
            valueTextBox.Width = valueNumericUpDown.Width;

            valueComboBox.Location = valueNumericUpDown.Location;
            valueTextBox.Location = valueNumericUpDown.Location;
            valueDatePicker.Location = valueNumericUpDown.Location;
            physicalUnitsComboBox.Location = new Point(valueNumericUpDown.Location.X + valueNumericUpDown.Width + 3, valueNumericUpDown.Location.Y);

            this.Text += " Version " + ProductVersion;

            //initialize NT viewer
            viewer = new NTViewer(tuningButton);

            if (!automationEnabled)
            {
                #region try to load cached configuration.xml
                addProgress("Trying to load cached configuration.xml");
                try
                {
                    if (File.Exists(configurationCacheFile))
                    {
                        configurationFilePathNameTextBox.Text = File.ReadAllText(configurationCacheFile);
                        loadConfiguration(configurationFilePathNameTextBox.Text);
                        addProgress("Loaded cached configuration.xml");
                        robotConfigurationFileComboBox_TextChanged(null, null);
                    }
                    else
                    {
                        addProgress("Cached configuration.xml does not exist, robot configuration will not be automatically loaded");
                    }
                }
                catch (Exception ex)
                {
                    addProgress("Issue encountered while loading the cached generator configuration file\r\n" + ex.ToString());
                }
                #endregion
            }
            robotTreeView.ImageList = treeViewIcons;
        }

        private void addProgress(string info)
        {
            progressTextBox.AppendText(info + Environment.NewLine);
        }

        private TreeNode AddNode(TreeNode parent, object obj, string nodeName)
        {
            TreeNode tn = null;

            #region Add a new node if necessary
            TreeNodeCollection tnc = (parent == null) ? robotTreeView.Nodes : parent.Nodes;
            if (baseDataConfiguration.isACollection(obj))
            {
                // only add a collection if it has elements in it so that we hide empty collections so as not to clutter the treeview
                ICollection ic = obj as ICollection;
                if (ic.Count > 0)
                    tn = tnc.Add(nodeName);
            }
            else
            {
                tn = tnc.Add(nodeName);
            }
            #endregion

            if (tn != null) // if a node has been added
            {
                nodeTag nt = new nodeTag(nodeName, obj);
                tn.Tag = nt;

                if (baseDataConfiguration.isACollection(obj)) // if it is a collection, add an entry for each item
                {
                    ICollection ic = obj as ICollection;
                    foreach (var v in ic)
                        AddNode(tn, v, v.GetType().Name);
                }
                else // we are adding a single item which could have nested objects
                {
                    #region Record the TreeNode in the actual object so that we do not have to back search for mechanisms
                    if (obj is mechanismInstance)
                        ((mechanismInstance)obj).theTreeNode = tn;
                    else if (obj is mechanism)
                        ((mechanism)obj).theTreeNode = tn;
                    #endregion

                    string unitsAsString = "";
                    Family unitsFamily = Family.none;
                    DataConfiguration.valueRange range = null;
                    bool isConstant = false;
                    bool isTunable = false;
                    bool treatAsLeafNode = false;

                    if (obj is baseElement)
                    {
                        baseElement beObj = (baseElement)obj;
                        unitsAsString = beObj.physicalUnits;
                        unitsFamily = beObj.unitsFamily;
                        treatAsLeafNode = !beObj.showExpanded;
                        range = beObj.range;
                        isConstant = beObj.isConstant;
                        isTunable = beObj.isTunable;
                    }
                    else if (isABasicSystemType(obj))
                    {
                        treatAsLeafNode = true;

                        // everything inside baseElement is always editable, except for the name and type properties
                        if (tn.Parent != null)
                        {
                            nodeTag parentNt = (nodeTag)(tn.Parent.Tag);
                            if (parentNt != null)
                            {
                                if (parentNt.obj is baseElement)
                                {
                                    #region handle the name property
                                    if (nodeName == "name")
                                    {
                                        PropertyInfo nameProp = nodeTag.getType(parentNt).GetProperty("name", BindingFlags.Public | BindingFlags.Instance);
                                        if (nameProp != null)
                                        {
                                            isConstant = true;

                                            // if the parent of nt.obj is inside a collection, then the name will be editable, otherwise not
                                            TreeNode grandParentNode = tn.Parent.Parent;
                                            if ((grandParentNode != null) &&
                                                (baseDataConfiguration.isACollection(((nodeTag)grandParentNode.Tag).obj)))
                                                isConstant = false;
                                        }
                                    }
                                    #endregion

                                    #region handle the type property
                                    if (nodeName == "type")
                                    {
                                        PropertyInfo typeProp = nodeTag.getType(parentNt).GetProperty("type", BindingFlags.Public | BindingFlags.Instance);
                                        if (typeProp != null)
                                        {
                                            isConstant = true;
                                        }
                                    }
                                    #endregion

                                    #region handle the value property
                                    if (nodeName == "value")
                                    {
                                        PropertyInfo valueProp = nodeTag.getType(parentNt).GetProperty("value", BindingFlags.Public | BindingFlags.Instance);
                                        if (valueProp != null)
                                        {
                                            if (valueProp.GetCustomAttribute<TunableParameterAttribute>() != null)
                                                isTunable = true;
                                        }
                                    }
                                    #endregion
                                }
                                else
                                {
                                    PropertyInfo prop = nodeTag.getType(parentNt).GetProperty(nodeName, BindingFlags.Public | BindingFlags.Instance);
                                    if (prop != null)
                                    {
                                        if ((prop.GetCustomAttribute<ConstantAttribute>() != null) ||
                                            ((prop.GetCustomAttribute<ConstantInMechInstanceAttribute>() != null) && isPartOfAMechanismInaMechInstance(tn))
                                            )
                                            isConstant = true;
                                    }
                                }
                            }
                        }
                    }

                    Type objType = obj.GetType();


                    PropertyInfo[] propertyInfos = objType.GetProperties();
                    if (!treatAsLeafNode && (propertyInfos.Length > 0))
                    {
                        foreach (PropertyInfo pi in propertyInfos) // add its children
                        {
                            if (!(pi.Name.StartsWith("__") && pi.Name.EndsWith("__")))
                            {
                                if (!isHiddenPartOfbaseElement(pi.Name))
                                {
                                    object theObj = pi.GetValue(obj);

                                    if (theObj != null)
                                        AddNode(tn, theObj, pi.Name);
                                }
                            }
                        }

                        tn.Text = getDisplayName(obj, "");
                    }
                    else
                    {
                        // this means that this is a leaf node
                        int imageIndex = treeIconIndex_unlockedPadlock;
                        if (isConstant)
                            imageIndex = treeIconIndex_lockedPadlock;
                        else if (isTunable)
                            imageIndex = treeIconIndex_wrench;
                        //else if (isPartOfAMechanismInaMechInstance(tn))
                        //    imageIndex = treeIconIndex_lockedPadlock;

                        tn.ImageIndex = imageIndex;
                        tn.SelectedImageIndex = imageIndex;

                        nodeTag lnt = new nodeTag(nodeName, obj);

                        tn.Tag = lnt;

                        tn.Text = getDisplayName(treatAsLeafNode ? obj : nodeTag.getObject(parent.Tag), nodeName);
                    }
                }
            }

            return tn;
        }

        bool isHiddenPartOfbaseElement(string name)
        {
            if (name == "unitsFamily") return false;
            if (name == "name") return false;
            if (name == "parent") return true;

            return typeof(baseElement).GetProperty(name) != null;
        }

        private string getDisplayName(object obj, string instanceName)
        {
            helperFunctions.RefreshLevel refresh;
            return getDisplayName(obj, instanceName, out refresh);
        }

        private string getDisplayName(object obj, string instanceName, out helperFunctions.RefreshLevel refresh)
        {
            refresh = helperFunctions.RefreshLevel.none;

            MethodInfo mi = obj.GetType().GetMethod("getDisplayName");
            if (mi != null)
            {
                string temp = "";
                ParameterInfo[] pi = mi.GetParameters();
                if (pi.Length == 0)
                    temp = (string)mi.Invoke(obj, new object[] { });
                else if (pi.Length == 2)
                {
                    helperFunctions.RefreshLevel tempRefresh = helperFunctions.RefreshLevel.none;
                    object[] parameters = new object[] { instanceName, tempRefresh };
                    temp = (string)mi.Invoke(obj, parameters);
                    refresh = (helperFunctions.RefreshLevel)parameters[1];
                }

                return temp;
            }

            return instanceName + " (" + obj.ToString() + ")";
        }

        private static physicalUnit.Family GetTheUnitsFamilyName(TreeNode parent, object obj, string originalNodeName)
        {
            physicalUnit.Family unitsFamily = physicalUnit.Family.none;
            PropertyInfo unitsFamilyPi = nodeTag.getObject(parent.Tag).GetType().GetProperty("unitsFamily");
            if (unitsFamilyPi != null)
                unitsFamily = (physicalUnit.Family)unitsFamilyPi.GetValue(nodeTag.getObject(parent.Tag)); // the units family is defined in a property as part of the class
            else
            {
                if ((parent != null) && (nodeTag.getObject(parent.Tag) != null))
                {
                    PropertyInfo info = nodeTag.getObject(parent.Tag).GetType().GetProperty(originalNodeName);
                    if (info != null)
                    {
                        PhysicalUnitsFamilyAttribute unitFamilyAttr = info.GetCustomAttribute<PhysicalUnitsFamilyAttribute>();
                        if (unitFamilyAttr != null)
                            unitsFamily = unitFamilyAttr.family; // the units family is defined as an attribute at the usage of the class type (at the instance definition)
                    }
                }
            }

            return unitsFamily;
        }

        private string GetUnitsShortName(object obj)
        {
            string unitsAsString = null;
            PropertyInfo unitsPi = obj.GetType().GetProperty("__units__");
            if (unitsPi != null)
            {
                unitsAsString = (string)unitsPi.GetValue(obj);
            }
            if (unitsAsString == null)
                unitsAsString = "";
            return unitsAsString;
        }

        private void populateTree(applicationDataConfig theApplicationDataConfig)
        {
            robotTreeView.Nodes.Clear();
            AddNode(null, theApplicationDataConfig.theRobotVariants, "Robot Variant");
            if (theApplicationDataConfig.theRobotVariants.Robots.Count > 0)
                robotTreeView.Nodes[0].Expand();
        }

        public void loadGeneratorConfig(string configurationFullPathName)
        {
            try
            {
                generatorConfig = generatorConfig.deserialize(configurationFullPathName);
                if (generatorConfig.appDataConfigurations.Count == 0)
                {
                    generatorConfig.appDataConfigurations = new List<string>();
                    if (!string.IsNullOrEmpty(generatorConfig.robotConfiguration.Trim()))
                        generatorConfig.appDataConfigurations.Add(generatorConfig.robotConfiguration.Trim());
                }
            }
            catch (Exception ex)
            {
                throw new Exception("Cannot load the generator configuration. " + ex.Message);
            }
        }

        public void saveGeneratorConfig(string configurationFullPathName)
        {
            try
            {
                generatorConfig.serialize(configurationFullPathName);
            }
            catch (Exception ex)
            {
                throw new Exception("Cannot save the generator configuration. " + ex.Message);
            }
        }

        private void button1_Click(object sender, EventArgs e)
        {
            try
            {
                codeGenerator.generate(theAppDataConfiguration, generatorConfig);
            }
            catch (Exception ex)
            {
                MessageBox.Show("Something went wrong. See below. \r\n\r\n" + ex.Message, "Code generator error", MessageBoxButtons.OK, MessageBoxIcon.Error);
            }
        }

        private void configurationBrowseButton_Click(object sender, EventArgs e)
        {
            try
            {
                using (OpenFileDialog dlg = new OpenFileDialog())
                {
                    if (dlg.ShowDialog() == DialogResult.OK)
                    {
                        configurationFilePathNameTextBox.Text = dlg.FileName;

                        loadConfiguration(configurationFilePathNameTextBox.Text);

                        if (!automationEnabled)
                        {
                            //now that generator config has loaded succesfully, save to a temp file to save the desired config for future uses
                            File.WriteAllText(configurationCacheFile, configurationFilePathNameTextBox.Text);
                            addProgress("Wrote cached configuration.xml to: " + configurationCacheFile);
                        }
                    }
                }
            }
            catch (Exception ex)
            {
                addProgress("Issue encountered while loading the generator configuration file\r\n" + ex.ToString());
            }

            robotConfigurationFileComboBox_TextChanged(null, null);
        }

        private void loadConfiguration(string filePathName)
        {
            addProgress("Loading the generator configuration file " + filePathName);
            loadGeneratorConfig(filePathName);
            addProgress("Configuration file loaded.");

            loadRobotConfig = false;
            #region Load the Combobox with the robot configuration file list
            robotConfigurationFileComboBox.Items.Clear();
            foreach (string f in generatorConfig.appDataConfigurations)
            {
                string fullfilePath = Path.Combine(Path.GetDirectoryName(filePathName), f);
                fullfilePath = Path.GetFullPath(fullfilePath);
                robotConfigurationFileComboBox.Items.Add(fullfilePath);
            }
            #endregion

            generatorConfig.rootOutputFolder = Path.GetFullPath(Path.Combine(Path.GetDirectoryName(filePathName), generatorConfig.rootOutputFolder));
            generatorConfig.robotConfiguration = Path.GetFullPath(Path.Combine(Path.GetDirectoryName(filePathName), robotConfigurationFileComboBox.Text));
            loadRobotConfig = true;

            // select the config in the combobox after setting loadRobotConfig to true, otherwise robotConfigurationFileComboBox_TextChanged might fire before loadRobotConfig == true
            if (robotConfigurationFileComboBox.Items.Count > 0)
                robotConfigurationFileComboBox.SelectedIndex = 0;
        }

        private void robotConfigurationFileComboBox_TextChanged(object sender, EventArgs e)
        {
            if (loadRobotConfig)
            {
                try
                {
                    generatorConfig.robotConfiguration = Path.GetFullPath(Path.Combine(Path.GetDirectoryName(configurationFilePathNameTextBox.Text), robotConfigurationFileComboBox.Text));

                    try
                    {
                        theAppDataConfiguration.collectionBaseTypes = generatorConfig.collectionBaseTypes;
                        theAppDataConfiguration.physicalUnits = generatorConfig.physicalUnits;
                        theAppDataConfiguration.load(generatorConfig.robotConfiguration);
                    }
                    catch (Exception ex)
                    {
                        throw new Exception("Issue encountered while loading the robot configuration file\r\n" + ex.ToString());
                    }

                    try
                    {
                        addProgress("Populating the robot configuration tree view.");
                        populateTree(theAppDataConfiguration);
                        addProgress("... Tree view populated.");
                    }
                    catch (Exception ex)
                    {
                        throw new Exception("Issue encountered while populating the robot configuration tree view\r\n" + ex.ToString());
                    }

                    configuredOutputFolderLabel.Text = generatorConfig.rootOutputFolder;

                    deleteTreeElementButton.Enabled = false;
                    clearNeedsSaving();
                }
                catch (Exception ex)
                {
                    addProgress(ex.ToString());
                }
            }
        }

        void setNeedsSaving()
        {
            needsSaving = true;
            saveConfigBbutton.Enabled = needsSaving;
        }

        void clearNeedsSaving()
        {
            needsSaving = false;
            saveConfigBbutton.Enabled = needsSaving;
        }

        List<robotElementType> getEmptyPossibleCollectionSubTypes(object obj)
        {
            List<robotElementType> types = new List<robotElementType>();

            if (obj is nodeTag)
                obj = ((nodeTag)obj).obj;
            else if (obj is nodeTag)
                obj = nodeTag.getObject(obj);
            else
                obj = null;

            if (theAppDataConfiguration.isASubClassedCollection(obj))
            {
                Type elementType = obj.GetType().GetGenericArguments().Single();
                List<Type> subTypes = Assembly.GetAssembly(elementType).GetTypes().Where(t => t.BaseType == elementType).ToList();
                foreach (Type type in subTypes)
                    addRobotElementType(type, types);
            }
            else
            {
                PropertyInfo[] propertyInfos = obj.GetType().GetProperties();

                foreach (PropertyInfo propertyInfo in propertyInfos)
                {
                    if (theAppDataConfiguration.isASubClassedCollection(propertyInfo.PropertyType))
                    {
                        ICollection ic = propertyInfo.GetValue(obj) as ICollection;
                        if (ic.Count == 0)
                        {
                            Type elementType = propertyInfo.PropertyType.GetGenericArguments().Single();
                            List<Type> subTypes = Assembly.GetAssembly(obj.GetType()).GetTypes().Where(t => t.BaseType == elementType).ToList();
                            foreach (Type type in subTypes)
                                addRobotElementType(type, types);
                        }
                    }
                    //else if (theRobotConfiguration.isASubClassedCollection(obj.GetType()))
                    //{
                    //    //Type elementType = propertyInfo.PropertyType.GetGenericArguments().Single();
                    //    //List<Type> subTypes = Assembly.GetAssembly(obj.GetType()).GetTypes().Where(t => t.BaseType == elementType).ToList();
                    //    //foreach (Type type in subTypes)
                    //    //    types.Add(new robotElementType(type));
                    //}
                    else if (DataConfiguration.baseDataConfiguration.isACollection(propertyInfo.PropertyType))
                    {
                        ICollection ic = propertyInfo.GetValue(obj) as ICollection;
                        if (ic.Count == 0)
                            addRobotElementType(propertyInfo.PropertyType, propertyInfo.Name, types);
                    }
                    else if (propertyInfo.PropertyType == typeof(mechanismInstance))
                    {
                        addRobotElementType(propertyInfo.PropertyType, types);
                    }
                    else if (!propertyInfo.PropertyType.FullName.StartsWith("System."))
                    {
                        if (!DataConfiguration.baseDataConfiguration.isACollection(obj))
                        {
                            if (propertyInfo.GetValue(obj, null) == null)
                                addRobotElementType(propertyInfo.PropertyType, types);
                        }
                    }
                }
            }

            return types;
        }

        void addRobotElementType(Type theType, string name, List<robotElementType> types)
        {
            NotUserAddableAttribute nuaa = theType.GetCustomAttribute<NotUserAddableAttribute>();
            if (nuaa == null)
            {
                if (name == null)
                    types.Add(new robotElementType(theType));
                else
                    types.Add(new robotElementType(theType, name));

            }
        }

        void addRobotElementType(Type theType, List<robotElementType> types)
        {
            addRobotElementType(theType, null, types);
        }

        void hideAllValueEntryBoxes()
        {
            bool visible = false;

            valueComboBox.Visible = visible;
            valueNumericUpDown.Visible = visible;
            valueTextBox.Visible = visible;
            valueDatePicker.Visible = visible;
        }
        string setPhysicalUnitsComboBox(physicalUnit.Family unitsFamily, string shortUnitsName)
        {
            string updatedUnits = null;

            physicalUnitsComboBox.Items.Clear();
            physicalUnitsComboBox.Visible = unitsFamily != physicalUnit.Family.none;
            if (physicalUnitsComboBox.Visible)
            {
                List<physicalUnit> unitsList = generatorConfig.physicalUnits.FindAll(p => p.family == unitsFamily);
                foreach (physicalUnit unit in unitsList)
                {
                    physicalUnitsComboBox.Items.Add(unit);
                }

                physicalUnit units = unitsList.Find(u => u.shortName == shortUnitsName);
                if (units != null)
                    physicalUnitsComboBox.SelectedIndex = physicalUnitsComboBox.FindStringExact(units.shortName);
                else if (unitsList.Count > 0)
                {
                    physicalUnitsComboBox.SelectedIndex = 0;
                    updatedUnits = unitsList[0].shortName;
                }
            }

            return updatedUnits;
        }
        void showValueComboBox()
        {
            hideAllValueEntryBoxes();
            valueComboBox.Visible = true;
        }

        void showValueNumericUpDown()
        {
            hideAllValueEntryBoxes();
            valueNumericUpDown.Visible = true;
        }

        void showValueTextBox()
        {
            hideAllValueEntryBoxes();
            valueTextBox.Visible = true;
        }

        void showValueDatePicker()
        {
            hideAllValueEntryBoxes();
            valueDatePicker.Visible = true;
        }


        TreeNode lastSelectedValueNode = null;
        TreeNode lastSelectedArrayNode = null;
        bool enableCallback = false;
        List<robotElementType> theCurrentElementPossibilities = new List<robotElementType>();
        private void robotTreeView_AfterSelect(object sender, TreeViewEventArgs e)
        {
            valueTextBox.Visible = false;
            valueComboBox.Visible = false;
            valueNumericUpDown.Visible = false;
            physicalUnitsComboBox.Visible = false;
            addTreeElementButton.Enabled = false;

            lastSelectedArrayNode = null;
            lastSelectedValueNode = null;

            bool isInaMechanismInstance = isPartOfAMechanismInaMechInstance(e.Node);
            deleteTreeElementButton.Enabled = isDeletable(e.Node) && !isInaMechanismInstance;

            if (e.Node.Tag != null)
            {
                int nodeCount_noSubs = e.Node.GetNodeCount(false);
                int nodeCount = e.Node.GetNodeCount(true);
                bool visible_And_or_Enabled = false;

                #region Add items which can be added as children of the currently selected item
                theCurrentElementPossibilities = getEmptyPossibleCollectionSubTypes(e.Node.Tag);
                if ((theCurrentElementPossibilities.Count > 0) && (!isPartOfAMechanismInaMechInstance(e.Node)))
                {
                    visible_And_or_Enabled = true;

                    robotElementCheckedListBox.Items.Clear();
                    foreach (robotElementType t in theCurrentElementPossibilities)
                    {
                        Type elementType;
                        if (DataConfiguration.baseDataConfiguration.isACollection(t.t))
                            elementType = t.t.GetGenericArguments().Single();
                        else
                            elementType = t.t;

                        // Add the defined mechanisms as choices to add to a robot variant
                        if (elementType.Equals((new mechanismInstance()).GetType()))
                        {
                            foreach (mechanism m in theAppDataConfiguration.theRobotVariants.Mechanisms)
                            {
                                robotElementType ret = new robotElementType(m.GetType(), m);

                                robotElementCheckedListBox.Items.Add(ret);
                            }
                        }
                        else
                        {
                            robotElementCheckedListBox.Items.Add(t);
                        }
                    }
                }
                #endregion

                robotElementCheckedListBox.Visible = visible_And_or_Enabled;
                addRobotElementLabel.Visible = visible_And_or_Enabled;
                addTreeElementButton.Enabled = visible_And_or_Enabled;

                nodeTag nt = (nodeTag)e.Node.Tag;

                enableCallback = false;
                // first take care of nodes that are a collection (such as List<>)
                if (baseDataConfiguration.isACollection(nt.obj))
                {
                    lastSelectedArrayNode = e.Node;
                    addTreeElementButton.Enabled = !isInaMechanismInstance;
                }
                else if (e.Node.Parent == null) // The very top node of the whole tree
                {
                    lastSelectedValueNode = e.Node;
                }
                else
                {
                    nodeTag parentNt = (nodeTag)(e.Node.Parent.Tag);

                    //((e.Node.GetNodeCount(false) == 0) && // todo why this condition?
                    lastSelectedValueNode = e.Node;

                    object value = null;
                    bool allowEdit = false;
                    bool showUnits = false;

                    // A node can be a basic type such as double, int, string, etc
                    // or a basElement
                    // or some other class
                    if (nt.obj is baseElement)
                    {
                        baseElement beObj = ((baseElement)nt.obj);
                        if (!beObj.isConstant)
                        {
                            if (beObj.showExpanded)
                            {
                                value = nt.obj;
                            }
                            else
                            {
                                showUnits = true;
                                PropertyInfo valueProp = nodeTag.getType(nt).GetProperty("value", BindingFlags.Public | BindingFlags.Instance);
                                value = valueProp.GetValue(nt.obj);
                            }

                            allowEdit = beObj.isTunable ? true : !beObj.showExpanded;
                        }

                        
                        if ((beObj.name == "value") || showUnits)
                        {
                            string updatedUnits = setPhysicalUnitsComboBox(beObj.unitsFamily, beObj.physicalUnits);
                            if (!String.IsNullOrEmpty(updatedUnits))
                            {
                                PropertyInfo valueProp = nodeTag.getType(nt).GetProperty("__units__", BindingFlags.Public | BindingFlags.Instance);
                                if (valueProp != null)
                                {
                                    valueProp.SetValue(nt.obj, updatedUnits);
                                    beObj.physicalUnits = updatedUnits;
                                    setNeedsSaving();
                                }
                            }
                        }
                        else
                            physicalUnitsComboBox.Visible = false;
                    }
                    else if (isABasicSystemType(nt.obj))
                    {
                        allowEdit = true;
                        if (parentNt.obj is baseElement)
                        {
                            // everything inside baseElement is always editable, except for the name and type properties

                            #region handle the name property
                            PropertyInfo nameProp = nodeTag.getType(parentNt).GetProperty("name", BindingFlags.Public | BindingFlags.Instance);
                            object theNameObject = nameProp.GetValue(parentNt.obj);
                            if (theNameObject == nt.obj) // means that the nt.obj is the name property as opposed to some other string in the class
                            {
                                allowEdit = false;

                                // if the parent of nt.obj is inside a collection, then the name will be editable, otherwise not
                                TreeNode grandParentNode = e.Node.Parent.Parent;
                                if ((grandParentNode != null) &&
                                    (baseDataConfiguration.isACollection(((nodeTag)grandParentNode.Tag).obj)))
                                    allowEdit = true;
                            }
                            #endregion

                            #region handle the type property
                            PropertyInfo typeProp = nodeTag.getType(parentNt).GetProperty("type", BindingFlags.Public | BindingFlags.Instance);
                            object theTypeObject = typeProp.GetValue(parentNt.obj);
                            if (theTypeObject == nt.obj) // means that the nt.obj is the type property as opposed to some other string in the class
                                allowEdit = false;
                            #endregion

                            baseElement beParentObj = (baseElement)parentNt.obj;
                            enableCallback = false;
                            if ((nt.name == "value") || showUnits)
                            {
                                string updatedUnits = setPhysicalUnitsComboBox(beParentObj.unitsFamily, beParentObj.physicalUnits);
                                if (!String.IsNullOrEmpty(updatedUnits))
                                {
                                    PropertyInfo valueProp = nodeTag.getType(parentNt).GetProperty("__units__", BindingFlags.Public | BindingFlags.Instance);
                                    if (valueProp != null)
                                    {
                                        valueProp.SetValue(beParentObj, updatedUnits);
                                        beParentObj.physicalUnits = updatedUnits;
                                        setNeedsSaving();
                                    }
                                }
                            }
                            else
                                physicalUnitsComboBox.Visible = false;
                        }
                        else
                        {
                            // we need to check if the object is marked constant or constantInAMechanismInstance
                            PropertyInfo pi = parentNt.obj.GetType().GetProperty(nt.name);
                            if (pi != null)
                            {
                                allowEdit = !((pi.GetCustomAttribute<ConstantAttribute>() != null) ||
                                    ((pi.GetCustomAttribute<ConstantInMechInstanceAttribute>() != null) && isInaMechanismInstance));
                            }
                        }

                        value = nt.obj;
                    }

                    if (allowEdit)
                    {
                        PropertyInfo valueStringList = nodeTag.getType(nt).GetProperty("value_strings", BindingFlags.NonPublic | BindingFlags.Instance);
                        if (valueStringList != null)
                        {
                            showValueComboBox();
                            valueComboBox.Items.Clear();

                            List<string> strings = (List<string>)valueStringList.GetValue(nt.obj);
                            foreach (string en in strings)
                                valueComboBox.Items.Add(en);

                            valueComboBox.SelectedIndex = valueComboBox.FindStringExact(value.ToString());
                        }
                        else if (nodeTag.getType(nt).IsEnum)
                        {
                            showValueComboBox();
                            valueComboBox.Items.Clear();

                            string[] enumList = Enum.GetNames(nodeTag.getType(nt));
                            foreach (string en in enumList)
                                valueComboBox.Items.Add(en);

                            valueComboBox.SelectedIndex = valueComboBox.FindStringExact(value.ToString());
                        }
                        else if (value is uint)
                        {
                            valueNumericUpDown.Minimum = nt.obj is baseElement ? Convert.ToUInt32(((baseElement)nt.obj).range.minRange) : uint.MinValue;
                            valueNumericUpDown.Maximum = nt.obj is baseElement ? Convert.ToUInt32(((baseElement)nt.obj).range.maxRange) : uint.MaxValue;

                            valueNumericUpDown.DecimalPlaces = 0;
                            valueNumericUpDown.Value = (uint)value;
                            showValueNumericUpDown();
                        }
                        else if (value is int)
                        {
                            valueNumericUpDown.Minimum = nt.obj is baseElement ? Convert.ToInt32(((baseElement)nt.obj).range.minRange) : int.MinValue;
                            valueNumericUpDown.Maximum = nt.obj is baseElement ? Convert.ToInt32(((baseElement)nt.obj).range.maxRange) : int.MaxValue;

                            valueNumericUpDown.DecimalPlaces = 0;
                            valueNumericUpDown.Value = (int)value;
                            showValueNumericUpDown();
                        }
                        else if (value is double)
                        {
                            valueNumericUpDown.Minimum = nt.obj is baseElement ? Convert.ToDecimal(((baseElement)nt.obj).range.minRange) : decimal.MinValue;
                            valueNumericUpDown.Maximum = nt.obj is baseElement ? Convert.ToDecimal(((baseElement)nt.obj).range.maxRange) : decimal.MaxValue;

                            valueNumericUpDown.DecimalPlaces = 5;
                            valueNumericUpDown.Value = Convert.ToDecimal(value);
                            showValueNumericUpDown();
                        }
                        else if (value is DateTime)
                        {
                            showValueDatePicker();
                            valueDatePicker.Value = (DateTime)value;
                        }
                        else if (value is bool)
                        {
                            showValueComboBox();
                            valueComboBox.Items.Clear();

                            valueComboBox.Items.Add(true.ToString());
                            valueComboBox.Items.Add(false.ToString());

                            valueComboBox.SelectedIndex = valueComboBox.FindStringExact(value.ToString());
                        }
                        else
                        {
                            showValueTextBox();
                            valueTextBox.Text = value.ToString();
                        }
                    }
                    enableCallback = true;
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

        #region handle the events when values are changed
        private void physicalUnitsComboBox_SelectedValueChanged(object sender, EventArgs e)
        {
            if (enableCallback)
            {
                if (lastSelectedValueNode != null)
                {
                    try
                    {
                        nodeTag lnt = (nodeTag)(lastSelectedValueNode.Tag);

                        object parentObj = nodeTag.getObject(lastSelectedValueNode.Parent.Tag);

                        PropertyInfo prop = null;
                        Object objToOperateOn = lnt.obj;
                        if (DataConfiguration.baseDataConfiguration.isACollection(parentObj))
                        {
                            Type elementType = parentObj.GetType().GetGenericArguments().Single();
                            prop = elementType.GetProperty("__units__", BindingFlags.Public | BindingFlags.Instance);
                        }
                        else
                        {
                            prop = objToOperateOn.GetType().GetProperty("__units__", BindingFlags.Public | BindingFlags.Instance);
                            if (prop == null)
                            {
                                objToOperateOn = parentObj;
                                prop = objToOperateOn.GetType().GetProperty("__units__", BindingFlags.Public | BindingFlags.Instance);
                            }
                        }


                        if ((prop != null) && prop.CanWrite)
                        {
                            prop.SetValue(objToOperateOn, physicalUnitsComboBox.Text);

                            lastSelectedValueNode.Text = getDisplayName(parentObj, lnt.name);

                            if (lastSelectedValueNode.Parent != null)
                                lastSelectedValueNode.Parent.Text = getDisplayName(nodeTag.getObject(lastSelectedValueNode.Parent.Tag), "");

                            mechanism theMechanism;
                            if (isPartOfAMechanismTemplate(lastSelectedValueNode, out theMechanism))
                                updateMechInstancesFromMechTemplate(theMechanism);

                            setNeedsSaving();
                        }
                    }
                    catch (Exception)
                    {
                        MessageBox.Show("Failed to set the units of " + lastSelectedValueNode.Text + " to " + physicalUnitsComboBox.Text);
                    }
                }
            }
        }
        private void valueComboBox_SelectedValueChanged(object sender, EventArgs e)
        {
            if (enableCallback)
            {
                if (lastSelectedValueNode != null)
                {
                    try
                    {
                        nodeTag nt = (nodeTag)(lastSelectedValueNode.Tag);

                        object obj = nodeTag.getObject(nt);
                        object parentObj = nodeTag.getObject(lastSelectedValueNode.Parent.Tag);

                        PropertyInfo prop = null;
                        PropertyInfo valueStringList = null;

                        bool isValue = true;
                        if (baseDataConfiguration.isACollection(parentObj))
                        {
                            Type elementType = parentObj.GetType().GetGenericArguments().Single();
                            prop = elementType.GetProperty("value", BindingFlags.Public | BindingFlags.Instance);
                            valueStringList = elementType.GetProperty("value_strings", BindingFlags.NonPublic | BindingFlags.Instance);
                        }
                        else
                        {
                            prop = nodeTag.getType(nt).GetProperty("value", BindingFlags.Public | BindingFlags.Instance);
                            if (prop == null)
                            {
                                isValue = false;
                                prop = parentObj.GetType().GetProperty(nt.name, BindingFlags.Public | BindingFlags.Instance);
                            }
                            else
                                parentObj = nodeTag.getObject(nt);
                        }


                        if ((prop != null) && (prop.CanWrite))
                        {
                            if (valueStringList == null)
                                if (isValue)
                                    prop.SetValue(nt.obj, valueComboBox.Text == "True");
                                else
                                {
                                    if (prop.PropertyType == typeof(boolParameter))
                                        prop.SetValue(parentObj, valueComboBox.Text == "True");
                                    else
                                        prop.SetValue(parentObj, Enum.Parse(obj.GetType(), valueComboBox.Text));
                                }
                            else
                            {
                                prop.SetValue(nt.obj, valueComboBox.Text);
                            }
                            if (!isValue)
                            {
                                nt.obj = prop.GetValue(parentObj);

                                if (prop.Name == "unitsFamily")
                                {
                                    physicalUnit.Family family = (physicalUnit.Family)Enum.Parse(typeof(physicalUnit.Family), nt.obj.ToString());
                                    physicalUnit firstUnit = generatorConfig.physicalUnits.Find(p => p.family == family);

                                    // update the leafNodeTags
                                    TreeNodeCollection tnc = lastSelectedValueNode.Parent.Nodes;
                                    foreach (TreeNode node in tnc)
                                    {
                                        if ((node.Tag != null) && (node.Tag is nodeTag))
                                        {
                                            //todo  ((nodeTag)node.Tag).unitsFamily = family;
                                            //todo   ((nodeTag)node.Tag).physicalUnits = firstUnit == null ? "" : firstUnit.ToString();
                                        }
                                    }

                                    // update the __units__
                                    PropertyInfo pi = nodeTag.getObject(lastSelectedValueNode.Parent.Tag).GetType().GetProperty("__units__");
                                    if (pi != null)
                                        pi.SetValue(nodeTag.getObject(lastSelectedValueNode.Parent.Tag), firstUnit == null ? "" : firstUnit.ToString());
                                }
                            }
                         }

                        helperFunctions.RefreshLevel refresh;
                        if (isValue)
                            lastSelectedValueNode.Text = getDisplayName(parentObj, nt.name, out refresh);
                        else
                            lastSelectedValueNode.Text = getDisplayName(parentObj, prop.Name, out refresh);

                        if (lastSelectedValueNode.Parent != null)
                        {
                            if ((refresh == helperFunctions.RefreshLevel.parentHeader) || (refresh == helperFunctions.RefreshLevel.fullParent))
                                lastSelectedValueNode.Parent.Text = getDisplayName(nodeTag.getObject(lastSelectedValueNode.Parent.Tag), "");

                            if (refresh == helperFunctions.RefreshLevel.fullParent)
                            {
                                TreeNodeCollection tnc = lastSelectedValueNode.Parent.Nodes;
                                foreach (TreeNode node in tnc)
                                {
                                    node.Text = getDisplayName(nodeTag.getObject(node.Parent.Tag), ((nodeTag)node.Tag).name);
                                }
                            }
                        }

                        mechanism theMechanism;
                        if (isPartOfAMechanismTemplate(lastSelectedValueNode, out theMechanism))
                            updateMechInstancesFromMechTemplate(theMechanism);

                        setNeedsSaving();
                    }
                    catch (Exception)
                    {
                        MessageBox.Show("Failed to set " + lastSelectedValueNode.Text + " to " + valueComboBox.Text);
                    }
                }
            }
        }

        private void valueDatePicker_ValueChanged(object sender, EventArgs e)
        {
            if (enableCallback)
            {
                if (lastSelectedValueNode != null)
                {
                    try
                    {
                        nodeTag lnt = (nodeTag)(lastSelectedValueNode.Tag);

                        PropertyInfo prop = nodeTag.getObject(lastSelectedValueNode.Parent.Tag).GetType().GetProperty(lnt.name, BindingFlags.Public | BindingFlags.Instance);
                        if (null != prop && prop.CanWrite)
                        {
                            prop.SetValue(nodeTag.getObject(lastSelectedValueNode.Parent.Tag), valueDatePicker.Value);
                        }

                        lastSelectedValueNode.Text = getDisplayName(lnt.obj, lnt.name);

                        if (lastSelectedValueNode.Parent != null)
                            lastSelectedValueNode.Parent.Text = getDisplayName(nodeTag.getObject(lastSelectedValueNode.Parent.Tag), "");

                        mechanism theMechanism;
                        if (isPartOfAMechanismTemplate(lastSelectedValueNode, out theMechanism))
                            updateMechInstancesFromMechTemplate(theMechanism);

                        setNeedsSaving();
                    }
                    catch (Exception)
                    {
                        MessageBox.Show("Failed to set " + lastSelectedValueNode.Text + " to " + valueDatePicker.Value.ToShortDateString());
                    }
                }
            }
        }

        private void valueTextBox_TextChanged(object sender, EventArgs e)
        {
            if (enableCallback)
            {
                if (lastSelectedValueNode != null)
                {
                    try
                    {
                        nodeTag lnt = (nodeTag)lastSelectedValueNode.Tag;

                        bool isValue__ = true;
                        object obj = lnt.obj;
                        PropertyInfo prop = nodeTag.getType(lnt).GetProperty("value__", BindingFlags.Public | BindingFlags.Instance);
                        if (prop == null)
                        {
                            isValue__ = false;
                            obj = nodeTag.getObject(lastSelectedValueNode.Parent.Tag);
                            prop = obj.GetType().GetProperty(lnt.name, BindingFlags.Public | BindingFlags.Instance);
                        }

                        if ((prop != null) && (prop.CanWrite))
                        {
                            prop.SetValue(obj, valueTextBox.Text);

                            if (!isValue__)
                                lnt.obj = prop.GetValue(obj);

                            //if (lnt.isTunable)
                            //{
                            //    if (viewer != null)
                            //        viewer.PushValue(valueTextBox.Text, NTViewer.ConvertFullNameToTuningKey(lnt.name));
                            //}
                        }

                        helperFunctions.RefreshLevel refresh;
                        //lastSelectedValueNode.Text = getDisplayName(isValue__ ? obj : prop.GetValue(obj), lnt.name, out refresh);
                        lastSelectedValueNode.Text = getDisplayName( obj, lnt.name, out refresh);

                        if (lastSelectedValueNode.Parent != null)
                        {
                            if (refresh == helperFunctions.RefreshLevel.parentHeader)
                                lastSelectedValueNode.Parent.Text = getDisplayName(nodeTag.getObject(lastSelectedValueNode.Parent.Tag), "");

                            if (refresh == helperFunctions.RefreshLevel.fullParent)
                            {
                                TreeNodeCollection tnc = lastSelectedValueNode.Parent.Nodes;
                                foreach (TreeNode node in tnc)
                                {
                                    helperFunctions.RefreshLevel refr;
                                    node.Text = getDisplayName(((nodeTag)node.Tag).obj, ((nodeTag)node.Tag).name, out refr);
                                }
                            }
                        }

                        mechanism theMechanism;
                        if (isPartOfAMechanismTemplate(lastSelectedValueNode, out theMechanism))
                            updateMechInstancesFromMechTemplate(theMechanism);

                        setNeedsSaving();
                    }
                    catch (Exception)
                    {
                        MessageBox.Show("Failed to set " + lastSelectedValueNode.Text + " to " + valueTextBox.Text);
                    }
                }
            }
        }
        private void valueNumericUpDown_ValueChanged(object sender, EventArgs e)
        {
            if (enableCallback)
            {
                if (lastSelectedValueNode != null)
                {
                    try
                    {
                        nodeTag nt = (nodeTag)(lastSelectedValueNode.Tag);

                        if ((nt != null) && (nt.obj != null))
                        {
                            bool showExpanded = true;
                            object objToOperateOn = null;
                            PropertyInfo prop = null;
                            if (nt.obj is baseElement)
                            {
                                baseElement beObj = (baseElement)nt.obj;
                                showExpanded = beObj.showExpanded;
                                objToOperateOn = nt.obj;
                                prop = nodeTag.getType(nt).GetProperty("value", BindingFlags.Public | BindingFlags.Instance);
                            }
                            else if (((nodeTag)lastSelectedValueNode.Parent.Tag).obj is baseElement) // todo what if the parent or Tag or obj are null
                            {
                                nodeTag parentNT = (nodeTag)lastSelectedValueNode.Parent.Tag;
                                objToOperateOn = parentNT.obj;
                                prop = nodeTag.getType(parentNT).GetProperty("value", BindingFlags.Public | BindingFlags.Instance);
                            }

                            if ((prop != null) && (prop.CanWrite))
                            {
                                if (prop.PropertyType.Name == "UInt")
                                    prop.SetValue(objToOperateOn, (uint)Math.Round(valueNumericUpDown.Value, valueNumericUpDown.DecimalPlaces, MidpointRounding.AwayFromZero));
                                else if (prop.PropertyType.Name == "UInt32")
                                    prop.SetValue(objToOperateOn, (UInt32)Math.Round(valueNumericUpDown.Value, valueNumericUpDown.DecimalPlaces, MidpointRounding.AwayFromZero));
                                else if (prop.PropertyType.Name == "Int")
                                    prop.SetValue(objToOperateOn, (int)Math.Round(valueNumericUpDown.Value, valueNumericUpDown.DecimalPlaces, MidpointRounding.AwayFromZero));
                                else if (prop.PropertyType.Name == "Int32")
                                    prop.SetValue(objToOperateOn, (Int32)Math.Round(valueNumericUpDown.Value, valueNumericUpDown.DecimalPlaces, MidpointRounding.AwayFromZero));
                                else if (prop.PropertyType.Name == "Double")
                                {
                                    prop.SetValue(objToOperateOn, (double)valueNumericUpDown.Value);
                                }

                                //if (lnt.isTunable)
                                //{
                                //    if (viewer != null)
                                //        viewer.PushValue((double)valueNumericUpDown.Value, NTViewer.ConvertFullNameToTuningKey(lnt.name));
                                //}
                            }
                            else if (isABasicSystemType(nt.obj))
                            {
                                nt.obj = (double)valueNumericUpDown.Value;
                            }

                            helperFunctions.RefreshLevel refresh;
                            if (!showExpanded)
                                lastSelectedValueNode.Text = getDisplayName(objToOperateOn, nt.name, out refresh);
                            else
                                lastSelectedValueNode.Text = getDisplayName(objToOperateOn, prop.Name, out refresh);

                            if (lastSelectedValueNode.Parent != null)
                            {
                                if (refresh == helperFunctions.RefreshLevel.parentHeader)
                                    lastSelectedValueNode.Parent.Text = getDisplayName(nodeTag.getObject(lastSelectedValueNode.Parent.Tag), "");

                                if (refresh == helperFunctions.RefreshLevel.fullParent)
                                {
                                    TreeNodeCollection tnc = lastSelectedValueNode.Parent.Nodes;
                                    foreach (TreeNode node in tnc)
                                    {
                                        helperFunctions.RefreshLevel refr;
                                        node.Text = getDisplayName(((nodeTag)node.Tag).obj, ((nodeTag)node.Tag).name, out refr);
                                    }
                                }
                            }

                            mechanism theMechanism;
                            if (isPartOfAMechanismTemplate(lastSelectedValueNode, out theMechanism))
                                updateMechInstancesFromMechTemplate(theMechanism);

                            setNeedsSaving();
                        }
                    }
                    catch (Exception)
                    {
                        MessageBox.Show("Failed to set " + lastSelectedValueNode.Text + " to " + valueNumericUpDown.Text);
                    }
                }
            }
        }
        #endregion
        private void saveConfigBbutton_Click(object sender, EventArgs e)
        {
            try
            {
                theAppDataConfiguration.save(generatorConfig.robotConfiguration);
                //MessageBox.Show("File saved");
                addProgress("File saved");
                clearNeedsSaving();
            }
            catch (Exception ex)
            {
                addProgress(ex.Message);
            }
        }

        private void addTreeElementButton_Click(object sender, EventArgs e)
        {
            if (lastSelectedValueNode != null)
            {
                List<(string,object)> objectsToAddToCurrentNode = new List<(string,object)>();

                TreeNode mechanismInstancesNode = null;
                TreeNode tn = null;
                foreach (object robotElementObj in robotElementCheckedListBox.CheckedItems)
                {
                    object obj = null;

                    string name = "";
                    bool addToCollection = true;
                    if (theAppDataConfiguration.isDerivedFromGenericClass(((robotElementType)robotElementObj).t))
                    {
                        obj = Activator.CreateInstance(((robotElementType)robotElementObj).t);

                        Type baseType = ((robotElementType)robotElementObj).t.BaseType;
                        name = baseType.Name;

                        Type t = nodeTag.getType(lastSelectedValueNode.Tag);
                        PropertyInfo[] PIs = t.GetProperties();
                        List<PropertyInfo> thePIs = PIs.ToList().FindAll(p => baseDataConfiguration.isACollectionOfType(p.PropertyType, baseType));
                        if (thePIs.Count == 0)
                            addProgress("Could not find a List of " + baseType.Name);
                        else if (thePIs.Count == 1)
                            name = thePIs[0].Name;
                        else
                            addProgress("Found more than one List of " + baseType.Name);
                    }
                    else if (((robotElementType)robotElementObj).t == typeof(mechanism))
                    {
                        obj = Activator.CreateInstance((new mechanismInstance()).GetType());

                        name = "mechanismInstances";
                        ((mechanismInstance)obj).mechanism = applicationDataConfig.DeepClone((mechanism)((robotElementType)robotElementObj).theObject);
                    }
                    else if (baseDataConfiguration.isACollection(((robotElementType)robotElementObj).t))
                    {
                        Type elementType = ((robotElementType)robotElementObj).t.GetGenericArguments().Single();
                        obj = Activator.CreateInstance(elementType);

                        // get the name of the property of this type
                        PropertyInfo[] pis = nodeTag.getObject(lastSelectedValueNode.Tag).GetType().GetProperties();

                        PropertyInfo pi = pis.ToList().Find(p => p.Name == ((robotElementType)robotElementObj).name);
                        name = ((robotElementType)robotElementObj).name;
                        if (pi == null) // then try to search by the type
                        {
                            pi = pis.ToList().Find(p => p.PropertyType == ((robotElementType)robotElementObj).t);
                            name = (pi == null) ? ((robotElementType)robotElementObj).ToString() : pi.Name;
                        }
                    }
                    else
                    {
                        addToCollection = false;
                        obj = Activator.CreateInstance(((robotElementType)robotElementObj).t);
                        name = ((robotElementType)robotElementObj).ToString();
                    }

                    if (obj != null)
                    {

                        PropertyInfo pi = nodeTag.getObject(lastSelectedValueNode.Tag).GetType().GetProperty(name);
                        object theObj = pi.GetValue(nodeTag.getObject(lastSelectedValueNode.Tag), null);

                        if (name != "mechanismInstances")
                            theAppDataConfiguration.initializeData(theObj, obj, name, null);

                        if (addToCollection)
                        {
                            PropertyInfo objPi = obj.GetType().GetProperty("parent");
                            if (objPi != null)
                                objPi.SetValue(obj, theObj);

                            // then add it to the collection
                            theObj.GetType().GetMethod("Add").Invoke(theObj, new object[] { obj });
                            int count = (int)theObj.GetType().GetProperty("Count").GetValue(theObj);

                            string nameStr = "here5";
                            try
                            {
                                nameStr = obj.GetType().GetProperty("name").GetValue(obj).ToString();
                                nameStr += "_" + count;
                                PropertyInfo thisPi = obj.GetType().GetProperty("name");
                                if (thisPi != null)
                                    thisPi.SetValue(obj, nameStr);
                            }
                            catch { }

                            if (robotElementObj is mechanism)
                            {
                                if (mechanismInstancesNode == null)
                                {
                                    tn = AddNode(lastSelectedValueNode, theObj, name);
                                    mechanismInstancesNode = tn;
                                }
                                else
                                    tn = AddNode(mechanismInstancesNode, obj, name + (count - 1));
                            }
                            else
                            {
                                if (!objectsToAddToCurrentNode.Exists(o => o.Item2 == theObj))
                                    objectsToAddToCurrentNode.Add((name,theObj));

                                //if (tn == null)
                                //    tn = AddNode(lastSelectedValueNode, theObj, name); // add the List containing an element
                                //else
                                //    tn = AddNode(tn, obj, nameStr);
                            }
                        }
                        else
                        {
                            // it is not part of a collection, set the value only if it is null
                            if (theObj == null)
                            {
                                pi.SetValue(nodeTag.getObject(lastSelectedValueNode.Tag), obj);
                                tn = AddNode(lastSelectedValueNode, obj, name);
                            }
                        }
                    }

                    mechanism theMechanism;
                    if (isPartOfAMechanismTemplate(lastSelectedValueNode, out theMechanism))
                        updateMechInstancesFromMechTemplate(theMechanism);
                }

                foreach ((string,object) newObj in objectsToAddToCurrentNode)
                {
                    tn = AddNode(lastSelectedValueNode, newObj.Item2, newObj.Item1);
                }

                if (tn != null)
                {
                    tn.EnsureVisible();
                    tn.Expand();
                }

                if (tn != null)
                    robotTreeView.SelectedNode = tn;

                setNeedsSaving();
            }

            else if (lastSelectedArrayNode != null)
            {
                if (lastSelectedArrayNode.Text == "mechanismInstances")
                {
                    foreach (object robotElementObj in robotElementCheckedListBox.CheckedItems) // there should only be mechanisms in the checkedItems list 
                    {
                        robotElementType ret = (robotElementType)robotElementObj;
                        if (ret.theObject is mechanism)
                        {
                            Type elementType = ret.t;

                            object obj = Activator.CreateInstance((new mechanismInstance()).GetType());
                            ((mechanismInstance)obj).mechanism = applicationDataConfig.DeepClone((mechanism)ret.theObject);

                            // then add it to the collection
                            object theCollectionObj = nodeTag.getObject(lastSelectedArrayNode.Tag);
                            theCollectionObj.GetType().GetMethod("Add").Invoke(theCollectionObj, new object[] { obj });
                            int count = (int)theCollectionObj.GetType().GetProperty("Count").GetValue(theCollectionObj);

                            AddNode(lastSelectedArrayNode, obj, elementType.Name + (count - 1));
                        }
                    }
                }
                else
                {
                    // first create a new instance

                    if (theAppDataConfiguration.isASubClassedCollection(nodeTag.getObject(lastSelectedArrayNode.Tag).GetType()))
                    {
                        foreach (object robotElementObj in robotElementCheckedListBox.CheckedItems)
                        {
                            Type elementType = ((robotElementType)robotElementObj).t;

                            object obj = Activator.CreateInstance(elementType);
                            theAppDataConfiguration.initializeData(obj, obj, "here3", null);

                            // then add it to the collection
                            nodeTag.getType(lastSelectedArrayNode.Tag).GetMethod("Add").Invoke(nodeTag.getObject(lastSelectedArrayNode.Tag), new object[] { obj });
                            int count = (int)nodeTag.getType(lastSelectedArrayNode.Tag).GetProperty("Count").GetValue(nodeTag.getObject(lastSelectedArrayNode.Tag));

                            try
                            {
                                string nameStr = obj.GetType().GetProperty("name").GetValue(obj).ToString();
                                obj.GetType().GetProperty("name").SetValue(obj, nameStr + "_" + count);
                            }
                            catch { }

                            AddNode(lastSelectedArrayNode, obj, elementType.Name + (count - 1));
                        }
                    }
                    else
                    {
                        Type elementType = nodeTag.getType(lastSelectedArrayNode.Tag).GetGenericArguments().Single();
                        object obj = Activator.CreateInstance(elementType);
                        theAppDataConfiguration.initializeData(obj, obj, elementType.Name, null);

                        // then add it to the collection
                        nodeTag.getType(lastSelectedArrayNode.Tag).GetMethod("Add").Invoke(nodeTag.getObject(lastSelectedArrayNode.Tag), new object[] { obj });
                        int count = (int)nodeTag.getType(lastSelectedArrayNode.Tag).GetProperty("Count").GetValue(nodeTag.getObject(lastSelectedArrayNode.Tag));

                        string nameStr = "here2";
                        try
                        {
                            nameStr = obj.GetType().GetProperty("name").GetValue(obj).ToString();
                            nameStr += "_" + count;
                            obj.GetType().GetProperty("name").SetValue(obj, nameStr);
                        }
                        catch { }

                        theAppDataConfiguration.initializeData(nodeTag.getObject(lastSelectedArrayNode.Tag), obj, nameStr, null);
                        AddNode(lastSelectedArrayNode, obj, elementType.Name + (count - 1));
                    }
                }

                setNeedsSaving();

                mechanism theMechanism;
                if (isPartOfAMechanismTemplate(lastSelectedArrayNode, out theMechanism))
                    updateMechInstancesFromMechTemplate(theMechanism);
            }
        }

        void updateMechInstancesFromMechTemplate(mechanism theMechanism)
        {
            foreach (applicationData r in theAppDataConfiguration.theRobotVariants.Robots)
            {
                foreach (mechanismInstance mi in r.mechanismInstances)
                {
                    if (mi.mechanism.GUID == theMechanism.GUID)
                    {
                        mechanism m = applicationDataConfig.DeepClone(theMechanism);

                        theAppDataConfiguration.MergeMechanismParametersIntoStructure(m, mi.mechanism);

                        ((TreeNode)mi.mechanism.theTreeNode).Remove();

                        mi.mechanism = m;
                        mi.mechanism.theTreeNode = AddNode((TreeNode)mi.theTreeNode, mi.mechanism, mi.mechanism.name);
                    }
                }
            }
        }


        bool isPartOfAMechanismTemplate(TreeNode tn, out mechanism theTemplateMechanism)
        {
            List<nodeTag> lineage = new List<nodeTag>();
            string fullPath = tn.FullPath;

            if (tn != null)
            {
                lineage.Add((nodeTag)tn.Tag);
                while (tn.Parent != null)
                {
                    tn = tn.Parent;
                    lineage.Add((nodeTag)tn.Tag);
                }

                //this finds the index of the "closest" parent mechanism
                int indexOfMechanism = lineage.IndexOf(lineage.Where(x => x.obj.GetType() == typeof(mechanism)).FirstOrDefault());

                if (indexOfMechanism != -1)
                {
                    theTemplateMechanism = (mechanism)lineage[indexOfMechanism].obj;

                    //checks if the selected node is underneath the mechanisms collection.
                    //uses "EndsWith" function to make sure that the mechanisms collection is not considered part of a template
                    return fullPath.Contains("Mechanisms") && !fullPath.EndsWith("Mechanisms"); //this may be able to just return true, we should know that we are a child of a mechanism if index isn't -1
                }
            }

            theTemplateMechanism = null;
            return false;
        }

        bool isPartOfAMechanismInaMechInstance(TreeNode tn)
        {
            if (tn != null)
            {
                string fullPath = tn.FullPath;

                string mechInstancesName = @"\mechanismInstances\";
                int index = fullPath.IndexOf(mechInstancesName);
                if (index == -1)
                    return false;

                fullPath = fullPath.Substring(index + mechInstancesName.Length);

                string[] parts = fullPath.Split('\\');
                if (parts.Length <= 1)
                    return false;

                if (parts.Length > 2)
                    return true;

                if ("name (" + parts[0] + ")" == parts[1])
                    return false;

                return true;
            }

            return false;
        }

        private void MainForm_FormClosing(object sender, FormClosingEventArgs e)
        {
            if (needsSaving)
            {
                DialogResult dlgRes = MessageBox.Show("Do you want to save changes?", "302 Code Generator", MessageBoxButtons.YesNoCancel, MessageBoxIcon.Question);
                if (dlgRes == DialogResult.Yes)
                {
                    saveConfigBbutton_Click(null, null);
                    clearNeedsSaving();
                }
                else if (dlgRes == DialogResult.Cancel)
                {
                    e.Cancel = true;
                }
            }
        }

        private void clearReportButton_Click(object sender, EventArgs e)
        {
            progressTextBox.Clear();
        }
        private bool isDeletable(TreeNode tn)
        {
            TreeNode parent = tn.Parent;
            if (parent != null)
            {
                if (parent.Tag != null)
                    return DataConfiguration.baseDataConfiguration.isACollection(nodeTag.getObject(parent.Tag));
            }
            return false;
        }
        private void deleteTreeElementButton_Click(object sender, EventArgs e)
        {
            // The delete button will be disabled if the highlighted tree item cannot be deleted
            // Only a member of a collection can be deleted
            TreeNode tn = robotTreeView.SelectedNode;
            if (isDeletable(tn))
            {
                TreeNode parent = tn.Parent;
                if (parent != null)
                {
                    if ((parent.Tag != null) && (tn.Tag != null))
                    {

                        object theObjectToDelete = tn.Tag;
                        if (tn.Tag is nodeTag)
                            theObjectToDelete = ((nodeTag)tn.Tag).obj;
                        else if (tn.Tag is nodeTag)
                            theObjectToDelete = ((nodeTag)tn.Tag).obj;

                        if (theObjectToDelete != null)
                        {
                            bool updateMechanismInstances = false;
                            mechanism theMechanism;
                            if (isPartOfAMechanismTemplate(tn, out theMechanism))
                                updateMechanismInstances = true;

                            nodeTag.getObject(parent.Tag).GetType().GetMethod("Remove").Invoke(nodeTag.getObject(parent.Tag), new object[] { theObjectToDelete });
                            tn.Remove();
                            setNeedsSaving();

                            if ((int)nodeTag.getObject(parent.Tag).GetType().GetProperty("Count").GetValue(nodeTag.getObject(parent.Tag)) == 0)
                            {
                                parent.Remove();
                            }

                            if (updateMechanismInstances == true)
                                updateMechInstancesFromMechTemplate(theMechanism);

                        }
                    }
                }
            }
        }

        private void createNewAppDataConfigButton_Click(object sender, EventArgs e)
        {
            try
            {
                if (generatorConfig == null)
                    throw new Exception("Please load a configuration file before creating a new robot variants file");

                SaveFileDialog dlg = new SaveFileDialog();
                dlg.AddExtension = true;
                dlg.DefaultExt = "xml";
                dlg.Filter = "Robot Variants Files | *.xml";

                if (!string.IsNullOrEmpty(generatorConfig.robotConfiguration))
                {
                    string path = Path.GetDirectoryName(configurationFilePathNameTextBox.Text);
                    if (Directory.Exists(path))
                        dlg.InitialDirectory = path;
                }

                if (dlg.ShowDialog() == DialogResult.OK)
                {
                    using (var myFileStream = new FileStream(dlg.FileName, FileMode.Create))
                    {
                        topLevelAppDataElement newAppDataConfig = new topLevelAppDataElement();

                        var mySerializer = new XmlSerializer(typeof(topLevelAppDataElement));
                        mySerializer.Serialize(myFileStream, newAppDataConfig);
                    }

                    Uri uriNewFile = new Uri(dlg.FileName);
                    Uri uriConfigFilePath = new Uri(configurationFilePathNameTextBox.Text);
                    string realtivePath = uriConfigFilePath.MakeRelativeUri(uriNewFile).ToString();
                    if (!generatorConfig.appDataConfigurations.Contains(realtivePath))
                        generatorConfig.appDataConfigurations.Add(realtivePath);

                    int index = robotConfigurationFileComboBox.Items.IndexOf(dlg.FileName);
                    if (index == -1)
                    {
                        index = robotConfigurationFileComboBox.Items.Add(dlg.FileName);
                    }
                    robotConfigurationFileComboBox.SelectedIndex = index;

                    saveGeneratorConfig(Path.GetDirectoryName(configurationFilePathNameTextBox.Text));
                }
            }
            catch (Exception ex)
            {
                addProgress(ex.Message);
            }
        }

        private TreeNode getNode(string fullPath)
        {
            TreeNode node = null;
            List<string> splitPath = fullPath.Split('\\').ToList();
            int currentIndex = 0;

            foreach (TreeNode tn in robotTreeView.Nodes)
            {
                if (tn.Text == splitPath[currentIndex])
                {
                    if (currentIndex == splitPath.Count - 1)
                        node = tn;
                    else
                        node = selectNode(splitPath, tn, ++currentIndex);
                    break;
                }
            }

            return node;
        }

        private TreeNode selectNode(List<string> splitPath, TreeNode currentNode, int currentIndex)
        {
            TreeNode node = null;

            foreach (TreeNode tn in currentNode.Nodes)
            {
                string name = tn.Text;

                if (name == splitPath[currentIndex])
                {
                    if (currentIndex == splitPath.Count - 1)
                        node = tn;
                    else
                        node = selectNode(splitPath, tn, ++currentIndex);
                    break;
                }
            }

            return node;
        }

        private void selectNodeButton_Click(object sender, EventArgs e)
        {
            string nodePath = infoIOtextBox.Text;
            TreeNode n = getNode(nodePath);

            if (n != null)
            {
                robotTreeView.SelectedNode = n;
                n.EnsureVisible();
            }
        }
        private void getCheckBoxListItemsButton_Click(object sender, EventArgs e)
        {
            StringBuilder sb = new StringBuilder();

            if (infoIOtextBox.Text == "ComboBox")
            {
                foreach (object obj in valueComboBox.Items)
                {
                    sb.Append("#" + obj.ToString());
                }
            }
            else if (infoIOtextBox.Text == "CheckListBox")
            {
                foreach (robotElementType ret in robotElementCheckedListBox.Items)
                {
                    sb.Append("#" + ret.name);
                }
            }

            infoIOtextBox.Text = sb.ToString();
        }
        private void getSelectedTreeElementPathButton_Click(object sender, EventArgs e)
        {
            infoIOtextBox.Text = robotTreeView.SelectedNode == null ? "" : robotTreeView.SelectedNode.FullPath;
        }
        private void checkCheckBoxListItemButton_Click(object sender, EventArgs e)
        {
            string text = infoIOtextBox.Text.Trim();
            string[] list = text.Split(':');
            if (list.Length == 2)
            {
                try
                {
                    int index = Convert.ToInt32(list[1]);

                    if (list[0] == "ComboBox")
                        valueComboBox.SelectedIndex = index;
                    if (list[0] == "CheckListBox")
                        robotElementCheckedListBox.SetItemChecked(index, true);
                }
                catch
                {
                }
            }
        }

        private void tuningButton_Click(object sender, EventArgs e)
        {
            if (viewer != null)
                viewer.ConnectToNetworkTables();
        }
    }

    class valueRange
    {
        public double minRange { get; set; }
        public double maxRange { get; set; }
    }

    class defaultValue
    {
        public object value { get; set; }
    }

    class nodeTag
    {
        public string name { get; private set; }
        public object obj { get; set; }

        public nodeTag(string name, object obj)
        {
            this.name = name;
            this.obj = obj;
        }

        static public object getObject(object nt)
        {
            if (nt == null)
                return null;

            return ((nodeTag)nt).obj;
        }
        static public Type getType(object nt)
        {
            if (nt == null)
                return null;

            return ((nodeTag)nt).obj.GetType();
        }
        static public string getName(object nt)
        {
            if (nt == null)
                return null;

            return ((nodeTag)nt).name;
        }
    }

    class robotElementType
    {
        public Type t;
        public string name;
        public object theObject;

        public robotElementType(Type t)
        {
            this.t = t;

            string s = t.ToString();
            int indexlastDot = s.LastIndexOf('.');
            s = s.Substring(indexlastDot + 1);
            name = s.TrimEnd(']');
            theObject = null;
        }

        public robotElementType(Type t, string name)
        {
            this.t = t;
            this.name = name;
            theObject = null;
        }

        public robotElementType(Type t, mechanism m)
        {
            this.t = t;
            this.name = m.name;
            theObject = m;
        }

        public override string ToString()
        {
            return name;
        }
    }
}
