using Microsoft.VisualStudio.TestTools.UnitTesting;
using OpenQA.Selenium;
using OpenQA.Selenium.Appium;
using OpenQA.Selenium.Appium.Windows;
using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.Diagnostics;
using System.IO;
using System.Linq;
using System.Reflection;
using System.Runtime.CompilerServices;
using System.Threading;

namespace UnitTests
{
    [TestClass]
    public class UnitTests
    {
        private const string WinAppDriver = @"C:\Program Files\Windows Application Driver\WinAppDriver.exe";
        private const string WindowsApplicationDriverUrl = "http://127.0.0.1:4723";
        private const string applicationFullPathName = @"C:\FRC\2023SummerProject\codeGenerator\src\FRCrobotCodeGen302\bin\Debug\FRCrobotCodeGen302.exe";
        private static WindowsDriver<WindowsElement> Session { get; set; }
        private static Process theWinAppDriverProcess = null;

        //============================ configurable settings
        private static string testRobotConfigurationFile = "testRobotConfiguration.xml";
        private static string configurationFileReference = "configurationFileReference.xml";
        private static string configurationFile = "configuration.xml";

        private static string baseTestDirectory = "guiTests";
        private static string referenceFilesDirectory = "referenceFiles";
        private static string configDirectory = "configuration";
        private static string robotConfigDirectory = "robotConfiguration";

        // debug settings
        private double pauseAtEndOfEachTest = 1;

        [ClassInitialize]
        public static void ClassInitialize(TestContext testContext)
        {
            string codeGeneratorDir = Path.GetFullPath(Path.Combine(testContext.TestDir, "..", "..", ".."));

            baseTestDirectory = Path.Combine(codeGeneratorDir, baseTestDirectory);
            referenceFilesDirectory = Path.Combine(baseTestDirectory, referenceFilesDirectory);
            robotConfigDirectory = Path.Combine(baseTestDirectory, robotConfigDirectory);
            configDirectory = Path.Combine(baseTestDirectory, configDirectory);

            testRobotConfigurationFile = Path.Combine(robotConfigDirectory, testRobotConfigurationFile);
            configurationFileReference = Path.Combine(baseTestDirectory, configurationFileReference);
            configurationFile = Path.Combine(configDirectory, configurationFile);

            Console.WriteLine(codeGeneratorDir);
            Console.WriteLine(baseTestDirectory);
            Console.WriteLine(referenceFilesDirectory);
            Console.WriteLine(robotConfigDirectory);

            theWinAppDriverProcess = Process.Start(WinAppDriver);
            Thread.Sleep(TimeSpan.FromSeconds(2)); // to allow time for the win app driver to start 

            AppiumOptions options = new AppiumOptions();
            options.AddAdditionalCapability("app", applicationFullPathName);
            options.AddAdditionalCapability("appArguments", "enableAutomation");
            options.AddAdditionalCapability("deviceName", "WindowsPC");

            Session = new WindowsDriver<WindowsElement>(new Uri(WindowsApplicationDriverUrl), options);
            Assert.IsNotNull(Session);

            // Set implicit timeout to 1.5 seconds to make element search to retry every 500 ms for at most three times
            Session.Manage().Timeouts().ImplicitWait = TimeSpan.FromSeconds(1.5);
        }

        [ClassCleanup]
        public static void ClassCleanup()
        {
            // Close the application and delete the session
            if (Session != null)
            {
                Session.Quit();
                Session = null;
            }

            if (theWinAppDriverProcess != null)
                theWinAppDriverProcess.Kill();
        }

        /// <summary>
        /// Creates a new Robot variants configuration in a clean directory
        /// </summary>
        [TestMethod]
        public void TestMethod_00000_CreateNewConfiguration()
        {
            #region directory cleanup
            if (Directory.Exists(robotConfigDirectory))
                Directory.Delete(robotConfigDirectory, true);
            Directory.CreateDirectory(robotConfigDirectory);

            if (Directory.Exists(configDirectory))
                Directory.Delete(configDirectory, true);
            Directory.CreateDirectory(configDirectory);
            #endregion

            File.Copy(configurationFileReference, configurationFile);

            Session.FindElementByName("Main").Click();

            Thread.Sleep(TimeSpan.FromSeconds(0.5)); // time to switch tabs

            Session.FindElementByAccessibilityId("configurationBrowseButton").Click();
            Thread.Sleep(TimeSpan.FromSeconds(1)); // wait a bit for the browse file dialog to pop up
            Session.FindElementByXPath("//Edit[@Name='File name:']").SendKeys(configurationFile);
            Thread.Sleep(TimeSpan.FromSeconds(1.5));
            Session.FindElementByXPath("//Edit[@Name='File name:']").SendKeys(Keys.Enter);

            Thread.Sleep(TimeSpan.FromSeconds(3)); // time for config to load

            Session.FindElementByAccessibilityId("createNewRobotVariantsConfigButton").Click();
            Thread.Sleep(TimeSpan.FromSeconds(1)); // wait a bit for the save file dialog to pop up
            Session.FindElementByXPath("//Edit[@Name='File name:']").SendKeys(testRobotConfigurationFile);
            Thread.Sleep(TimeSpan.FromSeconds(0.5));
            Session.FindElementByName("Save").Click();

            Thread.Sleep(TimeSpan.FromSeconds(3)); // wait for the file to be written
            Assert.IsTrue(File.Exists(testRobotConfigurationFile));
        }

        [TestMethod]
        public void TestMethod_00001_AddARobot()
        {
            Session.FindElementByName("Configuration").Click();

            Thread.Sleep(TimeSpan.FromSeconds(0.5)); // time to switch tabs

            selectTreeNodeAndCheck(@"robotBuildDefinition");

            addRobotElement("Robots");

            selectTreeNodeAndCheck(@"robotBuildDefinition\Robots\Robot #1\robotID (1)");

            clickSave();
        }


        [TestMethod]
        public void TestMethod_00002_SetRobotNumber()
        {
            selectTreeNodeAndCheck(@"robotBuildDefinition\Robots\Robot #1\robotID (1)");

            setNumericUpDown(2);

            selectTreeNodeAndCheck(@"robotBuildDefinition\Robots\Robot #2\robotID (2)");

            clickSave();
        }

        [TestMethod]
        public void TestMethod_00003_SetRobotPdpToCTRE()
        {
            selectTreeNodeAndCheck(@"robotBuildDefinition\Robots\Robot #2\Pdp (REV)\Pdp (REV)");

            selectInComboBox("CTRE");

            selectTreeNodeAndCheck(@"robotBuildDefinition\Robots\Robot #2\Pdp (CTRE)\Pdp (CTRE)");

            clickSave();
        }

        [TestMethod]
        public void TestMethod_00004_RenameTestClass()
        {
            selectTreeNodeAndCheck(@"robotBuildDefinition\Robots\Robot #2\testClass\name (testClass)");

            setTextInput("myClass");

            selectTreeNodeAndCheck(@"robotBuildDefinition\Robots\Robot #2\myClass\name (myClass)");

            clickSave();
        }

        [TestMethod]
        public void TestMethod_00005_Set_aDouble()
        {
            string basePath = @"robotBuildDefinition\Robots\Robot #2\myClass\aDouble (##value##)";
            setAndCheckDouble(basePath, -4, -40, "m", -10, 10);

            clickSave();
        }

        [TestMethod]
        public void TestMethod_00006_AddAMechanismTemplate()
        {
            selectTreeNodeAndCheck(@"robotBuildDefinition");

            addRobotElement("Mechanisms");

            selectTreeNodeAndCheck(@"robotBuildDefinition\Mechanisms\mechanism\name (mechanism)");
            setTextInput("Super_Intake");
            selectTreeNodeAndCheck(@"robotBuildDefinition\Mechanisms\Super_Intake\name (Super_Intake)");

            selectTreeNodeAndCheck(@"robotBuildDefinition\Mechanisms\Super_Intake");
            addRobotElement("testFalcon_Motor");

            selectTreeNodeAndCheck(@"robotBuildDefinition\Mechanisms\Super_Intake\testMotor");
            addRobotElement("testTalonSRX_Motor");

            clickSave();
        }

        /*         [TestMethod]
                public void TestMethod_00003_RenameFalcon_Motor()
                {
                    selectTreeNodeAndCheck(@"robotBuildDefinition\mechanisms\Super_Intake\motors\Falcon_1\name (Falcon_1)");
                    setTextInput("intakePedalMotor");
                    selectTreeNodeAndCheck(@"robotBuildDefinition\mechanisms\Super_Intake\motors\intakePedalMotor\name (intakePedalMotor)");

                    clickSave();
                }

                [TestMethod]4 m
                public void TestMethod_00004_RenameSolenoid()
                {
                    selectTreeNodeAndCheck(@"robotBuildDefinition\mechanisms\Super_Intake\solenoids\solenoid_1\name (solenoid_1)");
                    setTextInput("intakePusher");
                    selectTreeNodeAndCheck(@"robotBuildDefinition\mechanisms\Super_Intake\solenoids\intakePusher\name (intakePusher)");

                    clickSave();
                }

                [TestMethod]
                public void TestMethod_00005_CheckThatMotorsNotAvailableToAddAtMechLevel()
                {
                    selectTreeNodeAndCheck(@"robotBuildDefinition\mechanisms\Super_Intake");

                    List<string> elements = getListOfAvailableRobotElements();
                    Assert.AreEqual(0, elements.Count(e => e.EndsWith("_Motor")));

                    clickSave();
                }

                [TestMethod]
                public void TestMethod_00006_AddA2ndMotor()
                {
                    selectTreeNodeAndCheck(@"robotBuildDefinition\mechanisms\Super_Intake\motors");
                    addRobotElement("TalonSRX_Motor");
                    selectTreeNodeAndCheck(@"robotBuildDefinition\mechanisms\Super_Intake\motors\TalonSRX_2\name (TalonSRX_2)");

                    setTextInput("intakeFan");
                    selectTreeNodeAndCheck(@"robotBuildDefinition\mechanisms\Super_Intake\motors\intakeFan\name (intakeFan)");

                    clickSave();
                }

                [TestMethod]
                public void TestMethod_00007_AddA2ndMechanism()
                {
                    selectTreeNodeAndCheck(@"robotBuildDefinition\mechanisms");
                    addRobotElement("");
                    selectTreeNodeAndCheck(@"robotBuildDefinition\mechanisms\mechanism_2\name (mechanism_2)");

                    setTextInput("floorExtender");
                    selectTreeNodeAndCheck(@"robotBuildDefinition\mechanisms\floorExtender\name (floorExtender)");

                    clickSave();
                }

                [TestMethod]
                public void TestMethod_00008_AddAmechanismInstanceToRobot2()
                {
                    selectTreeNodeAndCheck(@"robotBuildDefinition\robots\Robot #2");
                    addRobotElement("Super_Intake");
                    selectTreeNodeAndCheck(@"robotBuildDefinition\robots\Robot #2\mechanismInstances\mechanismInstanceName_1\name (mechanismInstanceName_1)");

                    setTextInput("frontIntake");
                    selectTreeNodeAndCheck(@"robotBuildDefinition\robots\Robot #2\mechanismInstances\frontIntake\name (frontIntake)");

                    clickSave();
                }

                [TestMethod]
                public void TestMethod_00009_AddClosedLoopControlParametersToSuper_Intake()
                {
                    selectTreeNodeAndCheck(@"robotBuildDefinition\mechanisms\Super_Intake");
                    addRobotElement("closedLoopControlParameters");
                    selectTreeNodeAndCheck(@"robotBuildDefinition\mechanisms\Super_Intake\closedLoopControlParameters\closedLoopControlParameters_1\name (closedLoopControlParameters_1)");

                    setTextInput("intakePID");
                    selectTreeNodeAndCheck(@"robotBuildDefinition\mechanisms\Super_Intake\closedLoopControlParameters\intakePID\name (intakePID)");

                    clickSave();
                }

                [TestMethod]
                public void TestMethod_0000A_SetPGainTo2()
                {
                    selectTreeNodeAndCheck(@"robotBuildDefinition\mechanisms\Super_Intake\closedLoopControlParameters\intakePID\pGain (0)");
                    setNumericUpDown(2);
                    selectTreeNodeAndCheck(@"robotBuildDefinition\mechanisms\Super_Intake\closedLoopControlParameters\intakePID\pGain (2)");

                    clickSave();
                }
        */

        [TestCleanup]
        public void AfterEveryTest()
        {
            Thread.Sleep(TimeSpan.FromSeconds(pauseAtEndOfEachTest)); // to give us time to see what is going on 
        }

        #region ====================================== Helper functions ================================================

        void setAndCheckDouble(string basePath, double previousValue, double value, string units, double min, double max)
        {
            if (string.IsNullOrEmpty(units))
                selectTreeNodeAndCheck(basePath.Replace("##value##", string.Format("{0}", previousValue.ToString())));
            else
                selectTreeNodeAndCheck(basePath.Replace("##value##", string.Format("{0} {1}", previousValue.ToString(), units)));

            setNumericUpDown(value);
            value = value < min ? min : value;
            value = value > max ? max : value;
            if (string.IsNullOrEmpty(units))
                selectTreeNodeAndCheck(basePath.Replace("##value##", string.Format("{0}", value.ToString())));
            else
                selectTreeNodeAndCheck(basePath.Replace("##value##", string.Format("{0} {1}", value.ToString(), units)));
        }


        private void selectInComboBox(string value)
        {
            List<string> names = getListOfComboBoxStrings();
            int index = names.IndexOf(value);
            Assert.IsTrue(index >= 0);
            Session.FindElementByAccessibilityId("infoIOtextBox").SendKeys("ComboBox:" + index.ToString());

            Session.FindElementByAccessibilityId("checkCheckBoxListItemButton").Click();
        }
        private void setNumericUpDown(double value)
        {
            WindowsElement we = Session.FindElementByAccessibilityId("valueNumericUpDown");
            we.Click();
            we.SendKeys(Keys.Control + "a");
            we.SendKeys(Keys.Delete);
            we.SendKeys(value.ToString());
        }

        private void setTextInput(string value)
        {
            WindowsElement we = Session.FindElementByAccessibilityId("valueTextBox");
            we.Click();
            we.Clear();
            we.SendKeys(value);
        }

        private void clickSave()
        {
            Session.FindElementByAccessibilityId("saveConfigBbutton").Click();
        }

        /// <summary>
        /// click on something else so that when we click on the item that we want to check next, the path textbox is updated (since the onSelect callback will fire)
        /// </summary>
        private void clickOnSomethingElse()
        {
            selectTreeNode(@"Robot Variant");
        }

        private void selectTreeNodeAndCheck(string path)
        {
            selectTreeNode(path);
            Thread.Sleep(TimeSpan.FromSeconds(0.5));
            Assert.AreEqual(path, getSelectedTreeNodeFullPath());
        }
        private void selectTreeNode(string path)
        {
            Session.FindElementByAccessibilityId("infoIOtextBox").Click();
            Session.FindElementByAccessibilityId("infoIOtextBox").Clear();
            Session.FindElementByAccessibilityId("infoIOtextBox").SendKeys(path);
            Session.FindElementByAccessibilityId("selectNodeButton").Click();
        }
        private string getSelectedTreeNodeFullPath()
        {
            Session.FindElementByAccessibilityId("getSelectedTreeElementPathButton").Click();
            Thread.Sleep(TimeSpan.FromSeconds(0.5));
            return Session.FindElementByAccessibilityId("infoIOtextBox").Text;
        }
        private List<string> getListOfComboBoxStrings()
        {
            List<string> names = new List<string>();

            Session.FindElementByAccessibilityId("infoIOtextBox").SendKeys("ComboBox");
            Session.FindElementByAccessibilityId("getCheckBoxListItemsButton").Click();
            string items = Session.FindElementByAccessibilityId("infoIOtextBox").Text;
            names = items.Trim('#').Split('#').ToList();
            return names;
        }
        private List<string> getListOfAvailableRobotElements()
        {
            List<string> names = new List<string>();

            Session.FindElementByAccessibilityId("infoIOtextBox").SendKeys("CheckListBox");
            Session.FindElementByAccessibilityId("getCheckBoxListItemsButton").Click();
            string items = Session.FindElementByAccessibilityId("infoIOtextBox").Text;
            names = items.Trim('#').Split('#').ToList();
            return names;
        }

        private void checkmarkRobotElement(string name)
        {
            List<string> names = getListOfAvailableRobotElements();
            int index = names.IndexOf(name);
            Assert.IsTrue(index >= 0);
            Session.FindElementByAccessibilityId("infoIOtextBox").SendKeys("CheckListBox:" + index.ToString());

            Session.FindElementByAccessibilityId("checkCheckBoxListItemButton").Click();
        }

        private void addRobotElement(string name)
        {
            if (name != "")
                checkmarkRobotElement(name);

            Session.FindElementByAccessibilityId("addTreeElementButton").Click();
        }

        #endregion
    }
}
