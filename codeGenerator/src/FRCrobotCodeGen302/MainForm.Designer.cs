
namespace FRCrobotCodeGen302
{
    partial class MainForm
    {
        /// <summary>
        /// Required designer variable.
        /// </summary>
        private System.ComponentModel.IContainer components = null;

        /// <summary>
        /// Clean up any resources being used.
        /// </summary>
        /// <param name="disposing">true if managed resources should be disposed; otherwise, false.</param>
        protected override void Dispose(bool disposing)
        {
            if (disposing && (components != null))
            {
                components.Dispose();
            }
            base.Dispose(disposing);
        }

        #region Windows Form Designer generated code

        /// <summary>
        /// Required method for Designer support - do not modify
        /// the contents of this method with the code editor.
        /// </summary>
        private void InitializeComponent()
        {
            this.components = new System.ComponentModel.Container();
            System.ComponentModel.ComponentResourceManager resources = new System.ComponentModel.ComponentResourceManager(typeof(MainForm));
            this.generateButton = new System.Windows.Forms.Button();
            this.cleanButton = new System.Windows.Forms.Button();
            this.configurationBrowseButton = new System.Windows.Forms.Button();
            this.configurationFilePathNameTextBox = new System.Windows.Forms.TextBox();
            this.outputFolderLabel = new System.Windows.Forms.Label();
            this.configuredOutputFolderLabel = new System.Windows.Forms.Label();
            this.label1 = new System.Windows.Forms.Label();
            this.label2 = new System.Windows.Forms.Label();
            this.configurationGroupBox = new System.Windows.Forms.GroupBox();
            this.robotConfigurationFileComboBox = new System.Windows.Forms.ComboBox();
            this.createNewRobotVariantsConfigButton = new System.Windows.Forms.Button();
            this.progressTextBox = new System.Windows.Forms.TextBox();
            this.theTabControl = new System.Windows.Forms.TabControl();
            this.tabMainPage = new System.Windows.Forms.TabPage();
            this.clearReportButton = new System.Windows.Forms.Button();
            this.tabConfigurationPage = new System.Windows.Forms.TabPage();
            this.splitContainer1 = new System.Windows.Forms.SplitContainer();
            this.robotTreeView = new System.Windows.Forms.TreeView();
            this.panel1 = new System.Windows.Forms.Panel();
            this.tuningButton = new System.Windows.Forms.Button();
            this.selectNodeButton = new System.Windows.Forms.Button();
            this.getSelectedTreeElementPathButton = new System.Windows.Forms.Button();
            this.getCheckBoxListItemsButton = new System.Windows.Forms.Button();
            this.checkCheckBoxListItemButton = new System.Windows.Forms.Button();
            this.infoIOtextBox = new System.Windows.Forms.TextBox();
            this.selectedNodePathTextBox = new System.Windows.Forms.TextBox();
            this.addRobotElementLabel = new System.Windows.Forms.Label();
            this.deleteTreeElementButton = new System.Windows.Forms.Button();
            this.addTreeElementButton = new System.Windows.Forms.Button();
            this.configureStatesButton = new System.Windows.Forms.Button();
            this.saveConfigBbutton = new System.Windows.Forms.Button();
            this.valueTextBox = new System.Windows.Forms.TextBox();
            this.valueDatePicker = new System.Windows.Forms.DateTimePicker();
            this.physicalUnitsComboBox = new System.Windows.Forms.ComboBox();
            this.valueNumericUpDown = new System.Windows.Forms.NumericUpDown();
            this.valueComboBox = new System.Windows.Forms.ComboBox();
            this.robotElementCheckedListBox = new System.Windows.Forms.CheckedListBox();
            this.treeViewIcons = new System.Windows.Forms.ImageList(this.components);
            this.configurationGroupBox.SuspendLayout();
            this.theTabControl.SuspendLayout();
            this.tabMainPage.SuspendLayout();
            this.tabConfigurationPage.SuspendLayout();
            ((System.ComponentModel.ISupportInitialize)(this.splitContainer1)).BeginInit();
            this.splitContainer1.Panel1.SuspendLayout();
            this.splitContainer1.Panel2.SuspendLayout();
            this.splitContainer1.SuspendLayout();
            this.panel1.SuspendLayout();
            ((System.ComponentModel.ISupportInitialize)(this.valueNumericUpDown)).BeginInit();
            this.SuspendLayout();
            // 
            // generateButton
            // 
            this.generateButton.Anchor = ((System.Windows.Forms.AnchorStyles)((System.Windows.Forms.AnchorStyles.Bottom | System.Windows.Forms.AnchorStyles.Right)));
            this.generateButton.Location = new System.Drawing.Point(511, 263);
            this.generateButton.Margin = new System.Windows.Forms.Padding(2, 1, 2, 1);
            this.generateButton.Name = "generateButton";
            this.generateButton.Size = new System.Drawing.Size(70, 32);
            this.generateButton.TabIndex = 0;
            this.generateButton.Text = "Generate";
            this.generateButton.UseVisualStyleBackColor = true;
            this.generateButton.Click += new System.EventHandler(this.generateButton_Click);
            // 
            // cleanButton
            // 
            this.cleanButton.Anchor = ((System.Windows.Forms.AnchorStyles)((System.Windows.Forms.AnchorStyles.Bottom | System.Windows.Forms.AnchorStyles.Right)));
            this.cleanButton.Location = new System.Drawing.Point(432, 263);
            this.cleanButton.Margin = new System.Windows.Forms.Padding(2, 1, 2, 1);
            this.cleanButton.Name = "cleanButton";
            this.cleanButton.Size = new System.Drawing.Size(70, 32);
            this.cleanButton.TabIndex = 0;
            this.cleanButton.Text = "Clean";
            this.cleanButton.UseVisualStyleBackColor = true;
            this.cleanButton.Click += new System.EventHandler(this.cleanButton_Click);
            // 
            // configurationBrowseButton
            // 
            this.configurationBrowseButton.Anchor = ((System.Windows.Forms.AnchorStyles)((System.Windows.Forms.AnchorStyles.Top | System.Windows.Forms.AnchorStyles.Right)));
            this.configurationBrowseButton.Location = new System.Drawing.Point(525, 9);
            this.configurationBrowseButton.Margin = new System.Windows.Forms.Padding(5);
            this.configurationBrowseButton.Name = "configurationBrowseButton";
            this.configurationBrowseButton.Size = new System.Drawing.Size(57, 23);
            this.configurationBrowseButton.TabIndex = 3;
            this.configurationBrowseButton.Text = "Browse";
            this.configurationBrowseButton.UseVisualStyleBackColor = true;
            this.configurationBrowseButton.Click += new System.EventHandler(this.configurationBrowseButton_Click);
            // 
            // configurationFilePathNameTextBox
            // 
            this.configurationFilePathNameTextBox.Anchor = ((System.Windows.Forms.AnchorStyles)(((System.Windows.Forms.AnchorStyles.Top | System.Windows.Forms.AnchorStyles.Left) 
            | System.Windows.Forms.AnchorStyles.Right)));
            this.configurationFilePathNameTextBox.BorderStyle = System.Windows.Forms.BorderStyle.FixedSingle;
            this.configurationFilePathNameTextBox.Font = new System.Drawing.Font("Microsoft Sans Serif", 10F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.configurationFilePathNameTextBox.Location = new System.Drawing.Point(119, 10);
            this.configurationFilePathNameTextBox.Margin = new System.Windows.Forms.Padding(2, 1, 2, 1);
            this.configurationFilePathNameTextBox.Multiline = true;
            this.configurationFilePathNameTextBox.Name = "configurationFilePathNameTextBox";
            this.configurationFilePathNameTextBox.Size = new System.Drawing.Size(397, 22);
            this.configurationFilePathNameTextBox.TabIndex = 4;
            // 
            // outputFolderLabel
            // 
            this.outputFolderLabel.AutoSize = true;
            this.outputFolderLabel.Font = new System.Drawing.Font("Microsoft Sans Serif", 10F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.outputFolderLabel.Location = new System.Drawing.Point(17, 25);
            this.outputFolderLabel.Margin = new System.Windows.Forms.Padding(2, 0, 2, 0);
            this.outputFolderLabel.Name = "outputFolderLabel";
            this.outputFolderLabel.Size = new System.Drawing.Size(91, 17);
            this.outputFolderLabel.TabIndex = 5;
            this.outputFolderLabel.Text = "Output folder";
            // 
            // configuredOutputFolderLabel
            // 
            this.configuredOutputFolderLabel.Anchor = ((System.Windows.Forms.AnchorStyles)(((System.Windows.Forms.AnchorStyles.Top | System.Windows.Forms.AnchorStyles.Left) 
            | System.Windows.Forms.AnchorStyles.Right)));
            this.configuredOutputFolderLabel.BackColor = System.Drawing.SystemColors.Control;
            this.configuredOutputFolderLabel.BorderStyle = System.Windows.Forms.BorderStyle.FixedSingle;
            this.configuredOutputFolderLabel.Font = new System.Drawing.Font("Microsoft Sans Serif", 10F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.configuredOutputFolderLabel.Location = new System.Drawing.Point(109, 25);
            this.configuredOutputFolderLabel.Margin = new System.Windows.Forms.Padding(2, 0, 2, 0);
            this.configuredOutputFolderLabel.Name = "configuredOutputFolderLabel";
            this.configuredOutputFolderLabel.Size = new System.Drawing.Size(455, 20);
            this.configuredOutputFolderLabel.TabIndex = 5;
            // 
            // label1
            // 
            this.label1.AutoSize = true;
            this.label1.Font = new System.Drawing.Font("Microsoft Sans Serif", 10F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label1.Location = new System.Drawing.Point(17, 13);
            this.label1.Margin = new System.Windows.Forms.Padding(2, 0, 2, 0);
            this.label1.Name = "label1";
            this.label1.Size = new System.Drawing.Size(99, 17);
            this.label1.TabIndex = 6;
            this.label1.Text = "Gen config file";
            // 
            // label2
            // 
            this.label2.AutoSize = true;
            this.label2.Font = new System.Drawing.Font("Microsoft Sans Serif", 10F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label2.Location = new System.Drawing.Point(20, 55);
            this.label2.Margin = new System.Windows.Forms.Padding(2, 0, 2, 0);
            this.label2.Name = "label2";
            this.label2.Size = new System.Drawing.Size(88, 17);
            this.label2.TabIndex = 5;
            this.label2.Text = "Robot config";
            // 
            // configurationGroupBox
            // 
            this.configurationGroupBox.Anchor = ((System.Windows.Forms.AnchorStyles)(((System.Windows.Forms.AnchorStyles.Top | System.Windows.Forms.AnchorStyles.Left) 
            | System.Windows.Forms.AnchorStyles.Right)));
            this.configurationGroupBox.Controls.Add(this.robotConfigurationFileComboBox);
            this.configurationGroupBox.Controls.Add(this.configuredOutputFolderLabel);
            this.configurationGroupBox.Controls.Add(this.label2);
            this.configurationGroupBox.Controls.Add(this.outputFolderLabel);
            this.configurationGroupBox.Font = new System.Drawing.Font("Microsoft Sans Serif", 10F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.configurationGroupBox.Location = new System.Drawing.Point(7, 36);
            this.configurationGroupBox.Margin = new System.Windows.Forms.Padding(2, 1, 2, 1);
            this.configurationGroupBox.Name = "configurationGroupBox";
            this.configurationGroupBox.Padding = new System.Windows.Forms.Padding(2, 1, 2, 1);
            this.configurationGroupBox.Size = new System.Drawing.Size(574, 86);
            this.configurationGroupBox.TabIndex = 8;
            this.configurationGroupBox.TabStop = false;
            this.configurationGroupBox.Text = "Configuration";
            // 
            // robotConfigurationFileComboBox
            // 
            this.robotConfigurationFileComboBox.Anchor = ((System.Windows.Forms.AnchorStyles)(((System.Windows.Forms.AnchorStyles.Top | System.Windows.Forms.AnchorStyles.Left) 
            | System.Windows.Forms.AnchorStyles.Right)));
            this.robotConfigurationFileComboBox.DropDownStyle = System.Windows.Forms.ComboBoxStyle.DropDownList;
            this.robotConfigurationFileComboBox.FormattingEnabled = true;
            this.robotConfigurationFileComboBox.Location = new System.Drawing.Point(109, 53);
            this.robotConfigurationFileComboBox.Margin = new System.Windows.Forms.Padding(2, 1, 2, 1);
            this.robotConfigurationFileComboBox.Name = "robotConfigurationFileComboBox";
            this.robotConfigurationFileComboBox.Size = new System.Drawing.Size(456, 24);
            this.robotConfigurationFileComboBox.TabIndex = 8;
            this.robotConfigurationFileComboBox.TextChanged += new System.EventHandler(this.robotConfigurationFileComboBox_TextChanged);
            // 
            // createNewRobotVariantsConfigButton
            // 
            this.createNewRobotVariantsConfigButton.Anchor = ((System.Windows.Forms.AnchorStyles)((System.Windows.Forms.AnchorStyles.Bottom | System.Windows.Forms.AnchorStyles.Left)));
            this.createNewRobotVariantsConfigButton.Location = new System.Drawing.Point(150, 263);
            this.createNewRobotVariantsConfigButton.Margin = new System.Windows.Forms.Padding(2, 1, 2, 1);
            this.createNewRobotVariantsConfigButton.Name = "createNewRobotVariantsConfigButton";
            this.createNewRobotVariantsConfigButton.Size = new System.Drawing.Size(220, 32);
            this.createNewRobotVariantsConfigButton.TabIndex = 3;
            this.createNewRobotVariantsConfigButton.Text = "Create new robot variants configuration";
            this.createNewRobotVariantsConfigButton.UseVisualStyleBackColor = true;
            this.createNewRobotVariantsConfigButton.Click += new System.EventHandler(this.createNewAppDataConfigButton_Click);
            // 
            // progressTextBox
            // 
            this.progressTextBox.Anchor = ((System.Windows.Forms.AnchorStyles)((((System.Windows.Forms.AnchorStyles.Top | System.Windows.Forms.AnchorStyles.Bottom) 
            | System.Windows.Forms.AnchorStyles.Left) 
            | System.Windows.Forms.AnchorStyles.Right)));
            this.progressTextBox.Location = new System.Drawing.Point(7, 125);
            this.progressTextBox.Margin = new System.Windows.Forms.Padding(2, 1, 2, 1);
            this.progressTextBox.Multiline = true;
            this.progressTextBox.Name = "progressTextBox";
            this.progressTextBox.ReadOnly = true;
            this.progressTextBox.ScrollBars = System.Windows.Forms.ScrollBars.Both;
            this.progressTextBox.Size = new System.Drawing.Size(575, 135);
            this.progressTextBox.TabIndex = 9;
            // 
            // theTabControl
            // 
            this.theTabControl.Controls.Add(this.tabMainPage);
            this.theTabControl.Controls.Add(this.tabConfigurationPage);
            this.theTabControl.Dock = System.Windows.Forms.DockStyle.Fill;
            this.theTabControl.Location = new System.Drawing.Point(0, 0);
            this.theTabControl.Margin = new System.Windows.Forms.Padding(2, 1, 2, 1);
            this.theTabControl.Name = "theTabControl";
            this.theTabControl.SelectedIndex = 0;
            this.theTabControl.Size = new System.Drawing.Size(591, 321);
            this.theTabControl.TabIndex = 10;
            // 
            // tabMainPage
            // 
            this.tabMainPage.Controls.Add(this.clearReportButton);
            this.tabMainPage.Controls.Add(this.createNewRobotVariantsConfigButton);
            this.tabMainPage.Controls.Add(this.label1);
            this.tabMainPage.Controls.Add(this.generateButton);
            this.tabMainPage.Controls.Add(this.cleanButton);
            this.tabMainPage.Controls.Add(this.configurationFilePathNameTextBox);
            this.tabMainPage.Controls.Add(this.progressTextBox);
            this.tabMainPage.Controls.Add(this.configurationBrowseButton);
            this.tabMainPage.Controls.Add(this.configurationGroupBox);
            this.tabMainPage.Location = new System.Drawing.Point(4, 22);
            this.tabMainPage.Margin = new System.Windows.Forms.Padding(2, 1, 2, 1);
            this.tabMainPage.Name = "tabMainPage";
            this.tabMainPage.Padding = new System.Windows.Forms.Padding(2, 1, 2, 1);
            this.tabMainPage.Size = new System.Drawing.Size(583, 295);
            this.tabMainPage.TabIndex = 0;
            this.tabMainPage.Text = "Main";
            this.tabMainPage.UseVisualStyleBackColor = true;
            // 
            // clearReportButton
            // 
            this.clearReportButton.Anchor = ((System.Windows.Forms.AnchorStyles)((System.Windows.Forms.AnchorStyles.Bottom | System.Windows.Forms.AnchorStyles.Left)));
            this.clearReportButton.Location = new System.Drawing.Point(7, 263);
            this.clearReportButton.Margin = new System.Windows.Forms.Padding(2, 1, 2, 1);
            this.clearReportButton.Name = "clearReportButton";
            this.clearReportButton.Size = new System.Drawing.Size(123, 32);
            this.clearReportButton.TabIndex = 10;
            this.clearReportButton.Text = "Clear report window";
            this.clearReportButton.UseVisualStyleBackColor = true;
            this.clearReportButton.Click += new System.EventHandler(this.clearReportButton_Click);
            // 
            // tabConfigurationPage
            // 
            this.tabConfigurationPage.Controls.Add(this.splitContainer1);
            this.tabConfigurationPage.Location = new System.Drawing.Point(4, 22);
            this.tabConfigurationPage.Margin = new System.Windows.Forms.Padding(2, 1, 2, 1);
            this.tabConfigurationPage.Name = "tabConfigurationPage";
            this.tabConfigurationPage.Padding = new System.Windows.Forms.Padding(2, 1, 2, 1);
            this.tabConfigurationPage.Size = new System.Drawing.Size(583, 295);
            this.tabConfigurationPage.TabIndex = 1;
            this.tabConfigurationPage.Text = "Configuration";
            this.tabConfigurationPage.UseVisualStyleBackColor = true;
            // 
            // splitContainer1
            // 
            this.splitContainer1.Dock = System.Windows.Forms.DockStyle.Fill;
            this.splitContainer1.Location = new System.Drawing.Point(2, 1);
            this.splitContainer1.Margin = new System.Windows.Forms.Padding(2, 1, 2, 1);
            this.splitContainer1.Name = "splitContainer1";
            // 
            // splitContainer1.Panel1
            // 
            this.splitContainer1.Panel1.Controls.Add(this.robotTreeView);
            // 
            // splitContainer1.Panel2
            // 
            this.splitContainer1.Panel2.Controls.Add(this.panel1);
            this.splitContainer1.Panel2MinSize = 180;
            this.splitContainer1.Size = new System.Drawing.Size(579, 293);
            this.splitContainer1.SplitterDistance = 188;
            this.splitContainer1.SplitterWidth = 3;
            this.splitContainer1.TabIndex = 1;
            // 
            // robotTreeView
            // 
            this.robotTreeView.Dock = System.Windows.Forms.DockStyle.Fill;
            this.robotTreeView.HideSelection = false;
            this.robotTreeView.Location = new System.Drawing.Point(0, 0);
            this.robotTreeView.Margin = new System.Windows.Forms.Padding(2, 1, 2, 1);
            this.robotTreeView.Name = "robotTreeView";
            this.robotTreeView.Size = new System.Drawing.Size(188, 293);
            this.robotTreeView.TabIndex = 0;
            this.robotTreeView.AfterSelect += new System.Windows.Forms.TreeViewEventHandler(this.robotTreeView_AfterSelect);
            // 
            // panel1
            // 
            this.panel1.Controls.Add(this.tuningButton);
            this.panel1.Controls.Add(this.selectNodeButton);
            this.panel1.Controls.Add(this.getSelectedTreeElementPathButton);
            this.panel1.Controls.Add(this.getCheckBoxListItemsButton);
            this.panel1.Controls.Add(this.checkCheckBoxListItemButton);
            this.panel1.Controls.Add(this.infoIOtextBox);
            this.panel1.Controls.Add(this.selectedNodePathTextBox);
            this.panel1.Controls.Add(this.addRobotElementLabel);
            this.panel1.Controls.Add(this.deleteTreeElementButton);
            this.panel1.Controls.Add(this.addTreeElementButton);
            this.panel1.Controls.Add(this.configureStatesButton);
            this.panel1.Controls.Add(this.saveConfigBbutton);
            this.panel1.Controls.Add(this.valueTextBox);
            this.panel1.Controls.Add(this.valueDatePicker);
            this.panel1.Controls.Add(this.physicalUnitsComboBox);
            this.panel1.Controls.Add(this.valueNumericUpDown);
            this.panel1.Controls.Add(this.valueComboBox);
            this.panel1.Controls.Add(this.robotElementCheckedListBox);
            this.panel1.Dock = System.Windows.Forms.DockStyle.Fill;
            this.panel1.Location = new System.Drawing.Point(0, 0);
            this.panel1.Margin = new System.Windows.Forms.Padding(2, 1, 2, 1);
            this.panel1.Name = "panel1";
            this.panel1.Size = new System.Drawing.Size(388, 293);
            this.panel1.TabIndex = 2;
            // 
            // tuningButton
            // 
            this.tuningButton.Anchor = ((System.Windows.Forms.AnchorStyles)((System.Windows.Forms.AnchorStyles.Bottom | System.Windows.Forms.AnchorStyles.Right)));
            this.tuningButton.BackColor = System.Drawing.Color.IndianRed;
            this.tuningButton.Location = new System.Drawing.Point(231, 266);
            this.tuningButton.Name = "tuningButton";
            this.tuningButton.Size = new System.Drawing.Size(84, 23);
            this.tuningButton.TabIndex = 13;
            this.tuningButton.Text = "Toggle Tuning";
            this.tuningButton.UseVisualStyleBackColor = false;
            this.tuningButton.Click += new System.EventHandler(this.tuningButton_Click);
            // 
            // selectNodeButton
            // 
            this.selectNodeButton.Anchor = ((System.Windows.Forms.AnchorStyles)((System.Windows.Forms.AnchorStyles.Bottom | System.Windows.Forms.AnchorStyles.Right)));
            this.selectNodeButton.Enabled = false;
            this.selectNodeButton.Location = new System.Drawing.Point(302, 164);
            this.selectNodeButton.Name = "selectNodeButton";
            this.selectNodeButton.Size = new System.Drawing.Size(82, 26);
            this.selectNodeButton.TabIndex = 10;
            this.selectNodeButton.Text = "Select Node";
            this.selectNodeButton.UseVisualStyleBackColor = true;
            this.selectNodeButton.Visible = false;
            this.selectNodeButton.Click += new System.EventHandler(this.selectNodeButton_Click);
            // 
            // getSelectedTreeElementPathButton
            // 
            this.getSelectedTreeElementPathButton.Anchor = ((System.Windows.Forms.AnchorStyles)((System.Windows.Forms.AnchorStyles.Bottom | System.Windows.Forms.AnchorStyles.Right)));
            this.getSelectedTreeElementPathButton.Enabled = false;
            this.getSelectedTreeElementPathButton.Location = new System.Drawing.Point(302, 90);
            this.getSelectedTreeElementPathButton.Name = "getSelectedTreeElementPathButton";
            this.getSelectedTreeElementPathButton.Size = new System.Drawing.Size(82, 26);
            this.getSelectedTreeElementPathButton.TabIndex = 10;
            this.getSelectedTreeElementPathButton.Text = "Get Selected node path";
            this.getSelectedTreeElementPathButton.UseVisualStyleBackColor = true;
            this.getSelectedTreeElementPathButton.Visible = false;
            this.getSelectedTreeElementPathButton.Click += new System.EventHandler(this.getSelectedTreeElementPathButton_Click);
            // 
            // getCheckBoxListItemsButton
            // 
            this.getCheckBoxListItemsButton.Anchor = ((System.Windows.Forms.AnchorStyles)((System.Windows.Forms.AnchorStyles.Bottom | System.Windows.Forms.AnchorStyles.Right)));
            this.getCheckBoxListItemsButton.Enabled = false;
            this.getCheckBoxListItemsButton.Location = new System.Drawing.Point(302, 140);
            this.getCheckBoxListItemsButton.Name = "getCheckBoxListItemsButton";
            this.getCheckBoxListItemsButton.Size = new System.Drawing.Size(82, 26);
            this.getCheckBoxListItemsButton.TabIndex = 10;
            this.getCheckBoxListItemsButton.Text = "CheckList Items";
            this.getCheckBoxListItemsButton.UseVisualStyleBackColor = true;
            this.getCheckBoxListItemsButton.Visible = false;
            this.getCheckBoxListItemsButton.Click += new System.EventHandler(this.getCheckBoxListItemsButton_Click);
            // 
            // checkCheckBoxListItemButton
            // 
            this.checkCheckBoxListItemButton.Anchor = ((System.Windows.Forms.AnchorStyles)((System.Windows.Forms.AnchorStyles.Bottom | System.Windows.Forms.AnchorStyles.Right)));
            this.checkCheckBoxListItemButton.Enabled = false;
            this.checkCheckBoxListItemButton.Location = new System.Drawing.Point(302, 110);
            this.checkCheckBoxListItemButton.Name = "checkCheckBoxListItemButton";
            this.checkCheckBoxListItemButton.Size = new System.Drawing.Size(82, 26);
            this.checkCheckBoxListItemButton.TabIndex = 10;
            this.checkCheckBoxListItemButton.Text = "Check Item";
            this.checkCheckBoxListItemButton.UseVisualStyleBackColor = true;
            this.checkCheckBoxListItemButton.Visible = false;
            this.checkCheckBoxListItemButton.Click += new System.EventHandler(this.checkCheckBoxListItemButton_Click);
            // 
            // infoIOtextBox
            // 
            this.infoIOtextBox.Anchor = ((System.Windows.Forms.AnchorStyles)(((System.Windows.Forms.AnchorStyles.Bottom | System.Windows.Forms.AnchorStyles.Left) 
            | System.Windows.Forms.AnchorStyles.Right)));
            this.infoIOtextBox.Enabled = false;
            this.infoIOtextBox.Location = new System.Drawing.Point(3, 167);
            this.infoIOtextBox.Margin = new System.Windows.Forms.Padding(3, 2, 3, 2);
            this.infoIOtextBox.Name = "infoIOtextBox";
            this.infoIOtextBox.Size = new System.Drawing.Size(300, 20);
            this.infoIOtextBox.TabIndex = 9;
            this.infoIOtextBox.Visible = false;
            // 
            // selectedNodePathTextBox
            // 
            this.selectedNodePathTextBox.Anchor = ((System.Windows.Forms.AnchorStyles)(((System.Windows.Forms.AnchorStyles.Bottom | System.Windows.Forms.AnchorStyles.Left) 
            | System.Windows.Forms.AnchorStyles.Right)));
            this.selectedNodePathTextBox.Enabled = false;
            this.selectedNodePathTextBox.Location = new System.Drawing.Point(3, 138);
            this.selectedNodePathTextBox.Margin = new System.Windows.Forms.Padding(3, 2, 3, 2);
            this.selectedNodePathTextBox.Name = "selectedNodePathTextBox";
            this.selectedNodePathTextBox.Size = new System.Drawing.Size(300, 20);
            this.selectedNodePathTextBox.TabIndex = 9;
            this.selectedNodePathTextBox.Visible = false;
            // 
            // addRobotElementLabel
            // 
            this.addRobotElementLabel.AutoSize = true;
            this.addRobotElementLabel.Location = new System.Drawing.Point(3, 4);
            this.addRobotElementLabel.Name = "addRobotElementLabel";
            this.addRobotElementLabel.Size = new System.Drawing.Size(97, 13);
            this.addRobotElementLabel.TabIndex = 8;
            this.addRobotElementLabel.Text = "Select items to add";
            this.addRobotElementLabel.Visible = false;
            // 
            // deleteTreeElementButton
            // 
            this.deleteTreeElementButton.Anchor = ((System.Windows.Forms.AnchorStyles)((System.Windows.Forms.AnchorStyles.Bottom | System.Windows.Forms.AnchorStyles.Right)));
            this.deleteTreeElementButton.Location = new System.Drawing.Point(231, 238);
            this.deleteTreeElementButton.Margin = new System.Windows.Forms.Padding(2, 1, 2, 1);
            this.deleteTreeElementButton.Name = "deleteTreeElementButton";
            this.deleteTreeElementButton.Size = new System.Drawing.Size(153, 25);
            this.deleteTreeElementButton.TabIndex = 6;
            this.deleteTreeElementButton.Text = "Delete";
            this.deleteTreeElementButton.UseVisualStyleBackColor = true;
            this.deleteTreeElementButton.Click += new System.EventHandler(this.deleteTreeElementButton_Click);
            // 
            // addTreeElementButton
            // 
            this.addTreeElementButton.Anchor = ((System.Windows.Forms.AnchorStyles)((System.Windows.Forms.AnchorStyles.Bottom | System.Windows.Forms.AnchorStyles.Right)));
            this.addTreeElementButton.Enabled = false;
            this.addTreeElementButton.Location = new System.Drawing.Point(231, 212);
            this.addTreeElementButton.Margin = new System.Windows.Forms.Padding(2, 1, 2, 1);
            this.addTreeElementButton.Name = "addTreeElementButton";
            this.addTreeElementButton.Size = new System.Drawing.Size(153, 25);
            this.addTreeElementButton.TabIndex = 4;
            this.addTreeElementButton.Text = "Add";
            this.addTreeElementButton.UseVisualStyleBackColor = true;
            this.addTreeElementButton.Click += new System.EventHandler(this.addTreeElementButton_Click);
            // 
            // configureStatesButton
            // 
            this.configureStatesButton.Anchor = ((System.Windows.Forms.AnchorStyles)((System.Windows.Forms.AnchorStyles.Bottom | System.Windows.Forms.AnchorStyles.Right)));
            this.configureStatesButton.Location = new System.Drawing.Point(231, 160);
            this.configureStatesButton.Name = "configureStatesButton";
            this.configureStatesButton.Size = new System.Drawing.Size(153, 25);
            this.configureStatesButton.TabIndex = 14;
            this.configureStatesButton.Text = "Configure states";
            this.configureStatesButton.UseVisualStyleBackColor = true;
            this.configureStatesButton.Click += new System.EventHandler(this.configureStatesButton_Click);
            // 
            // saveConfigBbutton
            // 
            this.saveConfigBbutton.Anchor = ((System.Windows.Forms.AnchorStyles)((System.Windows.Forms.AnchorStyles.Bottom | System.Windows.Forms.AnchorStyles.Right)));
            this.saveConfigBbutton.Location = new System.Drawing.Point(315, 265);
            this.saveConfigBbutton.Margin = new System.Windows.Forms.Padding(2, 1, 2, 1);
            this.saveConfigBbutton.Name = "saveConfigBbutton";
            this.saveConfigBbutton.Size = new System.Drawing.Size(69, 25);
            this.saveConfigBbutton.TabIndex = 2;
            this.saveConfigBbutton.Text = "Save";
            this.saveConfigBbutton.UseVisualStyleBackColor = true;
            this.saveConfigBbutton.Click += new System.EventHandler(this.saveConfigBbutton_Click);
            // 
            // valueTextBox
            // 
            this.valueTextBox.Anchor = ((System.Windows.Forms.AnchorStyles)((System.Windows.Forms.AnchorStyles.Bottom | System.Windows.Forms.AnchorStyles.Right)));
            this.valueTextBox.Location = new System.Drawing.Point(74, 193);
            this.valueTextBox.Margin = new System.Windows.Forms.Padding(2, 1, 2, 1);
            this.valueTextBox.Name = "valueTextBox";
            this.valueTextBox.Size = new System.Drawing.Size(153, 20);
            this.valueTextBox.TabIndex = 0;
            this.valueTextBox.Visible = false;
            this.valueTextBox.TextChanged += new System.EventHandler(this.valueTextBox_TextChanged);
            // 
            // valueDatePicker
            // 
            this.valueDatePicker.Anchor = ((System.Windows.Forms.AnchorStyles)((System.Windows.Forms.AnchorStyles.Bottom | System.Windows.Forms.AnchorStyles.Right)));
            this.valueDatePicker.Format = System.Windows.Forms.DateTimePickerFormat.Short;
            this.valueDatePicker.Location = new System.Drawing.Point(474, 253);
            this.valueDatePicker.Name = "valueDatePicker";
            this.valueDatePicker.Size = new System.Drawing.Size(153, 20);
            this.valueDatePicker.TabIndex = 0;
            this.valueDatePicker.TextChanged += new System.EventHandler(this.valueDatePicker_ValueChanged);
            // 
            // physicalUnitsComboBox
            // 
            this.physicalUnitsComboBox.Anchor = ((System.Windows.Forms.AnchorStyles)((System.Windows.Forms.AnchorStyles.Bottom | System.Windows.Forms.AnchorStyles.Right)));
            this.physicalUnitsComboBox.DropDownStyle = System.Windows.Forms.ComboBoxStyle.DropDownList;
            this.physicalUnitsComboBox.FormattingEnabled = true;
            this.physicalUnitsComboBox.Location = new System.Drawing.Point(195, 193);
            this.physicalUnitsComboBox.Margin = new System.Windows.Forms.Padding(2, 1, 2, 1);
            this.physicalUnitsComboBox.Name = "physicalUnitsComboBox";
            this.physicalUnitsComboBox.Size = new System.Drawing.Size(50, 20);
            this.physicalUnitsComboBox.TabIndex = 1;
            this.physicalUnitsComboBox.Visible = false;
            this.physicalUnitsComboBox.SelectedValueChanged += new System.EventHandler(this.physicalUnitsComboBox_SelectedValueChanged);
            // 
            // valueNumericUpDown
            // 
            this.valueNumericUpDown.Anchor = ((System.Windows.Forms.AnchorStyles)((System.Windows.Forms.AnchorStyles.Bottom | System.Windows.Forms.AnchorStyles.Right)));
            this.valueNumericUpDown.Location = new System.Drawing.Point(231, 190);
            this.valueNumericUpDown.Margin = new System.Windows.Forms.Padding(2, 1, 2, 1);
            this.valueNumericUpDown.Name = "valueNumericUpDown";
            this.valueNumericUpDown.Size = new System.Drawing.Size(153, 20);
            this.valueNumericUpDown.TabIndex = 3;
            this.valueNumericUpDown.Visible = false;
            this.valueNumericUpDown.ValueChanged += new System.EventHandler(this.valueNumericUpDown_ValueChanged);
            // 
            // valueComboBox
            // 
            this.valueComboBox.Anchor = ((System.Windows.Forms.AnchorStyles)((System.Windows.Forms.AnchorStyles.Bottom | System.Windows.Forms.AnchorStyles.Right)));
            this.valueComboBox.DropDownStyle = System.Windows.Forms.ComboBoxStyle.DropDownList;
            this.valueComboBox.FormattingEnabled = true;
            this.valueComboBox.Location = new System.Drawing.Point(74, 215);
            this.valueComboBox.Margin = new System.Windows.Forms.Padding(2, 1, 2, 1);
            this.valueComboBox.Name = "valueComboBox";
            this.valueComboBox.Size = new System.Drawing.Size(153, 21);
            this.valueComboBox.TabIndex = 1;
            this.valueComboBox.Visible = false;
            this.valueComboBox.SelectedValueChanged += new System.EventHandler(this.valueComboBox_SelectedValueChanged);
            // 
            // robotElementCheckedListBox
            // 
            this.robotElementCheckedListBox.Anchor = ((System.Windows.Forms.AnchorStyles)((((System.Windows.Forms.AnchorStyles.Top | System.Windows.Forms.AnchorStyles.Bottom) 
            | System.Windows.Forms.AnchorStyles.Left) 
            | System.Windows.Forms.AnchorStyles.Right)));
            this.robotElementCheckedListBox.FormattingEnabled = true;
            this.robotElementCheckedListBox.Location = new System.Drawing.Point(2, 24);
            this.robotElementCheckedListBox.Margin = new System.Windows.Forms.Padding(2);
            this.robotElementCheckedListBox.Name = "robotElementCheckedListBox";
            this.robotElementCheckedListBox.Size = new System.Drawing.Size(382, 139);
            this.robotElementCheckedListBox.TabIndex = 7;
            this.robotElementCheckedListBox.Visible = false;
            // 
            // treeViewIcons
            // 
            this.treeViewIcons.ImageStream = ((System.Windows.Forms.ImageListStreamer)(resources.GetObject("treeViewIcons.ImageStream")));
            this.treeViewIcons.TransparentColor = System.Drawing.Color.Transparent;
            this.treeViewIcons.Images.SetKeyName(0, "lock.ico");
            this.treeViewIcons.Images.SetKeyName(1, "lock-unlock.ico");
            this.treeViewIcons.Images.SetKeyName(2, "gear.ico");
            this.treeViewIcons.Images.SetKeyName(3, "wrench.ico");
            this.treeViewIcons.Images.SetKeyName(4, "lock-unlock-instance.ico");
            // 
            // MainForm
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.ClientSize = new System.Drawing.Size(591, 321);
            this.Controls.Add(this.theTabControl);
            this.Icon = ((System.Drawing.Icon)(resources.GetObject("$this.Icon")));
            this.Margin = new System.Windows.Forms.Padding(2, 1, 2, 1);
            this.Name = "MainForm";
            this.Text = "Team 302 code generator";
            this.FormClosing += new System.Windows.Forms.FormClosingEventHandler(this.MainForm_FormClosing);
            this.configurationGroupBox.ResumeLayout(false);
            this.configurationGroupBox.PerformLayout();
            this.theTabControl.ResumeLayout(false);
            this.tabMainPage.ResumeLayout(false);
            this.tabMainPage.PerformLayout();
            this.tabConfigurationPage.ResumeLayout(false);
            this.splitContainer1.Panel1.ResumeLayout(false);
            this.splitContainer1.Panel2.ResumeLayout(false);
            ((System.ComponentModel.ISupportInitialize)(this.splitContainer1)).EndInit();
            this.splitContainer1.ResumeLayout(false);
            this.panel1.ResumeLayout(false);
            this.panel1.PerformLayout();
            ((System.ComponentModel.ISupportInitialize)(this.valueNumericUpDown)).EndInit();
            this.ResumeLayout(false);

        }

        #endregion

        private System.Windows.Forms.Button generateButton;
        private System.Windows.Forms.Button cleanButton;
        private System.Windows.Forms.Button configurationBrowseButton;
        private System.Windows.Forms.Button createNewRobotVariantsConfigButton;
        private System.Windows.Forms.TextBox configurationFilePathNameTextBox;
        private System.Windows.Forms.Label outputFolderLabel;
        private System.Windows.Forms.Label configuredOutputFolderLabel;
        private System.Windows.Forms.Label label1;
        private System.Windows.Forms.Label label2;
        private System.Windows.Forms.GroupBox configurationGroupBox;
        private System.Windows.Forms.TextBox progressTextBox;
        private System.Windows.Forms.TabControl theTabControl;
        private System.Windows.Forms.TabPage tabMainPage;
        private System.Windows.Forms.TabPage tabConfigurationPage;
        private System.Windows.Forms.TreeView robotTreeView;
        private System.Windows.Forms.SplitContainer splitContainer1;
        private System.Windows.Forms.ComboBox valueComboBox;
        private System.Windows.Forms.TextBox valueTextBox;
        private System.Windows.Forms.ComboBox physicalUnitsComboBox;
        private System.Windows.Forms.Panel panel1;
        private System.Windows.Forms.Button saveConfigBbutton;
        private System.Windows.Forms.NumericUpDown valueNumericUpDown;
        private System.Windows.Forms.Button addTreeElementButton;
        private System.Windows.Forms.Button configureStatesButton;
        private System.Windows.Forms.ComboBox robotConfigurationFileComboBox;
        private System.Windows.Forms.Button clearReportButton;
        private System.Windows.Forms.Button deleteTreeElementButton;
        private System.Windows.Forms.CheckedListBox robotElementCheckedListBox;
        private System.Windows.Forms.Label addRobotElementLabel;
        private System.Windows.Forms.ImageList treeViewIcons;
        private System.Windows.Forms.Button selectNodeButton;
        private System.Windows.Forms.Button getSelectedTreeElementPathButton;
        private System.Windows.Forms.Button getCheckBoxListItemsButton;
        private System.Windows.Forms.Button checkCheckBoxListItemButton;
        private System.Windows.Forms.TextBox infoIOtextBox;
        private System.Windows.Forms.TextBox selectedNodePathTextBox;
        private System.Windows.Forms.DateTimePicker valueDatePicker;
        private System.Windows.Forms.Button tuningButton;
    }
}

