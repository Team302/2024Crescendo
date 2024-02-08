namespace FtcDataTrace
{
    partial class mainForm
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
            System.Windows.Forms.DataVisualization.Charting.ChartArea chartArea1 = new System.Windows.Forms.DataVisualization.Charting.ChartArea();
            System.Windows.Forms.DataVisualization.Charting.Legend legend1 = new System.Windows.Forms.DataVisualization.Charting.Legend();
            System.Windows.Forms.DataVisualization.Charting.Series series1 = new System.Windows.Forms.DataVisualization.Charting.Series();
            System.ComponentModel.ComponentResourceManager resources = new System.ComponentModel.ComponentResourceManager(typeof(mainForm));
            this.traceUpdateTimer = new System.Windows.Forms.Timer(this.components);
            this.statusTextBox = new System.Windows.Forms.RichTextBox();
            this.chart1 = new System.Windows.Forms.DataVisualization.Charting.Chart();
            this.clearTracesButton = new System.Windows.Forms.Button();
            this.seriesCheckedListBox = new System.Windows.Forms.CheckedListBox();
            this.splitContainer1 = new System.Windows.Forms.SplitContainer();
            this.unCheckAllButton = new System.Windows.Forms.Button();
            this.RecalcAxesButton = new System.Windows.Forms.Button();
            this.parameterEntry = new System.Windows.Forms.TextBox();
            this.parametersTransmitButton = new System.Windows.Forms.Button();
            this.tabControl = new System.Windows.Forms.TabControl();
            this.tracingTabPage = new System.Windows.Forms.TabPage();
            this.ParametersTabPage = new System.Windows.Forms.TabPage();
            this.saveToFileButton = new System.Windows.Forms.Button();
            this.loadFromFileButton = new System.Windows.Forms.Button();
            ((System.ComponentModel.ISupportInitialize)(this.chart1)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.splitContainer1)).BeginInit();
            this.splitContainer1.Panel1.SuspendLayout();
            this.splitContainer1.Panel2.SuspendLayout();
            this.splitContainer1.SuspendLayout();
            this.tabControl.SuspendLayout();
            this.tracingTabPage.SuspendLayout();
            this.ParametersTabPage.SuspendLayout();
            this.SuspendLayout();
            // 
            // traceUpdateTimer
            // 
            this.traceUpdateTimer.Enabled = true;
            this.traceUpdateTimer.Tick += new System.EventHandler(this.timer1_Tick);
            // 
            // statusTextBox
            // 
            this.statusTextBox.Anchor = ((System.Windows.Forms.AnchorStyles)(((System.Windows.Forms.AnchorStyles.Top | System.Windows.Forms.AnchorStyles.Left) 
            | System.Windows.Forms.AnchorStyles.Right)));
            this.statusTextBox.Location = new System.Drawing.Point(8, 446);
            this.statusTextBox.Name = "statusTextBox";
            this.statusTextBox.Size = new System.Drawing.Size(220, 333);
            this.statusTextBox.TabIndex = 2;
            this.statusTextBox.Text = "";
            // 
            // chart1
            // 
            this.chart1.BorderlineColor = System.Drawing.Color.Black;
            this.chart1.BorderlineDashStyle = System.Windows.Forms.DataVisualization.Charting.ChartDashStyle.Solid;
            chartArea1.Name = "ChartArea1";
            this.chart1.ChartAreas.Add(chartArea1);
            this.chart1.Dock = System.Windows.Forms.DockStyle.Fill;
            legend1.Name = "Legend1";
            this.chart1.Legends.Add(legend1);
            this.chart1.Location = new System.Drawing.Point(0, 0);
            this.chart1.Name = "chart1";
            series1.ChartArea = "ChartArea1";
            series1.Legend = "Legend1";
            series1.Name = "Series1";
            this.chart1.Series.Add(series1);
            this.chart1.Size = new System.Drawing.Size(1036, 795);
            this.chart1.TabIndex = 3;
            this.chart1.Text = "chart1";
            this.chart1.MouseEnter += new System.EventHandler(this.Chart1_MouseEnter);
            this.chart1.MouseLeave += new System.EventHandler(this.Chart1_MouseLeave);
            this.chart1.MouseMove += new System.Windows.Forms.MouseEventHandler(this.chart1_MouseMove);
            // 
            // clearTracesButton
            // 
            this.clearTracesButton.Location = new System.Drawing.Point(12, 5);
            this.clearTracesButton.Margin = new System.Windows.Forms.Padding(4, 5, 4, 5);
            this.clearTracesButton.Name = "clearTracesButton";
            this.clearTracesButton.Size = new System.Drawing.Size(218, 37);
            this.clearTracesButton.TabIndex = 5;
            this.clearTracesButton.Text = "Clear Traces";
            this.clearTracesButton.UseVisualStyleBackColor = true;
            this.clearTracesButton.Click += new System.EventHandler(this.clearTracesButton_Click);
            // 
            // seriesCheckedListBox
            // 
            this.seriesCheckedListBox.Anchor = ((System.Windows.Forms.AnchorStyles)(((System.Windows.Forms.AnchorStyles.Top | System.Windows.Forms.AnchorStyles.Left) 
            | System.Windows.Forms.AnchorStyles.Right)));
            this.seriesCheckedListBox.FormattingEnabled = true;
            this.seriesCheckedListBox.Location = new System.Drawing.Point(12, 137);
            this.seriesCheckedListBox.Name = "seriesCheckedListBox";
            this.seriesCheckedListBox.Size = new System.Drawing.Size(216, 303);
            this.seriesCheckedListBox.TabIndex = 6;
            this.seriesCheckedListBox.ItemCheck += new System.Windows.Forms.ItemCheckEventHandler(this.seriesCheckedListBox_ItemCheck);
            // 
            // splitContainer1
            // 
            this.splitContainer1.Dock = System.Windows.Forms.DockStyle.Fill;
            this.splitContainer1.Location = new System.Drawing.Point(3, 3);
            this.splitContainer1.Name = "splitContainer1";
            // 
            // splitContainer1.Panel1
            // 
            this.splitContainer1.Panel1.Controls.Add(this.unCheckAllButton);
            this.splitContainer1.Panel1.Controls.Add(this.RecalcAxesButton);
            this.splitContainer1.Panel1.Controls.Add(this.clearTracesButton);
            this.splitContainer1.Panel1.Controls.Add(this.statusTextBox);
            this.splitContainer1.Panel1.Controls.Add(this.seriesCheckedListBox);
            // 
            // splitContainer1.Panel2
            // 
            this.splitContainer1.Panel2.Controls.Add(this.chart1);
            this.splitContainer1.Size = new System.Drawing.Size(1270, 795);
            this.splitContainer1.SplitterDistance = 230;
            this.splitContainer1.TabIndex = 7;
            // 
            // unCheckAllButton
            // 
            this.unCheckAllButton.Location = new System.Drawing.Point(12, 95);
            this.unCheckAllButton.Margin = new System.Windows.Forms.Padding(4, 5, 4, 5);
            this.unCheckAllButton.Name = "unCheckAllButton";
            this.unCheckAllButton.Size = new System.Drawing.Size(216, 35);
            this.unCheckAllButton.TabIndex = 8;
            this.unCheckAllButton.Text = "Uncheck all";
            this.unCheckAllButton.UseVisualStyleBackColor = true;
            this.unCheckAllButton.Click += new System.EventHandler(this.unCheckAllButton_Click);
            // 
            // RecalcAxesButton
            // 
            this.RecalcAxesButton.Location = new System.Drawing.Point(12, 49);
            this.RecalcAxesButton.Name = "RecalcAxesButton";
            this.RecalcAxesButton.Size = new System.Drawing.Size(218, 37);
            this.RecalcAxesButton.TabIndex = 7;
            this.RecalcAxesButton.Text = "Recalculate Axes Scale";
            this.RecalcAxesButton.UseVisualStyleBackColor = true;
            this.RecalcAxesButton.Click += new System.EventHandler(this.RecalcAxesButton_Click_2);
            // 
            // parameterEntry
            // 
            this.parameterEntry.Location = new System.Drawing.Point(3, 6);
            this.parameterEntry.Multiline = true;
            this.parameterEntry.Name = "parameterEntry";
            this.parameterEntry.Size = new System.Drawing.Size(1081, 782);
            this.parameterEntry.TabIndex = 8;
            // 
            // parametersTransmitButton
            // 
            this.parametersTransmitButton.Location = new System.Drawing.Point(1092, 9);
            this.parametersTransmitButton.Name = "parametersTransmitButton";
            this.parametersTransmitButton.Size = new System.Drawing.Size(170, 35);
            this.parametersTransmitButton.TabIndex = 9;
            this.parametersTransmitButton.Text = "Transmit";
            this.parametersTransmitButton.UseVisualStyleBackColor = true;
            this.parametersTransmitButton.Click += new System.EventHandler(this.button1_Click_2);
            // 
            // tabControl
            // 
            this.tabControl.Controls.Add(this.tracingTabPage);
            this.tabControl.Controls.Add(this.ParametersTabPage);
            this.tabControl.Dock = System.Windows.Forms.DockStyle.Fill;
            this.tabControl.Location = new System.Drawing.Point(0, 0);
            this.tabControl.Name = "tabControl";
            this.tabControl.SelectedIndex = 0;
            this.tabControl.Size = new System.Drawing.Size(1284, 834);
            this.tabControl.TabIndex = 4;
            // 
            // tracingTabPage
            // 
            this.tracingTabPage.Controls.Add(this.splitContainer1);
            this.tracingTabPage.Location = new System.Drawing.Point(4, 29);
            this.tracingTabPage.Name = "tracingTabPage";
            this.tracingTabPage.Padding = new System.Windows.Forms.Padding(3);
            this.tracingTabPage.Size = new System.Drawing.Size(1276, 801);
            this.tracingTabPage.TabIndex = 0;
            this.tracingTabPage.Text = "Tracing";
            this.tracingTabPage.UseVisualStyleBackColor = true;
            // 
            // ParametersTabPage
            // 
            this.ParametersTabPage.Controls.Add(this.saveToFileButton);
            this.ParametersTabPage.Controls.Add(this.loadFromFileButton);
            this.ParametersTabPage.Controls.Add(this.parametersTransmitButton);
            this.ParametersTabPage.Controls.Add(this.parameterEntry);
            this.ParametersTabPage.Location = new System.Drawing.Point(4, 29);
            this.ParametersTabPage.Name = "ParametersTabPage";
            this.ParametersTabPage.Padding = new System.Windows.Forms.Padding(3);
            this.ParametersTabPage.Size = new System.Drawing.Size(1276, 801);
            this.ParametersTabPage.TabIndex = 1;
            this.ParametersTabPage.Text = "Parameters";
            this.ParametersTabPage.UseVisualStyleBackColor = true;
            // 
            // saveToFileButton
            // 
            this.saveToFileButton.Location = new System.Drawing.Point(1094, 97);
            this.saveToFileButton.Margin = new System.Windows.Forms.Padding(4, 5, 4, 5);
            this.saveToFileButton.Name = "saveToFileButton";
            this.saveToFileButton.Size = new System.Drawing.Size(166, 35);
            this.saveToFileButton.TabIndex = 11;
            this.saveToFileButton.Text = "Save to file";
            this.saveToFileButton.UseVisualStyleBackColor = true;
            this.saveToFileButton.Click += new System.EventHandler(this.saveToFileButton_Click);
            // 
            // loadFromFileButton
            // 
            this.loadFromFileButton.Location = new System.Drawing.Point(1092, 52);
            this.loadFromFileButton.Margin = new System.Windows.Forms.Padding(4, 5, 4, 5);
            this.loadFromFileButton.Name = "loadFromFileButton";
            this.loadFromFileButton.Size = new System.Drawing.Size(170, 35);
            this.loadFromFileButton.TabIndex = 10;
            this.loadFromFileButton.Text = "Load from file";
            this.loadFromFileButton.UseVisualStyleBackColor = true;
            this.loadFromFileButton.Click += new System.EventHandler(this.loadFromFileButton_Click);
            // 
            // mainForm
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(9F, 20F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.ClientSize = new System.Drawing.Size(1284, 834);
            this.Controls.Add(this.tabControl);
            this.Icon = ((System.Drawing.Icon)(resources.GetObject("$this.Icon")));
            this.Name = "mainForm";
            this.Text = "FRC 302 Data Trace";
            this.Load += new System.EventHandler(this.Form1_Load);
            ((System.ComponentModel.ISupportInitialize)(this.chart1)).EndInit();
            this.splitContainer1.Panel1.ResumeLayout(false);
            this.splitContainer1.Panel2.ResumeLayout(false);
            ((System.ComponentModel.ISupportInitialize)(this.splitContainer1)).EndInit();
            this.splitContainer1.ResumeLayout(false);
            this.tabControl.ResumeLayout(false);
            this.tracingTabPage.ResumeLayout(false);
            this.ParametersTabPage.ResumeLayout(false);
            this.ParametersTabPage.PerformLayout();
            this.ResumeLayout(false);

        }

        #endregion
        private System.Windows.Forms.Timer traceUpdateTimer;
        private System.Windows.Forms.RichTextBox statusTextBox;
        private System.Windows.Forms.DataVisualization.Charting.Chart chart1;
        private System.Windows.Forms.Button clearTracesButton;
        private System.Windows.Forms.CheckedListBox seriesCheckedListBox;
        private System.Windows.Forms.SplitContainer splitContainer1;
        private System.Windows.Forms.Button RecalcAxesButton;
        private System.Windows.Forms.Button parametersTransmitButton;
        private System.Windows.Forms.TextBox parameterEntry;
        private System.Windows.Forms.TabControl tabControl;
        private System.Windows.Forms.TabPage tracingTabPage;
        private System.Windows.Forms.TabPage ParametersTabPage;
        private System.Windows.Forms.Button saveToFileButton;
        private System.Windows.Forms.Button loadFromFileButton;
        private System.Windows.Forms.Button unCheckAllButton;
    }
}

