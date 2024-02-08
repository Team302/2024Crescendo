using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;
using System.Net;
using System.Net.Sockets;
using System.Windows.Forms.DataVisualization.Charting;
using System.Threading;
using System.Xml.Serialization;
using System.IO;

namespace FtcDataTrace
{
    public partial class mainForm : Form
    {
        #region Zooming
        private const float CZoomScale = 4f;
        private int FZoomLevel = 0;
        #endregion

        #region Socket
        Socket theSocket;
        Socket listener;
        byte[] bytes = new byte[4096];
        byte[] messageBytes = new byte[4096];
        int port = 30200;
        #endregion

        decoderDataSet decoderInfo = new decoderDataSet();

        public mainForm()
        {
            InitializeComponent();

            //dataTraceGroup dtg = new dataTraceGroup();
            //dtg.name = "wheelPower";
            //dtg.dataItems.Add(new dataTraceItem("FL Power", dataTraceItem.dataType.integerType));
            //dtg.dataItems.Add(new dataTraceItem("FR Power", dataTraceItem.dataType.doubleType));
            //dtg.dataItems.Add(new dataTraceItem("RL Power", dataTraceItem.dataType.doubleType));
            //dtg.dataItems.Add(new dataTraceItem("RR Power", dataTraceItem.dataType.doubleType));
            //dtg.dataItems.Add(new dataTraceItem("Correction", dataTraceItem.dataType.doubleType));

            //decoderInfo.data.Add(dtg);
            //SerializeDataSet(@"decoderInfo.xml");

            try
            {
                loadDecoderDataSet("decoderInfo.xml");
            }
            catch(Exception ex)
            {
                MessageBox.Show(ex.Message);
            }

            //chart1.Series.Add("FL Ang Vel");
            //chart1.Series.Add("FR Ang Vel");
            //chart1.Series.Add("RL Ang Vel");
            //chart1.Series.Add("RR Ang Vel");

            //chart1.Series.Add("FL Encoder");
            //chart1.Series.Add("FR Encoder");
            //chart1.Series.Add("RL Encoder");
            //chart1.Series.Add("RR Encoder");
            //chart1.Series.Add("Target Encoder Cnt");
            //chart1.Series.Add("remainingEncoderCountsAbsolute");
            //chart1.Series.Add("minRemainingEncoderCountsAbsolute");

            //chart1.Series.Add("Actual Angle");
            //chart1.Series.Add("Target Angle");
            //chart1.Series.Add("Error");
            //chart1.Series.Add("DrivingAngle");
            //chart1.Series.Add("GlobalAngle");

            foreach (Series s in chart1.Series)
            {
                s.ChartType = SeriesChartType.Line;
                int index = seriesCheckedListBox.Items.Add(s.Name, true);
            }

            chart1.ChartAreas[0].CursorX.Interval = 0.001;
            chart1.ChartAreas[0].CursorY.Interval = 0.001;

            chart1.ChartAreas[0].AxisX.MajorGrid.LineColor = Color.Gainsboro;
            chart1.ChartAreas[0].AxisY.MajorGrid.LineColor = Color.Gainsboro;
            chart1.ChartAreas[0].AxisX.MajorGrid.LineDashStyle = ChartDashStyle.Dot;
            chart1.ChartAreas[0].AxisY.MajorGrid.LineDashStyle = ChartDashStyle.Dot;

            startListeningForConnection();
        }

        private void loadDecoderDataSet(string filename)
        {
            var mySerializer = new XmlSerializer(typeof(decoderDataSet));
            decoderInfo = (decoderDataSet)mySerializer.Deserialize(new FileStream(filename, FileMode.Open));

            chart1.Series.Clear();

            foreach (dataTraceGroup g in decoderInfo.data)
            {
                foreach (dataTraceItem dti in g.dataItems)
                {
                    chart1.Series.Add(dti.getQualifiedName(g.name));
                }
            }
        }

        private void startListeningForConnection()
        {
            AddTextLineToStatusTextBox("Waiting for robot to connect");
            connectionThread = new Thread(listenForConnection);
            connectionThread.Name = "connectionThread";
            connectionThread.IsBackground = true;
            connectionThread.Start();
        }

        private void listenForConnection()
        {
            try
            {
                IPHostEntry ipHostInfo = Dns.GetHostEntry(Dns.GetHostName());
                IPAddress ipAddress = ipHostInfo.AddressList.ToList().Find(a => a.AddressFamily == AddressFamily.InterNetwork);
                IPEndPoint localEndPoint = new IPEndPoint(ipAddress, port);

                listener = new Socket(SocketType.Stream, ProtocolType.Tcp);
                listener.Bind(localEndPoint);
                listener.Listen(10);

                theSocket = listener.Accept();
                AddTextLineToStatusTextBox("Robot connected");

                receiveMsgs = true;
                msgRxThread = new Thread(receiveMessages);
                msgRxThread.Name = "MessageRxThread";
                msgRxThread.IsBackground = true;
                msgRxThread.Start();
            }
            catch(Exception ex)
            {

            }
        }


        int dollarCount = 0;
        int poundCount = 0;
        StringBuilder theRxMessage = new StringBuilder();
        Queue<String> theRxMessages = new Queue<string>();
        int msgByteWriteIndex = 0;
        double i = 0;
        Thread msgRxThread;
        Thread connectionThread;
        bool receiveMsgs;

        double lastTimestamp = -1;
        void receiveMessages()
        {
            while (true)
            {
                try
                {
                    while (receiveMsgs)
                    {
                        int bytesRec = theSocket.Receive(bytes);
                        if (bytesRec == 0)
                            break;

                        for (int counter = 0; counter < bytesRec; counter++)
                        {
                            byte b = bytes[counter];
                            if (b == '$')
                            {
                                dollarCount++;
                            }
                            else if (b == '#')
                            {
                                poundCount++;
                                if (poundCount == 2)
                                {
                                    if (dollarCount == 2)
                                    {
                                        //the previous message has been fully received
                                        theRxMessages.Enqueue(System.Text.Encoding.ASCII.GetString(messageBytes, 0, msgByteWriteIndex));
                                    }
                                    msgByteWriteIndex = 0;
                                    poundCount = 0;
                                    dollarCount = 0;
                                }
                            }
                            else if (dollarCount == 2)
                            {
                                if (poundCount == 0)
                                {
                                    // theRxMessage.Append();
                                    messageBytes[msgByteWriteIndex++] = b;
                                }
                            }
                            else
                            {
                                dollarCount = 0;
                                // discard the byte
                            }
                        }
                    }
                }
                catch (Exception ex)
                {

                }
                finally
                {
                    receiveMsgs = true;
                    AddTextLineToStatusTextBox("Waiting for robot to connect");
                    theSocket = listener.Accept();
                    AddTextLineToStatusTextBox("Robot connected");
                }
            }
        }
        private void timer1_Tick(object sender, EventArgs e)
        {
            try
            {
                while (theRxMessages.Count > 0)
                {
                    string m = theRxMessages.Dequeue();

                    if (m != null)
                    {
                        string[] sub = m.Split(':');

                        if (sub.Length <= 1)
                        {
                            AddTextLineToStatusTextBox("Message too short");
                        }
                        if (sub.Length > 1) // name and timestamp
                        {
                            int headerLength = 2;
                            double timestamp = Convert.ToDouble(sub[1]);

                            dataTraceGroup dtg = decoderInfo.data.Find(g => g.name == sub[0]);

                            if (dtg == null)
                            {
                                AddTextLineToStatusTextBox("Received an unknown packet : " + sub[0]);
                            }
                            else
                            {
                                int expectedLength = headerLength + dtg.dataItems.Count;
                                if (sub.Length != expectedLength)
                                {
                                    AddTextLineToStatusTextBox(String.Format("Wrong Packet length: {0}, expected : {1} ", sub.Length, expectedLength));
                                }
                                else
                                {
                                    int i = 2;
                                    foreach (dataTraceItem dti in dtg.dataItems)
                                    {
                                        DataPointCollection dpc = chart1.Series[dti.getQualifiedName(dtg.name)].Points;
                                        if (dti.type == dataTraceItem.dataType.integerType)
                                        {
                                            dpc.AddXY(timestamp, Convert.ToInt32(sub[i]));
                                        }
                                        else
                                        {
                                            dpc.AddXY(timestamp, Convert.ToDouble(sub[i]));
                                        }
                                        i++;
                                    }
                                }
                            }

                        }
                    }
                }

            }
            catch (Exception ex)
            { }
        }

        private void button1_Click(object sender, EventArgs e)
        {
            chart1.Series.Clear();
            chart1.Series.Add("Test");
            chart1.Series["Test"].ChartType = SeriesChartType.Line;
            DataPoint Dp = new DataPoint();
            Dp.SetValueY(3);

            for (double i = 0; i < 3.14; i += 0.01)
            {
                double y = Math.Sin(i);
                chart1.Series["Test"].Points.AddXY(i, Math.Sin(i));
            }
        }

        private void clearTracesButton_Click(object sender, EventArgs e)
        {
            foreach (Series s in chart1.Series)
            {
                s.Points.Clear();
            }
        }

        private void Form1_Load(object sender, EventArgs e)
        {
            chart1.ChartAreas[0].AxisX.ScaleView.Zoomable = true;
            chart1.ChartAreas[0].AxisY.ScaleView.Zoomable = true;

            chart1.MouseWheel += Chart1_MouseWheel;
        }

        private void Chart1_MouseEnter(object sender, EventArgs e)
        {
            if (!chart1.Focused)
                chart1.Focus();
        }

        private void Chart1_MouseLeave(object sender, EventArgs e)
        {
            if (chart1.Focused)
                chart1.Parent.Focus();
        }

        private void Chart1_MouseWheel(object sender, MouseEventArgs e)
        {
            try
            {
                Axis theAxis = chart1.ChartAreas[0].AxisX;
                if ((Control.ModifierKeys & Keys.Shift) == Keys.Shift)
                    theAxis = chart1.ChartAreas[0].AxisY;
                
                double xMin = theAxis.ScaleView.ViewMinimum;
                double xMax = theAxis.ScaleView.ViewMaximum;
                double xPixelPos = theAxis.PixelPositionToValue(e.Location.X);

                if (e.Delta < 0 && FZoomLevel > 0)
                {
                    // Scrolled down, meaning zoom out
                    if (--FZoomLevel <= 0)
                    {
                        FZoomLevel = 0;
                        theAxis.ScaleView.ZoomReset();
                    }
                    else
                    {
                        double xStartPos = Math.Max(xPixelPos - (xPixelPos - xMin) * CZoomScale, 0);
                        double xEndPos = Math.Min(xStartPos + (xMax - xMin) * CZoomScale, theAxis.Maximum);
                        theAxis.ScaleView.Zoom(xStartPos, xEndPos);
                    }
                }
                else if (e.Delta > 0)
                {
                    // Scrolled up, meaning zoom in
                    double xStartPos = Math.Max(xPixelPos - (xPixelPos - xMin) / CZoomScale, 0);
                    double xEndPos = Math.Min(xStartPos + (xMax - xMin) / CZoomScale, theAxis.Maximum);
                    theAxis.ScaleView.Zoom(xStartPos, xEndPos);
                    FZoomLevel++;
                }
            }
            catch { }
        }

        private void seriesCheckedListBox_ItemCheck(object sender, ItemCheckEventArgs e)
        {
            chart1.Series[seriesCheckedListBox.Items[e.Index].ToString()].Enabled = e.NewValue == CheckState.Checked;
        }

        private void button1_Click_1(object sender, EventArgs e)
        {

        }

        public delegate void AddTextLineToStatusTextBox_delegate(string text);

        void AddTextLineToStatusTextBox(string text)
        {
            if (statusTextBox.InvokeRequired)
            {
                AddTextLineToStatusTextBox_delegate d = new AddTextLineToStatusTextBox_delegate(AddTextLineToStatusTextBox);
                this.Invoke(d, new object[] { text });
            }
            else
            {
                statusTextBox.AppendText(text + "\r\n");
            }
        }

        DataPoint _prevPoint = null;
        private void chart1_MouseMove(object sender, MouseEventArgs e)
        {
            Point mousePoint = new Point(e.X, e.Y);
            chart1.ChartAreas[0].CursorX.SetCursorPixelPosition(mousePoint, false);
            chart1.ChartAreas[0].CursorY.SetCursorPixelPosition(mousePoint, false);

            // this if statement clears the values from the previously activated point.
            if (_prevPoint != null)
            {
                _prevPoint.MarkerStyle = MarkerStyle.None;
                _prevPoint.Label = "";
                _prevPoint.IsValueShownAsLabel = false;
            }

            var result = chart1.HitTest(e.X, e.Y, ChartElementType.DataPoint);
            if (result.ChartElementType == ChartElementType.DataPoint)
            {
                var prop = result.Object as DataPoint;
                if (prop != null)
                {
                    prop.IsValueShownAsLabel = true;
                    prop.Label = prop.XValue.ToString() + " : " + prop.YValues[0].ToString();
                    prop.MarkerStyle = MarkerStyle.Star4;
                    _prevPoint = prop;
                }
            }
        }

        private void RecalcAxesButton_Click_2(object sender, EventArgs e)
        {
            chart1.ChartAreas[0].RecalculateAxesScale();
        }

        private void button1_Click_2(object sender, EventArgs e)
        {
            List<string> lines = parameterEntry.Text.Split('\r').ToList();
            StringBuilder str = new StringBuilder();
            foreach (string s in lines)
            {
                string data = s.Trim();
                if ((!data.StartsWith("\'")) && (data.Length > 0))
                    theSocket.Send(Encoding.ASCII.GetBytes(data + " "));
            }
        }

        private void loadFromFileButton_Click(object sender, EventArgs e)
        {
            parameterEntry.Text = File.ReadAllText("parameters.txt");
        }

        private void saveToFileButton_Click(object sender, EventArgs e)
        {
            File.WriteAllText("parameters.txt", parameterEntry.Text);
        }

        private void unCheckAllButton_Click(object sender, EventArgs e)
        {
            for (int i = 0; i < seriesCheckedListBox.Items.Count; i++)
            {
                seriesCheckedListBox.SetItemChecked(i, false);
            }
        }

        private void SerializeDataSet(string filename)
        {
            XmlSerializer ser = new XmlSerializer(typeof(decoderDataSet));

            TextWriter writer = new StreamWriter(filename);
            ser.Serialize(writer, decoderInfo);
            writer.Close();
        }
    }

    [Serializable]
    public class decoderDataSet
    {
        public int portNumber;
        public List<dataTraceGroup> data;

        public decoderDataSet()
        {
            portNumber = 30200;
            data = new List<dataTraceGroup>();
        }
    }


    [Serializable]
    public class dataTraceGroup
    {
        public string name { set; get; }

        public List<dataTraceItem> dataItems;

        public dataTraceGroup()
        {
            dataItems = new List<dataTraceItem>();
        }
    }

    [Serializable]
    public class dataTraceItem
    {
        public enum dataType { integerType, doubleType}
        public string name { set; get; }
        public dataType type { set; get; }

        public dataTraceItem()
        {
            name = "default";
            type =  dataType.doubleType;
        }

        public dataTraceItem( string itemName, dataType dt)
        {
            name = itemName;
            type = dt;
        }

        public string getQualifiedName(string groupName)
        {
            return String.Format("{0} : {1}",name, groupName);
        }
    }
}
