namespace TermoLog
{
    partial class Form1
    {
        /// <summary>
        /// Требуется переменная конструктора.
        /// </summary>
        private System.ComponentModel.IContainer components = null;

        /// <summary>
        /// Освободить все используемые ресурсы.
        /// </summary>
        /// <param name="disposing">истинно, если управляемый ресурс должен быть удален; иначе ложно.</param>
        protected override void Dispose(bool disposing)
        {
            if (disposing && (components != null))
            {
                components.Dispose();
            }
            base.Dispose(disposing);
        }

        #region Код, автоматически созданный конструктором форм Windows

        /// <summary>
        /// Обязательный метод для поддержки конструктора - не изменяйте
        /// содержимое данного метода при помощи редактора кода.
        /// </summary>
        private void InitializeComponent()
        {
            this.components = new System.ComponentModel.Container();
            System.Windows.Forms.DataVisualization.Charting.ChartArea chartArea1 = new System.Windows.Forms.DataVisualization.Charting.ChartArea();
            System.Windows.Forms.DataVisualization.Charting.Legend legend1 = new System.Windows.Forms.DataVisualization.Charting.Legend();
            System.Windows.Forms.DataVisualization.Charting.Series series1 = new System.Windows.Forms.DataVisualization.Charting.Series();
            this.chart1 = new System.Windows.Forms.DataVisualization.Charting.Chart();
            this.serialPort1 = new System.IO.Ports.SerialPort(this.components);
            this.button1 = new System.Windows.Forms.Button();
            this.checkBox1 = new System.Windows.Forms.CheckBox();
            this.comboBox1 = new System.Windows.Forms.ComboBox();
            this.chAbox = new System.Windows.Forms.CheckBox();
            this.chBbox = new System.Windows.Forms.CheckBox();
            this.twoCh = new System.Windows.Forms.CheckBox();
            this.chCbox = new System.Windows.Forms.CheckBox();
            this.chDbox = new System.Windows.Forms.CheckBox();
            this.timer1 = new System.Windows.Forms.Timer(this.components);
            this.checkStatic = new System.Windows.Forms.CheckBox();
            ((System.ComponentModel.ISupportInitialize)(this.chart1)).BeginInit();
            this.SuspendLayout();
            // 
            // chart1
            // 
            this.chart1.Anchor = ((System.Windows.Forms.AnchorStyles)((((System.Windows.Forms.AnchorStyles.Top | System.Windows.Forms.AnchorStyles.Bottom) 
            | System.Windows.Forms.AnchorStyles.Left) 
            | System.Windows.Forms.AnchorStyles.Right)));
            chartArea1.AxisX.Enabled = System.Windows.Forms.DataVisualization.Charting.AxisEnabled.True;
            chartArea1.AxisX.IntervalAutoMode = System.Windows.Forms.DataVisualization.Charting.IntervalAutoMode.VariableCount;
            chartArea1.AxisX.LabelAutoFitMinFontSize = 10;
            chartArea1.AxisX.LabelAutoFitStyle = ((System.Windows.Forms.DataVisualization.Charting.LabelAutoFitStyles)((System.Windows.Forms.DataVisualization.Charting.LabelAutoFitStyles.StaggeredLabels | System.Windows.Forms.DataVisualization.Charting.LabelAutoFitStyles.WordWrap)));
            chartArea1.AxisX.LabelStyle.Interval = 10D;
            chartArea1.AxisX.MajorGrid.Interval = 10D;
            chartArea1.AxisX.MajorGrid.IntervalType = System.Windows.Forms.DataVisualization.Charting.DateTimeIntervalType.Number;
            chartArea1.AxisX.MajorTickMark.Interval = 10D;
            chartArea1.AxisX.Minimum = 0D;
            chartArea1.AxisX.MinorTickMark.Interval = 1D;
            chartArea1.AxisX.ScaleView.MinSize = 60D;
            chartArea1.AxisX.ScaleView.MinSizeType = System.Windows.Forms.DataVisualization.Charting.DateTimeIntervalType.Number;
            chartArea1.AxisX.ScaleView.Size = 60D;
            chartArea1.AxisX.ScaleView.SizeType = System.Windows.Forms.DataVisualization.Charting.DateTimeIntervalType.Number;
            chartArea1.AxisX.ScaleView.SmallScrollMinSizeType = System.Windows.Forms.DataVisualization.Charting.DateTimeIntervalType.Number;
            chartArea1.AxisX.ScaleView.SmallScrollSize = 10D;
            chartArea1.AxisX.ScaleView.SmallScrollSizeType = System.Windows.Forms.DataVisualization.Charting.DateTimeIntervalType.Number;
            chartArea1.AxisX.ScrollBar.IsPositionedInside = false;
            chartArea1.AxisX.ScrollBar.LineColor = System.Drawing.Color.FromArgb(((int)(((byte)(255)))), ((int)(((byte)(255)))), ((int)(((byte)(192)))));
            chartArea1.AxisX.ScrollBar.Size = 20D;
            chartArea1.AxisX2.Enabled = System.Windows.Forms.DataVisualization.Charting.AxisEnabled.False;
            chartArea1.AxisY.IsStartedFromZero = false;
            chartArea1.AxisY2.Interval = 0.1D;
            chartArea1.Name = "ChartArea1";
            this.chart1.ChartAreas.Add(chartArea1);
            legend1.Name = "Legend1";
            this.chart1.Legends.Add(legend1);
            this.chart1.Location = new System.Drawing.Point(93, 12);
            this.chart1.Name = "chart1";
            series1.BorderWidth = 3;
            series1.ChartArea = "ChartArea1";
            series1.ChartType = System.Windows.Forms.DataVisualization.Charting.SeriesChartType.Spline;
            series1.CustomProperties = "LineTension=0.2";
            series1.IsValueShownAsLabel = true;
            series1.Legend = "Legend1";
            series1.MarkerBorderColor = System.Drawing.Color.FromArgb(((int)(((byte)(0)))), ((int)(((byte)(0)))), ((int)(((byte)(192)))));
            series1.MarkerColor = System.Drawing.Color.FromArgb(((int)(((byte)(0)))), ((int)(((byte)(0)))), ((int)(((byte)(192)))));
            series1.MarkerStyle = System.Windows.Forms.DataVisualization.Charting.MarkerStyle.Square;
            series1.Name = "Series1";
            series1.SmartLabelStyle.AllowOutsidePlotArea = System.Windows.Forms.DataVisualization.Charting.LabelOutsidePlotAreaStyle.No;
            series1.SmartLabelStyle.CalloutLineAnchorCapStyle = System.Windows.Forms.DataVisualization.Charting.LineAnchorCapStyle.None;
            series1.SmartLabelStyle.CalloutStyle = System.Windows.Forms.DataVisualization.Charting.LabelCalloutStyle.None;
            series1.SmartLabelStyle.MaxMovingDistance = 0D;
            this.chart1.Series.Add(series1);
            this.chart1.Size = new System.Drawing.Size(1075, 397);
            this.chart1.TabIndex = 0;
            this.chart1.Text = "chart1";
            this.chart1.Click += new System.EventHandler(this.chart1_Click);
            // 
            // serialPort1
            // 
            this.serialPort1.BaudRate = 2400;
            this.serialPort1.PortName = "COM5";
            this.serialPort1.DataReceived += new System.IO.Ports.SerialDataReceivedEventHandler(this.serialPort1_DataReceived);
            // 
            // button1
            // 
            this.button1.Location = new System.Drawing.Point(12, 12);
            this.button1.Name = "button1";
            this.button1.Size = new System.Drawing.Size(75, 23);
            this.button1.TabIndex = 1;
            this.button1.Text = "button1";
            this.button1.UseVisualStyleBackColor = true;
            this.button1.Click += new System.EventHandler(this.button1_Click);
            // 
            // checkBox1
            // 
            this.checkBox1.AutoSize = true;
            this.checkBox1.Location = new System.Drawing.Point(4, 71);
            this.checkBox1.Name = "checkBox1";
            this.checkBox1.Size = new System.Drawing.Size(83, 21);
            this.checkBox1.TabIndex = 2;
            this.checkBox1.Text = "autonew";
            this.checkBox1.UseVisualStyleBackColor = true;
            this.checkBox1.CheckedChanged += new System.EventHandler(this.checkBox1_CheckedChanged);
            // 
            // comboBox1
            // 
            this.comboBox1.FormattingEnabled = true;
            this.comboBox1.Location = new System.Drawing.Point(12, 41);
            this.comboBox1.Name = "comboBox1";
            this.comboBox1.Size = new System.Drawing.Size(75, 24);
            this.comboBox1.TabIndex = 3;
            // 
            // chAbox
            // 
            this.chAbox.AutoSize = true;
            this.chAbox.Location = new System.Drawing.Point(4, 98);
            this.chAbox.Name = "chAbox";
            this.chAbox.Size = new System.Drawing.Size(54, 21);
            this.chAbox.TabIndex = 4;
            this.chAbox.Text = "chA";
            this.chAbox.UseVisualStyleBackColor = true;
            this.chAbox.CheckedChanged += new System.EventHandler(this.checkBox2_CheckedChanged);
            // 
            // chBbox
            // 
            this.chBbox.AutoSize = true;
            this.chBbox.Location = new System.Drawing.Point(4, 125);
            this.chBbox.Name = "chBbox";
            this.chBbox.Size = new System.Drawing.Size(54, 21);
            this.chBbox.TabIndex = 5;
            this.chBbox.Text = "chB";
            this.chBbox.UseVisualStyleBackColor = true;
            this.chBbox.CheckedChanged += new System.EventHandler(this.chBbox_CheckedChanged);
            // 
            // twoCh
            // 
            this.twoCh.AutoSize = true;
            this.twoCh.Location = new System.Drawing.Point(4, 208);
            this.twoCh.Name = "twoCh";
            this.twoCh.Size = new System.Drawing.Size(44, 21);
            this.twoCh.TabIndex = 6;
            this.twoCh.Text = "all";
            this.twoCh.UseVisualStyleBackColor = true;
            this.twoCh.CheckedChanged += new System.EventHandler(this.twoCh_CheckedChanged);
            // 
            // chCbox
            // 
            this.chCbox.AutoSize = true;
            this.chCbox.Location = new System.Drawing.Point(4, 152);
            this.chCbox.Name = "chCbox";
            this.chCbox.Size = new System.Drawing.Size(54, 21);
            this.chCbox.TabIndex = 7;
            this.chCbox.Text = "chC";
            this.chCbox.UseVisualStyleBackColor = true;
            this.chCbox.CheckedChanged += new System.EventHandler(this.chCbox_CheckedChanged);
            // 
            // chDbox
            // 
            this.chDbox.AutoSize = true;
            this.chDbox.Location = new System.Drawing.Point(4, 179);
            this.chDbox.Name = "chDbox";
            this.chDbox.Size = new System.Drawing.Size(55, 21);
            this.chDbox.TabIndex = 8;
            this.chDbox.Text = "chD";
            this.chDbox.UseVisualStyleBackColor = true;
            this.chDbox.CheckedChanged += new System.EventHandler(this.chDbox_CheckedChanged);
            // 
            // timer1
            // 
            this.timer1.Interval = 2000;
            this.timer1.Tick += new System.EventHandler(this.timer1_Tick);
            // 
            // checkStatic
            // 
            this.checkStatic.AutoSize = true;
            this.checkStatic.Location = new System.Drawing.Point(4, 235);
            this.checkStatic.Name = "checkStatic";
            this.checkStatic.Size = new System.Drawing.Size(71, 21);
            this.checkStatic.TabIndex = 9;
            this.checkStatic.Text = "Termo";
            this.checkStatic.UseVisualStyleBackColor = true;
            // 
            // Form1
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(8F, 16F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.ClientSize = new System.Drawing.Size(1202, 421);
            this.Controls.Add(this.checkStatic);
            this.Controls.Add(this.chDbox);
            this.Controls.Add(this.chCbox);
            this.Controls.Add(this.twoCh);
            this.Controls.Add(this.chBbox);
            this.Controls.Add(this.chAbox);
            this.Controls.Add(this.comboBox1);
            this.Controls.Add(this.checkBox1);
            this.Controls.Add(this.button1);
            this.Controls.Add(this.chart1);
            this.KeyPreview = true;
            this.Name = "Form1";
            this.Text = "Form1";
            this.FormClosing += new System.Windows.Forms.FormClosingEventHandler(this.Form1_FormClosing);
            this.Load += new System.EventHandler(this.Form1_Load);
            this.KeyPress += new System.Windows.Forms.KeyPressEventHandler(this.Form1_KeyPress);
            this.KeyUp += new System.Windows.Forms.KeyEventHandler(this.Form1_KeyUp);
            ((System.ComponentModel.ISupportInitialize)(this.chart1)).EndInit();
            this.ResumeLayout(false);
            this.PerformLayout();

        }

        #endregion

        private System.Windows.Forms.DataVisualization.Charting.Chart chart1;
        private System.IO.Ports.SerialPort serialPort1;
        private System.Windows.Forms.Button button1;
        private System.Windows.Forms.CheckBox checkBox1;
        private System.Windows.Forms.ComboBox comboBox1;
        private System.Windows.Forms.CheckBox chAbox;
        private System.Windows.Forms.CheckBox chBbox;
        private System.Windows.Forms.CheckBox twoCh;
        private System.Windows.Forms.CheckBox chCbox;
        private System.Windows.Forms.CheckBox chDbox;
        private System.Windows.Forms.Timer timer1;
        private System.Windows.Forms.CheckBox checkStatic;
    }
}

