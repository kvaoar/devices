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
using System.Threading;

namespace TermoLog
{
    public partial class Form1 : Form
    {

        public static bool messageReceived = false;
        static string ip = "192.168.1.115";
        static string port = "8080";
        static UdpClient udpSENDER;
        UDPListener lsnr;
        LineReceivedEvent func;

        public Form1()
        {
            InitializeComponent();
        }

        private void chart1_Click(object sender, EventArgs e)
        {

        }

        private void serialPort1_DataReceived(object sender, System.IO.Ports.SerialDataReceivedEventArgs e)
        {
            int t = 256*serialPort1.ReadByte() + serialPort1.ReadByte();
            if (t > 4095) t =  ~t+1;
            this.BeginInvoke(func,t);
        }


        delegate void LineReceivedEvent(int t);
        void LineReceived(int t)
        {
            double celsius = t;
            celsius /= 16;
            celsius = Math.Round(celsius, 2);
            if (celsius != 85)
            {
                Text = celsius.ToString() + " *C";

                                chart1.Series[0].Points.AddXY(2 * chart1.Series[0].Points.Count, celsius);
                if (chart1.ChartAreas[0].AxisX.Maximum > chart1.ChartAreas[0].AxisX.ScaleView.Size)
                    chart1.ChartAreas[0].AxisX.ScaleView.Scroll(chart1.ChartAreas[0].AxisX.Maximum);
                int n = chart1.Series[0].Points.Count;

                if (checkBox1.Checked)
                {
                    if (n > 100) chart1.ChartAreas[0].Axes[0].ScaleView.Position = (n / 100) * 100;
                }
                if (checkStatic.Checked) {

                    if (celsius < 30) chAbox.Checked = true;
                    if (celsius > 40) chAbox.Checked = false;
                }
            }
            
        }


        private void SendCmd(string ip, string port, string command)
        {
            Byte[] senddata = Encoding.ASCII.GetBytes(command);
            if(udpSENDER.Client.Connected) udpSENDER.Send(senddata, senddata.Length);
        }


        private void Form1_Load(object sender, EventArgs e)
        {
            func = new LineReceivedEvent(LineReceived);

            comboBox1.Items.AddRange(System.IO.Ports.SerialPort.GetPortNames());
            if (comboBox1.Items.Count > 0) comboBox1.Text = comboBox1.Items[comboBox1.Items.Count - 1].ToString();

            udpSENDER = new UdpClient();
            udpSENDER.Connect(ip, Int32.Parse(port));

            lsnr = new UDPListener();
            lsnr.NewMessageReceived += OnReciv;
            lsnr.StartListener(3);
            timer1.Start();
        }
        private void OnReciv(object sender, EventArgs e)
        {
            byte[] a = ((MyMessageArgs)e).data;
            string s = Encoding.UTF8.GetString(a);
            this.BeginInvoke(func, Int32.Parse(s)
            );
        }

        private void Form1_FormClosing(object sender, FormClosingEventArgs e)
        {
            SendCmd(ip, port, "t");
            lsnr.StopListener();
            serialPort1.Close();
           /* lock (gate)
            {
                isCancellationRequested = true;
            }*/
        }

        private void button1_Click(object sender, EventArgs e)
        {
            
            if (comboBox1.Text != "")
            {

                if (serialPort1.IsOpen) {
                    serialPort1.Close(); 
                    serialPort1.PortName = comboBox1.Text; 
                }
                else { 
                    serialPort1.PortName = comboBox1.Text; 
                    serialPort1.Open(); 
                };
            }
        }

        private void checkBox1_CheckedChanged(object sender, EventArgs e)
        {

        }

        private void checkBox2_CheckedChanged(object sender, EventArgs e)
        {
            if (chAbox.Checked) SendCmd(ip, port, "A+");
            else SendCmd(ip, port, "A-");

        }

        private void chBbox_CheckedChanged(object sender, EventArgs e)
        {
            if (chBbox.Checked) SendCmd(ip, port, "B+");
            else SendCmd(ip, port, "B-");
        }

        private void twoCh_CheckedChanged(object sender, EventArgs e)
        {
            if (twoCh.Checked) SendCmd(ip, port, "on");
            else SendCmd(ip, port, "off");
        }


        private void chCbox_CheckedChanged(object sender, EventArgs e)
        {
            if (chCbox.Checked) SendCmd(ip, port, "C+");
            else SendCmd(ip, port, "C-");
        }

        private void chDbox_CheckedChanged(object sender, EventArgs e)
        {
            if (chDbox.Checked) SendCmd(ip, port, "D+");
            else SendCmd(ip, port, "D-");
        }

        private void checkBox2_CheckedChanged_1(object sender, EventArgs e)
        {

        }

        private void timer1_Tick(object sender, EventArgs e)
        {
            SendCmd(ip, port, "t");
        }

        private void Form1_KeyUp(object sender, KeyEventArgs e)
        {
            //MessageBox.Show(e.KeyCode.ToString());
            if ((e.KeyCode == Keys.D1) || (e.KeyCode == Keys.NumPad1)) chAbox.Checked = !chAbox.Checked;
            if ((e.KeyCode == Keys.D2) || (e.KeyCode == Keys.NumPad2)) chBbox.Checked = !chBbox.Checked;
            if ((e.KeyCode == Keys.D3) || (e.KeyCode == Keys.NumPad3)) chCbox.Checked = !chCbox.Checked;
            if ((e.KeyCode == Keys.D4) || (e.KeyCode == Keys.NumPad4)) chDbox.Checked = !chDbox.Checked;
            if ((e.KeyCode == Keys.D5) || (e.KeyCode == Keys.NumPad5)) twoCh.Checked = !twoCh.Checked;

        }

        private void Form1_KeyPress(object sender, KeyPressEventArgs e)
        {
           // MessageBox.Show(e.KeyChar.ToString());
        }

    
    
    }

class UDPListener
{
    private int m_portToListen = 8080;
    private volatile bool listening;
    Thread m_ListeningThread;
    public event EventHandler<MyMessageArgs> NewMessageReceived;                       

    //constructor
    public UDPListener()
    {
        this.listening = false;
    }

    public void StartListener(int exceptedMessageLength)
    {
        if (!this.listening)
        {
            m_ListeningThread = new Thread(ListenForUDPPackages);
            this.listening = true;
            m_ListeningThread.Start();
        }
    }

    public void StopListener()
    {
        this.listening = false;            
    }

    public void ListenForUDPPackages()
    {
        var timeToWait = TimeSpan.FromSeconds(1);
        UdpClient listener = null;
        try
        {
            listener = new UdpClient(m_portToListen);
        }
        catch (SocketException)
        {
            //do nothing
        }

        if (listener != null)
        {
            IPEndPoint groupEP = new IPEndPoint(IPAddress.Any, m_portToListen);

            try
            {
                while (this.listening)
                {
                    byte[] receivedData = listener.Receive(ref groupEP);
                    /*var asyncResult = listener.BeginReceive(null, null);
                    asyncResult.AsyncWaitHandle.WaitOne(timeToWait);
                    if (asyncResult.IsCompleted)
                    {
                            byte[] receivedData = listener.EndReceive(asyncResult, ref groupEP);
                            // EndReceive worked and we have received data and remote endpoint*/
                            NewMessageReceived(this, new MyMessageArgs(receivedData)); 
                   /*    
                    }
                    else
                    {
                        // The operation wasn't completed before the timeout and we're off the hook
                    }
                    //raise event    
                    */
                    
                }
            }
            catch (Exception e)
            {
                Console.WriteLine(e.ToString());
            }
            finally
            {
                listener.Close();
                Console.WriteLine("Done listening for UDP broadcast");
            }
        }
    }
}

    public class MyMessageArgs : EventArgs
    {
        public byte[] data { get; set; }

        public MyMessageArgs(byte[] newData)
        {
            data = newData;
        }

    }
}
