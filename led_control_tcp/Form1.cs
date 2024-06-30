using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;
using System.Net.Sockets;
namespace led_control_tcp
{
    public partial class Form1 : Form
    {
        Socket client;
        public Form1()
        {
            InitializeComponent();
        }

        private void Form1_Load(object sender, EventArgs e)
        {
            client = new Socket(SocketType.Stream, ProtocolType.Tcp);
            client.Connect("27.74.203.174", 333);
            while(client.Connected != true)
            {
                System.Threading.Thread.Sleep(100);
            }
            System.Console.WriteLine("connect to ESP32 server success");
        }

        private void button1_Click(object sender, EventArgs e)
        {
            client.Send(System.Text.Encoding.ASCII.GetBytes("LED ON\r\n"));
        }

        private void button2_Click(object sender, EventArgs e)
        {
            client.Send(System.Text.Encoding.ASCII.GetBytes("LED OFF\r\n"));
        }
    }
}
