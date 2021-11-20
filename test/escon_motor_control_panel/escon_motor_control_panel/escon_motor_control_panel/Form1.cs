using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.IO.Ports;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;

namespace escon_motor_control_panel
{
    public partial class Form1 : Form
    {
        private SerialPort serialPort;

        public Form1()
        {
            InitializeComponent();
        }

        private void buttonSerialPortConnect_Click(object sender, EventArgs e)
        {
            if (serialPort == null)
            {
                var baudrate = (int)numericUpDownSerialPortBaudrate.Value;
                var port = comboBoxSerialPorts.SelectedText;
                serialPort = new SerialPort(port, baudrate);

                try
                {
                    serialPort.Open();
                    buttonSerialPortConnect.Text = "Disconnect";
                }
                catch (Exception ex)
                {
                    serialPort = null;
                    MessageBox.Show(ex.Message);
                }
            }
            else
            {
                serialPort.Close();
                serialPort = null;
                buttonSerialPortConnect.Text = "Connect";
            }
        }

        private void comboBoxSerialPorts_Click(object sender, EventArgs e)
        {
            comboBoxSerialPorts.Items.Clear();
            var allPorts = SerialPort.GetPortNames();
            comboBoxSerialPorts.Items.AddRange(allPorts);
        }
    }
}
