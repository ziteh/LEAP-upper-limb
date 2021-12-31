using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;
using System.IO.Ports;

namespace vs_leap_up_system
{
    public partial class Form1 : Form
    {
        private SerialPort serialPort = null;
        public Form1()
        {
            InitializeComponent();
            UpdateSerialPortName();
        }

        private void buttonSerialPortConnection_Click(object sender, EventArgs e)
        {
            if (serialPort == null)
            {
                try
                {
                    var port = comboBoxSerialPortName.SelectedItem.ToString();
                    serialPort = new SerialPort(port, 9600);
                    serialPort.Open();
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
            }
        }

        private void UpdateSerialPortName()
        {
            comboBoxSerialPortName.Items.Clear();
            var allPorts = SerialPort.GetPortNames();
            if (allPorts.Length > 0)
            {
                comboBoxSerialPortName.Items.AddRange(allPorts);
            }
            else
            {
                comboBoxSerialPortName.Items.Add("Not found");
            }
            comboBoxSerialPortName.SelectedIndex = 0;
        }

        private void comboBoxSerialPortName_Click(object sender, EventArgs e)
        {
            UpdateSerialPortName();
        }
    }
}
