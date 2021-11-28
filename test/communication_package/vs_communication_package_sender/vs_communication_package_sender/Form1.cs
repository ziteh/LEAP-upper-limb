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

namespace vs_communication_package_sender
{
    public partial class Form1 : Form
    {
        private SerialPort serialPort;

        public Form1()
        {
            InitializeComponent();

            UpdateSerialPorts();
            if (comboBoxSerialPorts.Items.Count > 0)
            {
                comboBoxSerialPorts.SelectedIndex = 0;
            }
        }

        private void UpdateSerialPorts()
        {
            comboBoxSerialPorts.Items.Clear();
            var allPorts = SerialPort.GetPortNames();
            comboBoxSerialPorts.Items.AddRange(allPorts);
        }

        private void UpdateSendData()
        {
            textBoxSendPreview.Text = $"0x80";
        }

        private byte[] ParseSendData()
        {
            byte en;
            if (radioButtonMotorEnable.Checked)
            {
                en = 0x01;
            }
            else if (radioButtonMotorEnableToggle.Checked)
            {
                en = 0x02;
            }
            else
            {
                en = 0x00;
            }

            byte dir;
            if (radioButtonDirToggle.Checked)
            {
                dir = 0x02;
            }
            else if (radioButtonDirCCW.Checked)
            {
                dir = 0x01;
            }
            else
            {
                dir = 0x00;
            }

            byte id = (byte)numericUpDownMotorID.Value;

            return new byte[] { 0x80, id, (byte)(en | (dir << 2)), 0x38, 0x38 };
        }

        private void comboBoxSerialPorts_Click(object sender, EventArgs e)
        {
            UpdateSerialPorts();
        }

        private void buttonConnect_Click(object sender, EventArgs e)
        {
            if (serialPort == null)
            {
                var baudrate = (int)numericUpDownBaudrate.Value;
                var port = comboBoxSerialPorts.SelectedItem.ToString();
                serialPort = new SerialPort(port, baudrate);

                try
                {
                    serialPort.Open();
                    buttonConnection.Text = "Disconnect";
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
                buttonConnection.Text = "Connect";
            }

        }

        private void buttonSend_Click(object sender, EventArgs e)
        {
            serialPort.Write(ParseSendData(), 0, 5);
        }
    }
}
