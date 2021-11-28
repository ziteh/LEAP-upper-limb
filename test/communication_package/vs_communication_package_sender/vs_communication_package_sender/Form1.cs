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

            comboBoxMode.SelectedIndex = 0;
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

        private byte[] ParseMotorBasicControlSendingeData()
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

            return new byte[] { 0x80, id, (byte)(en | (dir << 2)) };
        }

        private byte[] ParseMotorPositionControlSendingData()
        {
            byte id = (byte)numericUpDownMotorID.Value;
            var pos = (int)((numericUpDownMotorPosition.Value / 100) * 4095);
            var bPos = BitConverter.GetBytes(pos);
            var parsed = new byte[]
            {
                0x81,
                id,
                (byte)(bPos[0] & 0x3f),
                (byte)((bPos[0] & 0xc0 >> 6 | bPos[1] << 2) & 0x3f)
            };
            return parsed;
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
            if (comboBoxMode.SelectedItem.ToString() == "Motor Basic Control")
            {
                serialPort.Write(ParseMotorBasicControlSendingeData(), 0, 3);
            }
            else if (comboBoxMode.SelectedItem.ToString() == "Motor Position Control")
            {
                serialPort.Write(ParseMotorPositionControlSendingData(), 0, 4);
            }
        }
    }
}
