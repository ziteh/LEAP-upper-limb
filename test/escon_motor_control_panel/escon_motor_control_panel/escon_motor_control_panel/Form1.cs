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

            UpdateserialPorts();
            if (comboBoxSerialPorts.Items.Count > 0)
            {
                comboBoxSerialPorts.SelectedIndex = 0;
            }
        }

        private void buttonSerialPortConnect_Click(object sender, EventArgs e)
        {
            if (serialPort == null)
            {
                var baudrate = (int)numericUpDownSerialPortBaudrate.Value;
                var port = comboBoxSerialPorts.SelectedItem.ToString();
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
            UpdateserialPorts();
        }

        private void UpdateserialPorts()
        {
            comboBoxSerialPorts.Items.Clear();
            var allPorts = SerialPort.GetPortNames();
            comboBoxSerialPorts.Items.AddRange(allPorts);
        }

        private void buttonEnable_Click(object sender, EventArgs e)
        {
            SendCommand(0x81); /* 1000 0001 */
        }

        private void buttonDisable_Click(object sender, EventArgs e)
        {
            SendCommand(0x80); /* 1000 0000 */
        }

        private void buttonDirCW_Click(object sender, EventArgs e)
        {
            SendCommand(0x82); /* 1000 0010 */
        }

        private void buttonDirCCW_Click(object sender, EventArgs e)
        {
            SendCommand(0x83); /* 1000 0011 */
        }

        private void SendCommand(byte command)
        {
            if (serialPort != null)
            {
                if (serialPort.IsOpen)
                {
                    try
                    {
                        serialPort.Write(new[] { command }, 0, 1);
                    }
                    catch (Exception ex)
                    {
                        MessageBox.Show(ex.Message);
                    }
                }
            }
        }

        private void trackBarPwmDutycycle_ValueChanged(object sender, EventArgs e)
        {
            if (checkBoxSendOnMove.Checked)
            {
                SendValue();
            }
        }

        private void trackBarPwmDutycycle_MouseUp(object sender, MouseEventArgs e)
        {
            if (!checkBoxSendOnMove.Checked)
            {
                SendValue();
            }
        }

        private void SendValue()
        {
            var value = trackBarPwmDutycycle.Value;
            labelPwmDutycycle.Text = $"Value ({value}%)";
            SendCommand(Convert.ToByte(value));
        }
    }
}
