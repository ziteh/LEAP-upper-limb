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

namespace vs_multi_motor_control
{
    public partial class Form1 : Form
    {
        SerialPort serialPort = null;

        public Form1()
        {
            InitializeComponent();
            UpdateSerialPort();
            comboBoxJoints.SelectedIndex = 0;
        }

        private void UpdateSerialPort()
        {
            var allPorts = SerialPort.GetPortNames();

            comboBoxSerialPortName.Items.Clear();
            if (allPorts.Length > 0)
            {
                comboBoxSerialPortName.Items.AddRange(allPorts);
            }
            else
            {
                comboBoxSerialPortName.Items.Add("No found.");
            }
            comboBoxSerialPortName.SelectedIndex = 0;
        }

        private void trackBarJointPosition_Scroll(object sender, EventArgs e)
        {
            labelJointPosition.Text = trackBarJointPosition.Value.ToString();
        }

        private void comboBoxSerialPortName_Click(object sender, EventArgs e)
        {
            UpdateSerialPort();
        }

        private void buttonSerialPortConnection_Click(object sender, EventArgs e)
        {
            if (serialPort == null)
            {
                serialPort = new SerialPort(comboBoxSerialPortName.SelectedItem.ToString(), 9600);
                try
                {
                    serialPort.Open();
                    serialPort.DataReceived += SerialPortDataReceivedHandler;

                    buttonSendDataPackage.Enabled = true;
                }
                catch (Exception ex)
                {
                    MessageBox.Show(ex.Message);
                }
            }
            else
            {
                try
                {
                    serialPort.Close();
                    serialPort = null;
                    buttonSendDataPackage.Enabled = false;
                }
                catch (Exception ex)
                {
                    MessageBox.Show(ex.Message);
                }
            }
        }

        private void buttonSendDataPackage_Click(object sender, EventArgs e)
        {
            var jointId = comboBoxJoints.SelectedIndex;
            var jointPosition = trackBarJointPosition.Value;

            var dataPackage = new byte[]
            {
                0x81,
                (byte)jointId,
                (byte)((byte)jointPosition & 0x3F),
                (byte)((byte)(jointPosition >> 6) & 0x3F)
            };

            try
            {
                serialPort.Write(dataPackage, 0, dataPackage.Length);
            }
            catch (Exception ex)
            {
                MessageBox.Show(ex.Message);
            }
        }

        private void SerialPortDataReceivedHandler(object sender,
                                                   SerialDataReceivedEventArgs e)
        {
            SerialPort sp = (SerialPort)sender;
            var indata = sp.ReadTo("\r\n");

            if (indata != null)
            {
                Console.WriteLine(indata);
                //textBoxSerialPortReceivedData.Text = indata.ToString();
            }
        }
    }
}
