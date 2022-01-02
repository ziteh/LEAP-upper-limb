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
        #region Value

        private const double armL1 = 265;
        private const double armL2 = 220;
        private const byte EOT = 0xff;

        #endregion Value

        private SerialPort serialPort = null;
        private byte[] buffer = new byte[16];
        private int payloadNumber;

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
                    serialPort.DataReceived += SerialPortDataReceivedHandler;
                    serialPort.Open();

                    buttonSerialPortSend.Enabled = true;
                }
                catch (Exception ex)
                {
                    serialPort = null;
                    buttonSerialPortSend.Enabled = false;
                    MessageBox.Show(ex.Message);
                }
            }
            else
            {
                serialPort.Close();
                serialPort.DataReceived -= SerialPortDataReceivedHandler;
                serialPort = null;

                buttonSerialPortSend.Enabled = false;
            }
        }

        private void SerialPortDataReceivedHandler(object sender, SerialDataReceivedEventArgs e)
        {
            try
            {
                var sp = sender as SerialPort;
                int indata = sp.ReadByte();
                if (indata == 0x8c)
                {
                    payloadNumber = 5;
                }
                else if (indata != EOT)
                {
                    buffer[--payloadNumber] = (byte)indata;
                    if (payloadNumber == 0)
                    {
                        var joint = buffer[4] == 0 ? "EFE" : "SFE";
                        double now = (buffer[3] & 0x3f) | ((buffer[2] & 0x3f) << 6);
                        double goal = (buffer[1] & 0x3f) | ((buffer[0] & 0x3f) << 6);

                        now = now * 359.0 / 4095.0;
                        goal = goal * 359.0 / 4095.0;

                        now = now > 180 ? -(360 - now) : now;
                        goal = goal > 180 ? -(360 - goal) : goal;

                        Console.WriteLine($"{joint}: Now: {now: 000.00;-000.00}, Goal: {goal: 000.00;-000.00}");
                    }
                }
            }
            catch { }
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

        #region Kinematics

        struct Point3D { public double x, y, z; };

        private Point3D ForwardKinematics3(double r1, double r2, double r3, double l1, double l2)
        {
            Point3D res = new Point3D()
            {
                x = Math.Sin(r1) +
                    l1 * Math.Cos(r1) * Math.Cos(r2) +
                    l2 * Math.Cos(r1) * Math.Cos(r2 + r3),

                y = -Math.Cos(r1) +
                    l1 * Math.Sin(r1) * Math.Cos(r2) +
                    l2 * Math.Sin(r1) * Math.Cos(r2 + r3),

                z = l1 * Math.Sin(r2) + l2 * Math.Sin(r2 + r3)
            };

            return res;
        }

        private void InverseKinematics3(Point3D point3D,
                                        double l1,
                                        double l2,
                                        out double r1,
                                        out double r2,
                                        out double r3)
        {
            var a2 = Math.Sqrt(Math.Pow(point3D.x, 2) +
                               Math.Pow(point3D.y, 2) +
                               Math.Pow(point3D.z, 2));

            var c3 = (Math.Pow(a2, 2) - (Math.Pow(l1, 2) + Math.Pow(l2, 2))) /
                     (2 * l1 * l2);

            var cb = (Math.Pow(a2, 2) + Math.Pow(l1, 2) - Math.Pow(l2, 2)) /
                     (2 * l1 * a2);

            r1 = Math.Atan2(point3D.y, point3D.x);

            r2 = Math.Atan2(point3D.z, Math.Sqrt(Math.Pow(point3D.x, 2) + Math.Pow(point3D.y, 2))) +
                 Math.Atan2(Math.Sqrt(1 - Math.Pow(cb, 2)), cb);

            r3 = Math.Atan2(Math.Sqrt(1 - Math.Pow(c3, 2)), Math.Pow(c3, 2));
        }

        private void InverseKinematics2(Point3D point3D,
                                        double l1,
                                        double l2,
                                        out double r1,
                                        out double r2)
        {
            r2 = 2 * Math.Atan2(
                Math.Sqrt(Math.Pow(l1 + l2, 2) - (Math.Pow(point3D.x, 2) + Math.Pow(point3D.y, 2))),
                Math.Sqrt((Math.Pow(point3D.x, 2) + Math.Pow(point3D.y, 2)) - Math.Pow(l1 - l2, 2)));

            r1 = -(Math.Atan2(point3D.y, point3D.x) +
                   Math.Atan2(l2 * Math.Sin(r2), l1 + l2 * Math.Cos(r2)));

            r1 = r1 * 180.0 / Math.PI;
            r2 = r2 * 180.0 / Math.PI;
        }

        #endregion Kinematics

        private void buttonSerialPortSend_Click(object sender, EventArgs e)
        {
            var efeGoal = numericUpDownEfeGoal.Value;
            efeGoal = efeGoal < 0 ? (360 + efeGoal) : efeGoal;
            efeGoal = (decimal)((double)efeGoal * 4095.0 / 359.0);
            var efeData = new byte[]
            {
                0x81,
                0x00,
                (byte)((int)efeGoal & 0x3f),
                (byte)(((int)efeGoal >> 6) & 0x3f)
            };

            var sfeGoal = numericUpDownSfeGoal.Value;
            sfeGoal = sfeGoal < 0 ? (360 + sfeGoal) : sfeGoal;
            sfeGoal = (decimal)((double)sfeGoal * 4095.0 / 359.0);
            var sfeData = new byte[]
            {
                0x81,
                0x01,
                (byte)((int)sfeGoal & 0x3f),
                (byte)(((int)sfeGoal >> 6) & 0x3f)
            };

            serialPort.Write(efeData, 0, efeData.Length);
            serialPort.Write(sfeData, 0, sfeData.Length);
        }

        private void UpdateIK()
        {
            var point = new Point3D
            {
                x = (double)numericUpDownX.Value,
                y = (double)numericUpDownY.Value,
                z = 0
            };

            InverseKinematics2(point, armL1, armL2, out var r1, out var r2);
            textBoxIKsfe.Text = r1.ToString();
            textBoxIKefe.Text = r2.ToString();
        }

        private void numericUpDownX_ValueChanged(object sender, EventArgs e)
        {
            UpdateIK();
        }

        private void numericUpDownY_ValueChanged(object sender, EventArgs e)
        {
            UpdateIK();
        }

        private void buttonCopy_Click(object sender, EventArgs e)
        {
            try
            {
                numericUpDownSfeGoal.Value = Decimal.Parse(textBoxIKsfe.Text);
                numericUpDownEfeGoal.Value = Decimal.Parse(textBoxIKefe.Text);
            }
            catch { }
        }
    }
}
