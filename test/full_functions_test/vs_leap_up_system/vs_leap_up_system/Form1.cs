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
                    serialPort.DataReceived += (s, ea) =>
                    {
                        try
                        {
                            Console.WriteLine((s as SerialPort).ReadTo("\r\n"));
                        }
                        catch (Exception)
                        { }
                    };
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
    }
}
