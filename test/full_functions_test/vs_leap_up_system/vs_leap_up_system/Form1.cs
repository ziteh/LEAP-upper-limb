﻿using System;
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
        private const double armL2 = 425;

        #endregion Value

        private enum Joint
        {
            EFE,
            SFE
        };

        private double sfeAngle;
        private double efeAngle;
        private double forceSensorX;
        private double forceSensorY;
        private double forceSensorSourceX;
        private double forceSensorSourceY;

        private Timer timer = new Timer();

        public Form1()
        {
            InitializeComponent();

            UpdateSerialPortName();
            timer.Interval = 200;
            timer.Tick += TimerEvenHandler;
            timer.Enabled = true;
        }

        private void TimerEvenHandler(object sender, EventArgs e)
        {
            var convertedPoint = ForceSensorCoordinateConvert(
                new Point3D() { x = forceSensorSourceX, y = forceSensorSourceY, z = 0 },
                (sfeAngle + efeAngle));

            forceSensorX = convertedPoint.x;
            forceSensorY = convertedPoint.y;

            textBoxForceSensorX.Text = forceSensorX.ToString(" 0000.00;-0000.00");
            textBoxForceSensorY.Text = forceSensorY.ToString(" 0000.00;-0000.00");
            if (Math.Abs(forceSensorX) > 110)
            {
                RelaviteMove(0, forceSensorX / 10.0);
            }
            if (Math.Abs(forceSensorY) > 110)
            {
                RelaviteMove(1, -forceSensorY / 10.0);
            }

            textBoxNowPositionSfe.Text = sfeAngle.ToString(" 000.00;-000.00");
            textBoxNowPositionEfe.Text = efeAngle.ToString(" 000.00;-000.00");

            var point = ForwardKinematics2(sfeAngle, efeAngle, armL1, armL2);
            textBoxX.Text = point.x.ToString(" 000.00;-000.00");
            textBoxY.Text = point.y.ToString(" 000.00;-000.00");
        }

        #region Serial Port

        private SerialPort serialPort = null;
        private byte[] buffer = new byte[16];
        private int payloadNumber;
        private byte header;
        private const byte EOT = 0xff;

        private void buttonSerialPortConnection_Click(object sender, EventArgs e)
        {
            if (serialPort == null)
            {
                try
                {
                    SerialPortConnect();
                }
                catch (Exception ex)
                {
                    SerialPortDisconnect();
                    MessageBox.Show(ex.Message);
                }
            }
            else
            {
                SerialPortDisconnect();
            }
        }

        private void SerialPortConnect()
        {
            var port = comboBoxSerialPortName.SelectedItem.ToString();
            serialPort = new SerialPort(port, 9600);
            serialPort.DataReceived += SerialPortDataReceivedHandler;
            serialPort.Open();

            timer.Start();
            buttonSerialPortSend.Enabled = true;
            this.Text = "LEAP-Up (Connected)";
        }

        private void SerialPortDisconnect()
        {
            if (serialPort != null)
            {
                serialPort.DataReceived -= SerialPortDataReceivedHandler;
                serialPort.Close();
                serialPort = null;
            }

            timer.Stop();
            buttonSerialPortSend.Enabled = false;
            this.Text = "LEAP-Up (Disconnected)";
        }

        private void SerialPortDataReceivedHandler(object sender, SerialDataReceivedEventArgs e)
        {
            try
            {
                var sp = sender as SerialPort;
                int indata = sp.ReadByte();
                if ((indata & 0x80) == 0x80)
                {
                    // Is header of EOT.
                    header = (byte)indata;
                    switch (indata)
                    {
                        case 0x8c:
                            payloadNumber = 5;
                            break;
                        case 0x8d:
                            payloadNumber = 7;
                            break;
                        default:
                            break;
                    }

                }
                else
                {
                    // Is payload.
                    buffer[--payloadNumber] = (byte)indata;
                    if (payloadNumber == 0)
                    {
                        switch (header)
                        {
                            case 0x8c:
                                {
                                    var joint = buffer[4] == 0 ? Joint.EFE : Joint.SFE;
                                    double now = (buffer[3] & 0x3f) | ((buffer[2] & 0x3f) << 6);
                                    double goal = (buffer[1] & 0x3f) | ((buffer[0] & 0x3f) << 6);

                                    now *= (359.0 / 4095.0);
                                    goal *= (359.0 / 4095.0);

                                    now = now > 180 ? -(360 - now) : now;
                                    goal = goal > 180 ? -(360 - goal) : goal;

                                    // Show on console
                                    var msg = $"{joint}: Now: {now: 000.00;-000.00}, Goal: {goal: 000.00;-000.00}";
                                    if (joint == Joint.SFE)
                                    {
                                        Console.WriteLine(msg);
                                        sfeAngle = now;
                                    }
                                    else
                                    {
                                        Console.Write(msg + ". ");
                                        efeAngle = now;
                                    }
                                    break;
                                }
                            case 0x8d:
                                {
                                    var id = buffer[6];
                                    int x = (buffer[5] & 0x3f) | ((buffer[4] & 0x3f) << 6);
                                    int y = (buffer[3] & 0x3f) | ((buffer[2] & 0x3f) << 6);
                                    int z = (buffer[1] & 0x3f) | ((buffer[0] & 0x3f) << 6);

                                    if ((x >> 11) == 1)
                                    {
                                        x = -((0xfff - x) + 1);
                                    }
                                    if ((y >> 11) == 1)
                                    {
                                        y = -((0xfff - y) + 1);
                                    }
                                    if ((z >> 11) == 1)
                                    {
                                        z = -((0xfff - z) + 1);
                                    }

                                    //y = -y;
                                    forceSensorSourceX = x;
                                    forceSensorSourceY = y;

                                    Console.WriteLine($"X:{x}, Y:{y}, Z:{z}");
                                    break;
                                }
                            default:
                                break;
                        }
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
        private void buttonSerialPortSend_Click(object sender, EventArgs e)
        {
            SendMotorPositionControl(Joint.EFE, (double)numericUpDownEfeGoal.Value);
            SendMotorPositionControl(Joint.SFE, (double)numericUpDownSfeGoal.Value);
        }

        #endregion Serial Port

        #region Kinematics

        struct Point3D { public double x, y, z; };

        private Point3D ForwardKinematics2(double r1, double r2, double l1, double l2)
        {
            r1 *= Math.PI / 180.0;
            r2 *= Math.PI / 180.0;

            return new Point3D
            {
                x = l1 * Math.Cos(r1) +
                    l2 * Math.Cos(r1 + r2),

                y = -(l1 * Math.Sin(r1) +
                    l2 * Math.Sin(r1 + r2)),

                z = 0
            };
        }

        // XXX
        private Point3D ForwardKinematics3(double r1, double r2, double r3, double l1, double l2)
        {
            r1 *= Math.PI / 180.0;
            r2 *= Math.PI / 180.0;
            r3 *= Math.PI / 180.0;

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

        // XXX
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

        private Point3D ForceSensorCoordinateConvert(Point3D source, double theta)
        {
            theta *= Math.PI / 180.0;
            return new Point3D()
            {
                x = source.x * Math.Cos(theta) - source.y * Math.Sin(theta),
                y = source.x * Math.Sin(theta) + source.y * Math.Cos(theta),
                z = source.z
            };
        }

        private void SendMotorPositionControl(Joint joint, double angleInDegree)
        {
            angleInDegree = angleInDegree < 0 ? (360 + angleInDegree) : angleInDegree;
            angleInDegree *= (4095.0 / 359.0);

            var data = new byte[]
            {
                0x81,
                (byte)joint,
                (byte)((int)angleInDegree & 0x3f),
                (byte)(((int)angleInDegree >> 6) & 0x3f)
            };

            try
            {
                serialPort.Write(data, 0, data.Length);
            }
            catch (Exception ex)
            {
                MessageBox.Show(ex.Message);
            }
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

        private void RelaviteMove(int index, double value)
        {
            var nowPoint = ForwardKinematics2(sfeAngle, efeAngle, armL1, armL2);
            if (index == 0)
            {
                nowPoint.x += value;
            }
            else
            {
                nowPoint.y += value;
            }

            InverseKinematics2(nowPoint, armL1, armL2, out var r1, out var r2);
            SendMotorPositionControl(Joint.SFE, r1);
            SendMotorPositionControl(Joint.EFE, r2);
        }

        private void buttonYp_Click(object sender, EventArgs e)
        {
            RelaviteMove(1, -(double)numericUpDownRelativeValue.Value);
        }

        private void buttonYm_Click(object sender, EventArgs e)
        {
            RelaviteMove(1, (double)numericUpDownRelativeValue.Value);
        }

        private void buttonXp_Click(object sender, EventArgs e)
        {
            RelaviteMove(0, (double)numericUpDownRelativeValue.Value);
        }

        private void buttonXm_Click(object sender, EventArgs e)
        {
            RelaviteMove(0, -(double)numericUpDownRelativeValue.Value);
        }

        private void numericUpDownForceSensorConverTestX_ValueChanged(object sender, EventArgs e)
        {
            UpdateTestForceSensorConvert();
        }

        private void numericUpDownForceSensorConverTestY_ValueChanged(object sender, EventArgs e)
        {

            UpdateTestForceSensorConvert();
        }

        private void UpdateTestForceSensorConvert()
        {
            var theta = (double)numericUpDownForceSensorConverTestXnumericUpDownForceSensorConverTestTheta.Value;
            var x = (double)numericUpDownForceSensorConverTestX.Value;
            var y = (double)numericUpDownForceSensorConverTestY.Value;

            var converted = ForceSensorCoordinateConvert(new Point3D() { x = x, y = y, z = 0 }, theta);
            numericUpDownForceSensorConverTestConvertedX.Text = converted.x.ToString("000.00");
            numericUpDownForceSensorConverTestConvertedY.Text = converted.y.ToString("000.00");
        }

        private void numericUpDownForceSensorConverTestXnumericUpDownForceSensorConverTestTheta_ValueChanged(object sender, EventArgs e)
        {
            UpdateTestForceSensorConvert();
        }
    }
}
