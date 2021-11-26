namespace escon_motor_control_panel
{
    partial class Form1
    {
        /// <summary>
        /// Required designer variable.
        /// </summary>
        private System.ComponentModel.IContainer components = null;

        /// <summary>
        /// Clean up any resources being used.
        /// </summary>
        /// <param name="disposing">true if managed resources should be disposed; otherwise, false.</param>
        protected override void Dispose(bool disposing)
        {
            if (disposing && (components != null))
            {
                components.Dispose();
            }
            base.Dispose(disposing);
        }

        #region Windows Form Designer generated code

        /// <summary>
        /// Required method for Designer support - do not modify
        /// the contents of this method with the code editor.
        /// </summary>
        private void InitializeComponent()
        {
            this.comboBoxSerialPorts = new System.Windows.Forms.ComboBox();
            this.numericUpDownSerialPortBaudrate = new System.Windows.Forms.NumericUpDown();
            this.buttonEnable = new System.Windows.Forms.Button();
            this.buttonDisable = new System.Windows.Forms.Button();
            this.buttonDirCW = new System.Windows.Forms.Button();
            this.buttonDirCCW = new System.Windows.Forms.Button();
            this.trackBarPwmDutycycle = new System.Windows.Forms.TrackBar();
            this.label1 = new System.Windows.Forms.Label();
            this.label2 = new System.Windows.Forms.Label();
            this.labelPwmDutycycle = new System.Windows.Forms.Label();
            this.groupBox1 = new System.Windows.Forms.GroupBox();
            this.groupBox2 = new System.Windows.Forms.GroupBox();
            this.buttonSerialPortConnect = new System.Windows.Forms.Button();
            this.checkBoxSendOnMove = new System.Windows.Forms.CheckBox();
            ((System.ComponentModel.ISupportInitialize)(this.numericUpDownSerialPortBaudrate)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.trackBarPwmDutycycle)).BeginInit();
            this.groupBox1.SuspendLayout();
            this.groupBox2.SuspendLayout();
            this.SuspendLayout();
            // 
            // comboBoxSerialPorts
            // 
            this.comboBoxSerialPorts.FormattingEnabled = true;
            this.comboBoxSerialPorts.Location = new System.Drawing.Point(173, 115);
            this.comboBoxSerialPorts.Name = "comboBoxSerialPorts";
            this.comboBoxSerialPorts.Size = new System.Drawing.Size(154, 33);
            this.comboBoxSerialPorts.TabIndex = 0;
            this.comboBoxSerialPorts.Click += new System.EventHandler(this.comboBoxSerialPorts_Click);
            // 
            // numericUpDownSerialPortBaudrate
            // 
            this.numericUpDownSerialPortBaudrate.Location = new System.Drawing.Point(173, 181);
            this.numericUpDownSerialPortBaudrate.Maximum = new decimal(new int[] {
            9600,
            0,
            0,
            0});
            this.numericUpDownSerialPortBaudrate.Minimum = new decimal(new int[] {
            9600,
            0,
            0,
            0});
            this.numericUpDownSerialPortBaudrate.Name = "numericUpDownSerialPortBaudrate";
            this.numericUpDownSerialPortBaudrate.Size = new System.Drawing.Size(154, 31);
            this.numericUpDownSerialPortBaudrate.TabIndex = 1;
            this.numericUpDownSerialPortBaudrate.Value = new decimal(new int[] {
            9600,
            0,
            0,
            0});
            // 
            // buttonEnable
            // 
            this.buttonEnable.Location = new System.Drawing.Point(6, 30);
            this.buttonEnable.Name = "buttonEnable";
            this.buttonEnable.Size = new System.Drawing.Size(141, 55);
            this.buttonEnable.TabIndex = 2;
            this.buttonEnable.Text = "Enable";
            this.buttonEnable.UseVisualStyleBackColor = true;
            this.buttonEnable.Click += new System.EventHandler(this.buttonEnable_Click);
            // 
            // buttonDisable
            // 
            this.buttonDisable.Location = new System.Drawing.Point(6, 91);
            this.buttonDisable.Name = "buttonDisable";
            this.buttonDisable.Size = new System.Drawing.Size(141, 55);
            this.buttonDisable.TabIndex = 2;
            this.buttonDisable.Text = "Disable";
            this.buttonDisable.UseVisualStyleBackColor = true;
            this.buttonDisable.Click += new System.EventHandler(this.buttonDisable_Click);
            // 
            // buttonDirCW
            // 
            this.buttonDirCW.Location = new System.Drawing.Point(6, 30);
            this.buttonDirCW.Name = "buttonDirCW";
            this.buttonDirCW.Size = new System.Drawing.Size(141, 55);
            this.buttonDirCW.TabIndex = 2;
            this.buttonDirCW.Text = "CW";
            this.buttonDirCW.UseVisualStyleBackColor = true;
            this.buttonDirCW.Click += new System.EventHandler(this.buttonDirCW_Click);
            // 
            // buttonDirCCW
            // 
            this.buttonDirCCW.Location = new System.Drawing.Point(6, 91);
            this.buttonDirCCW.Name = "buttonDirCCW";
            this.buttonDirCCW.Size = new System.Drawing.Size(141, 55);
            this.buttonDirCCW.TabIndex = 2;
            this.buttonDirCCW.Text = "CCW";
            this.buttonDirCCW.UseVisualStyleBackColor = true;
            this.buttonDirCCW.Click += new System.EventHandler(this.buttonDirCCW_Click);
            // 
            // trackBarPwmDutycycle
            // 
            this.trackBarPwmDutycycle.Location = new System.Drawing.Point(622, 295);
            this.trackBarPwmDutycycle.Maximum = 100;
            this.trackBarPwmDutycycle.Name = "trackBarPwmDutycycle";
            this.trackBarPwmDutycycle.Size = new System.Drawing.Size(254, 90);
            this.trackBarPwmDutycycle.TabIndex = 3;
            this.trackBarPwmDutycycle.ValueChanged += new System.EventHandler(this.trackBarPwmDutycycle_ValueChanged);
            this.trackBarPwmDutycycle.MouseUp += new System.Windows.Forms.MouseEventHandler(this.trackBarPwmDutycycle_MouseUp);
            // 
            // label1
            // 
            this.label1.AutoSize = true;
            this.label1.Location = new System.Drawing.Point(82, 115);
            this.label1.Name = "label1";
            this.label1.Size = new System.Drawing.Size(51, 25);
            this.label1.TabIndex = 4;
            this.label1.Text = "Port";
            // 
            // label2
            // 
            this.label2.AutoSize = true;
            this.label2.Location = new System.Drawing.Point(42, 183);
            this.label2.Name = "label2";
            this.label2.Size = new System.Drawing.Size(99, 25);
            this.label2.TabIndex = 4;
            this.label2.Text = "Baudrate";
            // 
            // labelPwmDutycycle
            // 
            this.labelPwmDutycycle.AutoSize = true;
            this.labelPwmDutycycle.Location = new System.Drawing.Point(460, 319);
            this.labelPwmDutycycle.Name = "labelPwmDutycycle";
            this.labelPwmDutycycle.Size = new System.Drawing.Size(155, 25);
            this.labelPwmDutycycle.TabIndex = 4;
            this.labelPwmDutycycle.Text = "Duty Cycle (%)";
            // 
            // groupBox1
            // 
            this.groupBox1.Controls.Add(this.buttonEnable);
            this.groupBox1.Controls.Add(this.buttonDisable);
            this.groupBox1.Location = new System.Drawing.Point(471, 77);
            this.groupBox1.Name = "groupBox1";
            this.groupBox1.Size = new System.Drawing.Size(166, 169);
            this.groupBox1.TabIndex = 5;
            this.groupBox1.TabStop = false;
            this.groupBox1.Text = "Enable";
            // 
            // groupBox2
            // 
            this.groupBox2.Controls.Add(this.buttonDirCW);
            this.groupBox2.Controls.Add(this.buttonDirCCW);
            this.groupBox2.Location = new System.Drawing.Point(679, 77);
            this.groupBox2.Name = "groupBox2";
            this.groupBox2.Size = new System.Drawing.Size(165, 169);
            this.groupBox2.TabIndex = 5;
            this.groupBox2.TabStop = false;
            this.groupBox2.Text = "Direction";
            // 
            // buttonSerialPortConnect
            // 
            this.buttonSerialPortConnect.Location = new System.Drawing.Point(173, 250);
            this.buttonSerialPortConnect.Name = "buttonSerialPortConnect";
            this.buttonSerialPortConnect.Size = new System.Drawing.Size(141, 55);
            this.buttonSerialPortConnect.TabIndex = 2;
            this.buttonSerialPortConnect.Text = "Connect";
            this.buttonSerialPortConnect.UseVisualStyleBackColor = true;
            this.buttonSerialPortConnect.Click += new System.EventHandler(this.buttonSerialPortConnect_Click);
            // 
            // checkBoxSendOnMove
            // 
            this.checkBoxSendOnMove.AutoSize = true;
            this.checkBoxSendOnMove.Location = new System.Drawing.Point(465, 356);
            this.checkBoxSendOnMove.Name = "checkBoxSendOnMove";
            this.checkBoxSendOnMove.Size = new System.Drawing.Size(187, 29);
            this.checkBoxSendOnMove.TabIndex = 6;
            this.checkBoxSendOnMove.Text = "Send On Move";
            this.checkBoxSendOnMove.UseVisualStyleBackColor = true;
            // 
            // Form1
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(12F, 25F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.ClientSize = new System.Drawing.Size(944, 402);
            this.Controls.Add(this.checkBoxSendOnMove);
            this.Controls.Add(this.buttonSerialPortConnect);
            this.Controls.Add(this.groupBox2);
            this.Controls.Add(this.groupBox1);
            this.Controls.Add(this.labelPwmDutycycle);
            this.Controls.Add(this.label2);
            this.Controls.Add(this.label1);
            this.Controls.Add(this.trackBarPwmDutycycle);
            this.Controls.Add(this.numericUpDownSerialPortBaudrate);
            this.Controls.Add(this.comboBoxSerialPorts);
            this.Name = "Form1";
            this.Text = "maxon ESCON Motor Control Panel";
            ((System.ComponentModel.ISupportInitialize)(this.numericUpDownSerialPortBaudrate)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.trackBarPwmDutycycle)).EndInit();
            this.groupBox1.ResumeLayout(false);
            this.groupBox2.ResumeLayout(false);
            this.ResumeLayout(false);
            this.PerformLayout();

        }

        #endregion

        private System.Windows.Forms.ComboBox comboBoxSerialPorts;
        private System.Windows.Forms.NumericUpDown numericUpDownSerialPortBaudrate;
        private System.Windows.Forms.Button buttonEnable;
        private System.Windows.Forms.Button buttonDisable;
        private System.Windows.Forms.Button buttonDirCW;
        private System.Windows.Forms.Button buttonDirCCW;
        private System.Windows.Forms.TrackBar trackBarPwmDutycycle;
        private System.Windows.Forms.Label label1;
        private System.Windows.Forms.Label label2;
        private System.Windows.Forms.Label labelPwmDutycycle;
        private System.Windows.Forms.GroupBox groupBox1;
        private System.Windows.Forms.GroupBox groupBox2;
        private System.Windows.Forms.Button buttonSerialPortConnect;
        private System.Windows.Forms.CheckBox checkBoxSendOnMove;
    }
}

