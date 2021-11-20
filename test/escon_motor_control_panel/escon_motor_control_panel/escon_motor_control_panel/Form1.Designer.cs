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
            this.label3 = new System.Windows.Forms.Label();
            this.groupBox1 = new System.Windows.Forms.GroupBox();
            this.groupBox2 = new System.Windows.Forms.GroupBox();
            this.buttonSerialPortConnect = new System.Windows.Forms.Button();
            ((System.ComponentModel.ISupportInitialize)(this.numericUpDownSerialPortBaudrate)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.trackBarPwmDutycycle)).BeginInit();
            this.groupBox1.SuspendLayout();
            this.groupBox2.SuspendLayout();
            this.SuspendLayout();
            // 
            // comboBoxSerialPorts
            // 
            this.comboBoxSerialPorts.FormattingEnabled = true;
            this.comboBoxSerialPorts.Location = new System.Drawing.Point(109, 74);
            this.comboBoxSerialPorts.Name = "comboBoxSerialPorts";
            this.comboBoxSerialPorts.Size = new System.Drawing.Size(154, 33);
            this.comboBoxSerialPorts.TabIndex = 0;
            this.comboBoxSerialPorts.Click += new System.EventHandler(this.comboBoxSerialPorts_Click);
            // 
            // numericUpDownSerialPortBaudrate
            // 
            this.numericUpDownSerialPortBaudrate.Location = new System.Drawing.Point(109, 143);
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
            this.numericUpDownSerialPortBaudrate.Size = new System.Drawing.Size(120, 31);
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
            // 
            // buttonDisable
            // 
            this.buttonDisable.Location = new System.Drawing.Point(6, 91);
            this.buttonDisable.Name = "buttonDisable";
            this.buttonDisable.Size = new System.Drawing.Size(141, 55);
            this.buttonDisable.TabIndex = 2;
            this.buttonDisable.Text = "Disable";
            this.buttonDisable.UseVisualStyleBackColor = true;
            // 
            // buttonDirCW
            // 
            this.buttonDirCW.Location = new System.Drawing.Point(6, 30);
            this.buttonDirCW.Name = "buttonDirCW";
            this.buttonDirCW.Size = new System.Drawing.Size(141, 55);
            this.buttonDirCW.TabIndex = 2;
            this.buttonDirCW.Text = "CW";
            this.buttonDirCW.UseVisualStyleBackColor = true;
            // 
            // buttonDirCCW
            // 
            this.buttonDirCCW.Location = new System.Drawing.Point(6, 91);
            this.buttonDirCCW.Name = "buttonDirCCW";
            this.buttonDirCCW.Size = new System.Drawing.Size(141, 55);
            this.buttonDirCCW.TabIndex = 2;
            this.buttonDirCCW.Text = "CCW";
            this.buttonDirCCW.UseVisualStyleBackColor = true;
            // 
            // trackBarPwmDutycycle
            // 
            this.trackBarPwmDutycycle.Location = new System.Drawing.Point(625, 532);
            this.trackBarPwmDutycycle.Maximum = 100;
            this.trackBarPwmDutycycle.Name = "trackBarPwmDutycycle";
            this.trackBarPwmDutycycle.Size = new System.Drawing.Size(254, 90);
            this.trackBarPwmDutycycle.TabIndex = 3;
            // 
            // label1
            // 
            this.label1.AutoSize = true;
            this.label1.Location = new System.Drawing.Point(52, 77);
            this.label1.Name = "label1";
            this.label1.Size = new System.Drawing.Size(51, 25);
            this.label1.TabIndex = 4;
            this.label1.Text = "Port";
            // 
            // label2
            // 
            this.label2.AutoSize = true;
            this.label2.Location = new System.Drawing.Point(12, 145);
            this.label2.Name = "label2";
            this.label2.Size = new System.Drawing.Size(99, 25);
            this.label2.TabIndex = 4;
            this.label2.Text = "Baudrate";
            // 
            // label3
            // 
            this.label3.AutoSize = true;
            this.label3.Location = new System.Drawing.Point(476, 563);
            this.label3.Name = "label3";
            this.label3.Size = new System.Drawing.Size(155, 25);
            this.label3.TabIndex = 4;
            this.label3.Text = "Duty Cycle (%)";
            // 
            // groupBox1
            // 
            this.groupBox1.Controls.Add(this.buttonEnable);
            this.groupBox1.Controls.Add(this.buttonDisable);
            this.groupBox1.Location = new System.Drawing.Point(63, 379);
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
            this.groupBox2.Location = new System.Drawing.Point(250, 379);
            this.groupBox2.Name = "groupBox2";
            this.groupBox2.Size = new System.Drawing.Size(165, 169);
            this.groupBox2.TabIndex = 5;
            this.groupBox2.TabStop = false;
            this.groupBox2.Text = "Direction";
            // 
            // buttonSerialPortConnect
            // 
            this.buttonSerialPortConnect.Location = new System.Drawing.Point(338, 101);
            this.buttonSerialPortConnect.Name = "buttonSerialPortConnect";
            this.buttonSerialPortConnect.Size = new System.Drawing.Size(141, 55);
            this.buttonSerialPortConnect.TabIndex = 2;
            this.buttonSerialPortConnect.Text = "Connect";
            this.buttonSerialPortConnect.UseVisualStyleBackColor = true;
            this.buttonSerialPortConnect.Click += new System.EventHandler(this.buttonSerialPortConnect_Click);
            // 
            // Form1
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(12F, 25F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.ClientSize = new System.Drawing.Size(944, 753);
            this.Controls.Add(this.buttonSerialPortConnect);
            this.Controls.Add(this.groupBox2);
            this.Controls.Add(this.groupBox1);
            this.Controls.Add(this.label3);
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
        private System.Windows.Forms.Label label3;
        private System.Windows.Forms.GroupBox groupBox1;
        private System.Windows.Forms.GroupBox groupBox2;
        private System.Windows.Forms.Button buttonSerialPortConnect;
    }
}

