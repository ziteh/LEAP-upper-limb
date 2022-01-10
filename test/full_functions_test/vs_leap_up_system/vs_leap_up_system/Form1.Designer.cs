namespace vs_leap_up_system
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
            this.comboBoxSerialPortName = new System.Windows.Forms.ComboBox();
            this.buttonSerialPortConnection = new System.Windows.Forms.Button();
            this.numericUpDownSfeGoal = new System.Windows.Forms.NumericUpDown();
            this.numericUpDownEfeGoal = new System.Windows.Forms.NumericUpDown();
            this.buttonSerialPortSend = new System.Windows.Forms.Button();
            this.label1 = new System.Windows.Forms.Label();
            this.label2 = new System.Windows.Forms.Label();
            this.groupBox1 = new System.Windows.Forms.GroupBox();
            this.numericUpDownX = new System.Windows.Forms.NumericUpDown();
            this.numericUpDownY = new System.Windows.Forms.NumericUpDown();
            this.textBoxIKsfe = new System.Windows.Forms.TextBox();
            this.textBoxIKefe = new System.Windows.Forms.TextBox();
            this.buttonCopy = new System.Windows.Forms.Button();
            this.buttonXp = new System.Windows.Forms.Button();
            this.buttonXm = new System.Windows.Forms.Button();
            this.buttonYm = new System.Windows.Forms.Button();
            this.buttonYp = new System.Windows.Forms.Button();
            this.numericUpDownRelativeValue = new System.Windows.Forms.NumericUpDown();
            this.textBoxNowPositionEfe = new System.Windows.Forms.TextBox();
            this.textBoxNowPositionSfe = new System.Windows.Forms.TextBox();
            this.label3 = new System.Windows.Forms.Label();
            this.label4 = new System.Windows.Forms.Label();
            this.label5 = new System.Windows.Forms.Label();
            this.textBoxY = new System.Windows.Forms.TextBox();
            this.label6 = new System.Windows.Forms.Label();
            this.textBoxX = new System.Windows.Forms.TextBox();
            this.textBoxForceSensorY = new System.Windows.Forms.TextBox();
            this.textBoxForceSensorX = new System.Windows.Forms.TextBox();
            ((System.ComponentModel.ISupportInitialize)(this.numericUpDownSfeGoal)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.numericUpDownEfeGoal)).BeginInit();
            this.groupBox1.SuspendLayout();
            ((System.ComponentModel.ISupportInitialize)(this.numericUpDownX)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.numericUpDownY)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.numericUpDownRelativeValue)).BeginInit();
            this.SuspendLayout();
            // 
            // comboBoxSerialPortName
            // 
            this.comboBoxSerialPortName.FormattingEnabled = true;
            this.comboBoxSerialPortName.Location = new System.Drawing.Point(34, 32);
            this.comboBoxSerialPortName.Name = "comboBoxSerialPortName";
            this.comboBoxSerialPortName.Size = new System.Drawing.Size(164, 33);
            this.comboBoxSerialPortName.TabIndex = 0;
            this.comboBoxSerialPortName.Click += new System.EventHandler(this.comboBoxSerialPortName_Click);
            // 
            // buttonSerialPortConnection
            // 
            this.buttonSerialPortConnection.Location = new System.Drawing.Point(222, 22);
            this.buttonSerialPortConnection.Name = "buttonSerialPortConnection";
            this.buttonSerialPortConnection.Size = new System.Drawing.Size(164, 50);
            this.buttonSerialPortConnection.TabIndex = 1;
            this.buttonSerialPortConnection.Text = "Dis/Connect";
            this.buttonSerialPortConnection.UseVisualStyleBackColor = true;
            this.buttonSerialPortConnection.Click += new System.EventHandler(this.buttonSerialPortConnection_Click);
            // 
            // numericUpDownSfeGoal
            // 
            this.numericUpDownSfeGoal.Location = new System.Drawing.Point(101, 46);
            this.numericUpDownSfeGoal.Maximum = new decimal(new int[] {
            359,
            0,
            0,
            0});
            this.numericUpDownSfeGoal.Minimum = new decimal(new int[] {
            359,
            0,
            0,
            -2147483648});
            this.numericUpDownSfeGoal.Name = "numericUpDownSfeGoal";
            this.numericUpDownSfeGoal.Size = new System.Drawing.Size(120, 31);
            this.numericUpDownSfeGoal.TabIndex = 2;
            // 
            // numericUpDownEfeGoal
            // 
            this.numericUpDownEfeGoal.Location = new System.Drawing.Point(101, 113);
            this.numericUpDownEfeGoal.Maximum = new decimal(new int[] {
            359,
            0,
            0,
            0});
            this.numericUpDownEfeGoal.Minimum = new decimal(new int[] {
            359,
            0,
            0,
            -2147483648});
            this.numericUpDownEfeGoal.Name = "numericUpDownEfeGoal";
            this.numericUpDownEfeGoal.Size = new System.Drawing.Size(120, 31);
            this.numericUpDownEfeGoal.TabIndex = 2;
            this.numericUpDownEfeGoal.Value = new decimal(new int[] {
            35,
            0,
            0,
            0});
            // 
            // buttonSerialPortSend
            // 
            this.buttonSerialPortSend.Location = new System.Drawing.Point(74, 165);
            this.buttonSerialPortSend.Name = "buttonSerialPortSend";
            this.buttonSerialPortSend.Size = new System.Drawing.Size(147, 50);
            this.buttonSerialPortSend.TabIndex = 3;
            this.buttonSerialPortSend.Text = "Send";
            this.buttonSerialPortSend.UseVisualStyleBackColor = true;
            this.buttonSerialPortSend.Click += new System.EventHandler(this.buttonSerialPortSend_Click);
            // 
            // label1
            // 
            this.label1.AutoSize = true;
            this.label1.Location = new System.Drawing.Point(24, 55);
            this.label1.Name = "label1";
            this.label1.Size = new System.Drawing.Size(53, 25);
            this.label1.TabIndex = 4;
            this.label1.Text = "SFE";
            // 
            // label2
            // 
            this.label2.AutoSize = true;
            this.label2.Location = new System.Drawing.Point(24, 122);
            this.label2.Name = "label2";
            this.label2.Size = new System.Drawing.Size(53, 25);
            this.label2.TabIndex = 4;
            this.label2.Text = "EFE";
            // 
            // groupBox1
            // 
            this.groupBox1.Controls.Add(this.buttonSerialPortSend);
            this.groupBox1.Controls.Add(this.label2);
            this.groupBox1.Controls.Add(this.numericUpDownSfeGoal);
            this.groupBox1.Controls.Add(this.label1);
            this.groupBox1.Controls.Add(this.numericUpDownEfeGoal);
            this.groupBox1.Location = new System.Drawing.Point(34, 113);
            this.groupBox1.Name = "groupBox1";
            this.groupBox1.Size = new System.Drawing.Size(272, 234);
            this.groupBox1.TabIndex = 5;
            this.groupBox1.TabStop = false;
            this.groupBox1.Text = "Manual";
            // 
            // numericUpDownX
            // 
            this.numericUpDownX.DecimalPlaces = 3;
            this.numericUpDownX.Location = new System.Drawing.Point(499, 171);
            this.numericUpDownX.Maximum = new decimal(new int[] {
            1000,
            0,
            0,
            0});
            this.numericUpDownX.Minimum = new decimal(new int[] {
            1000,
            0,
            0,
            -2147483648});
            this.numericUpDownX.Name = "numericUpDownX";
            this.numericUpDownX.Size = new System.Drawing.Size(120, 31);
            this.numericUpDownX.TabIndex = 6;
            this.numericUpDownX.ValueChanged += new System.EventHandler(this.numericUpDownX_ValueChanged);
            // 
            // numericUpDownY
            // 
            this.numericUpDownY.DecimalPlaces = 3;
            this.numericUpDownY.Location = new System.Drawing.Point(499, 208);
            this.numericUpDownY.Maximum = new decimal(new int[] {
            1000,
            0,
            0,
            0});
            this.numericUpDownY.Minimum = new decimal(new int[] {
            1000,
            0,
            0,
            -2147483648});
            this.numericUpDownY.Name = "numericUpDownY";
            this.numericUpDownY.Size = new System.Drawing.Size(120, 31);
            this.numericUpDownY.TabIndex = 6;
            this.numericUpDownY.ValueChanged += new System.EventHandler(this.numericUpDownY_ValueChanged);
            // 
            // textBoxIKsfe
            // 
            this.textBoxIKsfe.Location = new System.Drawing.Point(499, 271);
            this.textBoxIKsfe.Name = "textBoxIKsfe";
            this.textBoxIKsfe.ReadOnly = true;
            this.textBoxIKsfe.Size = new System.Drawing.Size(100, 31);
            this.textBoxIKsfe.TabIndex = 7;
            // 
            // textBoxIKefe
            // 
            this.textBoxIKefe.Location = new System.Drawing.Point(499, 330);
            this.textBoxIKefe.Name = "textBoxIKefe";
            this.textBoxIKefe.ReadOnly = true;
            this.textBoxIKefe.Size = new System.Drawing.Size(100, 31);
            this.textBoxIKefe.TabIndex = 7;
            // 
            // buttonCopy
            // 
            this.buttonCopy.Location = new System.Drawing.Point(499, 396);
            this.buttonCopy.Name = "buttonCopy";
            this.buttonCopy.Size = new System.Drawing.Size(120, 68);
            this.buttonCopy.TabIndex = 8;
            this.buttonCopy.Text = "Copy";
            this.buttonCopy.UseVisualStyleBackColor = true;
            this.buttonCopy.Click += new System.EventHandler(this.buttonCopy_Click);
            // 
            // buttonXp
            // 
            this.buttonXp.Location = new System.Drawing.Point(660, 282);
            this.buttonXp.Name = "buttonXp";
            this.buttonXp.Size = new System.Drawing.Size(71, 40);
            this.buttonXp.TabIndex = 9;
            this.buttonXp.Text = "+X";
            this.buttonXp.UseVisualStyleBackColor = true;
            this.buttonXp.Click += new System.EventHandler(this.buttonXp_Click);
            // 
            // buttonXm
            // 
            this.buttonXm.Location = new System.Drawing.Point(812, 282);
            this.buttonXm.Name = "buttonXm";
            this.buttonXm.Size = new System.Drawing.Size(71, 40);
            this.buttonXm.TabIndex = 9;
            this.buttonXm.Text = "-X";
            this.buttonXm.UseVisualStyleBackColor = true;
            this.buttonXm.Click += new System.EventHandler(this.buttonXm_Click);
            // 
            // buttonYm
            // 
            this.buttonYm.Location = new System.Drawing.Point(732, 334);
            this.buttonYm.Name = "buttonYm";
            this.buttonYm.Size = new System.Drawing.Size(71, 40);
            this.buttonYm.TabIndex = 9;
            this.buttonYm.Text = "-Y";
            this.buttonYm.UseVisualStyleBackColor = true;
            this.buttonYm.Click += new System.EventHandler(this.buttonYm_Click);
            // 
            // buttonYp
            // 
            this.buttonYp.Location = new System.Drawing.Point(732, 227);
            this.buttonYp.Name = "buttonYp";
            this.buttonYp.Size = new System.Drawing.Size(71, 40);
            this.buttonYp.TabIndex = 9;
            this.buttonYp.Text = "+Y";
            this.buttonYp.UseVisualStyleBackColor = true;
            this.buttonYp.Click += new System.EventHandler(this.buttonYp_Click);
            // 
            // numericUpDownRelativeValue
            // 
            this.numericUpDownRelativeValue.Location = new System.Drawing.Point(698, 420);
            this.numericUpDownRelativeValue.Name = "numericUpDownRelativeValue";
            this.numericUpDownRelativeValue.Size = new System.Drawing.Size(80, 31);
            this.numericUpDownRelativeValue.TabIndex = 10;
            this.numericUpDownRelativeValue.Value = new decimal(new int[] {
            10,
            0,
            0,
            0});
            // 
            // textBoxNowPositionEfe
            // 
            this.textBoxNowPositionEfe.Location = new System.Drawing.Point(108, 468);
            this.textBoxNowPositionEfe.Name = "textBoxNowPositionEfe";
            this.textBoxNowPositionEfe.ReadOnly = true;
            this.textBoxNowPositionEfe.Size = new System.Drawing.Size(100, 31);
            this.textBoxNowPositionEfe.TabIndex = 11;
            this.textBoxNowPositionEfe.TextAlign = System.Windows.Forms.HorizontalAlignment.Right;
            // 
            // textBoxNowPositionSfe
            // 
            this.textBoxNowPositionSfe.Location = new System.Drawing.Point(108, 415);
            this.textBoxNowPositionSfe.Name = "textBoxNowPositionSfe";
            this.textBoxNowPositionSfe.ReadOnly = true;
            this.textBoxNowPositionSfe.Size = new System.Drawing.Size(100, 31);
            this.textBoxNowPositionSfe.TabIndex = 11;
            this.textBoxNowPositionSfe.TextAlign = System.Windows.Forms.HorizontalAlignment.Right;
            // 
            // label3
            // 
            this.label3.AutoSize = true;
            this.label3.Location = new System.Drawing.Point(49, 422);
            this.label3.Name = "label3";
            this.label3.Size = new System.Drawing.Size(53, 25);
            this.label3.TabIndex = 4;
            this.label3.Text = "SFE";
            // 
            // label4
            // 
            this.label4.AutoSize = true;
            this.label4.Location = new System.Drawing.Point(49, 474);
            this.label4.Name = "label4";
            this.label4.Size = new System.Drawing.Size(53, 25);
            this.label4.TabIndex = 4;
            this.label4.Text = "EFE";
            // 
            // label5
            // 
            this.label5.AutoSize = true;
            this.label5.Location = new System.Drawing.Point(49, 538);
            this.label5.Name = "label5";
            this.label5.Size = new System.Drawing.Size(26, 25);
            this.label5.TabIndex = 4;
            this.label5.Text = "X";
            // 
            // textBoxY
            // 
            this.textBoxY.Location = new System.Drawing.Point(108, 584);
            this.textBoxY.Name = "textBoxY";
            this.textBoxY.ReadOnly = true;
            this.textBoxY.Size = new System.Drawing.Size(100, 31);
            this.textBoxY.TabIndex = 11;
            this.textBoxY.TextAlign = System.Windows.Forms.HorizontalAlignment.Right;
            // 
            // label6
            // 
            this.label6.AutoSize = true;
            this.label6.Location = new System.Drawing.Point(49, 590);
            this.label6.Name = "label6";
            this.label6.Size = new System.Drawing.Size(27, 25);
            this.label6.TabIndex = 4;
            this.label6.Text = "Y";
            // 
            // textBoxX
            // 
            this.textBoxX.Location = new System.Drawing.Point(108, 531);
            this.textBoxX.Name = "textBoxX";
            this.textBoxX.ReadOnly = true;
            this.textBoxX.Size = new System.Drawing.Size(100, 31);
            this.textBoxX.TabIndex = 11;
            this.textBoxX.TextAlign = System.Windows.Forms.HorizontalAlignment.Right;
            // 
            // textBoxForceSensorY
            // 
            this.textBoxForceSensorY.Location = new System.Drawing.Point(243, 584);
            this.textBoxForceSensorY.Name = "textBoxForceSensorY";
            this.textBoxForceSensorY.ReadOnly = true;
            this.textBoxForceSensorY.Size = new System.Drawing.Size(100, 31);
            this.textBoxForceSensorY.TabIndex = 11;
            this.textBoxForceSensorY.TextAlign = System.Windows.Forms.HorizontalAlignment.Right;
            // 
            // textBoxForceSensorX
            // 
            this.textBoxForceSensorX.Location = new System.Drawing.Point(243, 531);
            this.textBoxForceSensorX.Name = "textBoxForceSensorX";
            this.textBoxForceSensorX.ReadOnly = true;
            this.textBoxForceSensorX.Size = new System.Drawing.Size(100, 31);
            this.textBoxForceSensorX.TabIndex = 11;
            this.textBoxForceSensorX.TextAlign = System.Windows.Forms.HorizontalAlignment.Right;
            // 
            // Form1
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(12F, 25F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.ClientSize = new System.Drawing.Size(988, 681);
            this.Controls.Add(this.textBoxForceSensorX);
            this.Controls.Add(this.textBoxX);
            this.Controls.Add(this.label6);
            this.Controls.Add(this.textBoxForceSensorY);
            this.Controls.Add(this.textBoxNowPositionSfe);
            this.Controls.Add(this.textBoxY);
            this.Controls.Add(this.label4);
            this.Controls.Add(this.label5);
            this.Controls.Add(this.textBoxNowPositionEfe);
            this.Controls.Add(this.label3);
            this.Controls.Add(this.numericUpDownRelativeValue);
            this.Controls.Add(this.buttonYm);
            this.Controls.Add(this.buttonYp);
            this.Controls.Add(this.buttonXm);
            this.Controls.Add(this.buttonXp);
            this.Controls.Add(this.buttonCopy);
            this.Controls.Add(this.textBoxIKefe);
            this.Controls.Add(this.textBoxIKsfe);
            this.Controls.Add(this.numericUpDownY);
            this.Controls.Add(this.numericUpDownX);
            this.Controls.Add(this.groupBox1);
            this.Controls.Add(this.buttonSerialPortConnection);
            this.Controls.Add(this.comboBoxSerialPortName);
            this.Name = "Form1";
            this.Text = "LEAP-Up (Disconnected)";
            ((System.ComponentModel.ISupportInitialize)(this.numericUpDownSfeGoal)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.numericUpDownEfeGoal)).EndInit();
            this.groupBox1.ResumeLayout(false);
            this.groupBox1.PerformLayout();
            ((System.ComponentModel.ISupportInitialize)(this.numericUpDownX)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.numericUpDownY)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.numericUpDownRelativeValue)).EndInit();
            this.ResumeLayout(false);
            this.PerformLayout();

        }

        #endregion

        private System.Windows.Forms.ComboBox comboBoxSerialPortName;
        private System.Windows.Forms.Button buttonSerialPortConnection;
        private System.Windows.Forms.NumericUpDown numericUpDownSfeGoal;
        private System.Windows.Forms.NumericUpDown numericUpDownEfeGoal;
        private System.Windows.Forms.Button buttonSerialPortSend;
        private System.Windows.Forms.Label label1;
        private System.Windows.Forms.Label label2;
        private System.Windows.Forms.GroupBox groupBox1;
        private System.Windows.Forms.NumericUpDown numericUpDownX;
        private System.Windows.Forms.NumericUpDown numericUpDownY;
        private System.Windows.Forms.TextBox textBoxIKsfe;
        private System.Windows.Forms.TextBox textBoxIKefe;
        private System.Windows.Forms.Button buttonCopy;
        private System.Windows.Forms.Button buttonXp;
        private System.Windows.Forms.Button buttonXm;
        private System.Windows.Forms.Button buttonYm;
        private System.Windows.Forms.Button buttonYp;
        private System.Windows.Forms.NumericUpDown numericUpDownRelativeValue;
        private System.Windows.Forms.TextBox textBoxNowPositionEfe;
        private System.Windows.Forms.TextBox textBoxNowPositionSfe;
        private System.Windows.Forms.Label label3;
        private System.Windows.Forms.Label label4;
        private System.Windows.Forms.Label label5;
        private System.Windows.Forms.TextBox textBoxY;
        private System.Windows.Forms.Label label6;
        private System.Windows.Forms.TextBox textBoxX;
        private System.Windows.Forms.TextBox textBoxForceSensorY;
        private System.Windows.Forms.TextBox textBoxForceSensorX;
    }
}

