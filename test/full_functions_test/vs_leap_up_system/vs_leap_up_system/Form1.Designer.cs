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
            ((System.ComponentModel.ISupportInitialize)(this.numericUpDownSfeGoal)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.numericUpDownEfeGoal)).BeginInit();
            this.groupBox1.SuspendLayout();
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
            this.buttonSerialPortConnection.Location = new System.Drawing.Point(34, 80);
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
            this.label1.Location = new System.Drawing.Point(24, 46);
            this.label1.Name = "label1";
            this.label1.Size = new System.Drawing.Size(53, 25);
            this.label1.TabIndex = 4;
            this.label1.Text = "SFE";
            // 
            // label2
            // 
            this.label2.AutoSize = true;
            this.label2.Location = new System.Drawing.Point(24, 113);
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
            this.groupBox1.Location = new System.Drawing.Point(604, 172);
            this.groupBox1.Name = "groupBox1";
            this.groupBox1.Size = new System.Drawing.Size(272, 234);
            this.groupBox1.TabIndex = 5;
            this.groupBox1.TabStop = false;
            this.groupBox1.Text = "Manual";
            // 
            // Form1
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(12F, 25F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.ClientSize = new System.Drawing.Size(988, 681);
            this.Controls.Add(this.groupBox1);
            this.Controls.Add(this.buttonSerialPortConnection);
            this.Controls.Add(this.comboBoxSerialPortName);
            this.Name = "Form1";
            this.Text = "Form1";
            ((System.ComponentModel.ISupportInitialize)(this.numericUpDownSfeGoal)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.numericUpDownEfeGoal)).EndInit();
            this.groupBox1.ResumeLayout(false);
            this.groupBox1.PerformLayout();
            this.ResumeLayout(false);

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
    }
}

