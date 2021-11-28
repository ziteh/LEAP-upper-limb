namespace vs_communication_package_sender
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
            this.numericUpDownBaudrate = new System.Windows.Forms.NumericUpDown();
            this.buttonConnection = new System.Windows.Forms.Button();
            this.buttonSend = new System.Windows.Forms.Button();
            this.groupBox1 = new System.Windows.Forms.GroupBox();
            this.radioButtonMotorEnable = new System.Windows.Forms.RadioButton();
            this.radioButtonMotorDisable = new System.Windows.Forms.RadioButton();
            this.radioButtonMotorEnableToggle = new System.Windows.Forms.RadioButton();
            this.groupBox2 = new System.Windows.Forms.GroupBox();
            this.radioButtonDirToggle = new System.Windows.Forms.RadioButton();
            this.radioButtonDirCCW = new System.Windows.Forms.RadioButton();
            this.radioButtonDicCW = new System.Windows.Forms.RadioButton();
            this.numericUpDownMotorID = new System.Windows.Forms.NumericUpDown();
            this.numericUpDownMotorPosition = new System.Windows.Forms.NumericUpDown();
            this.textBoxSendPreview = new System.Windows.Forms.TextBox();
            ((System.ComponentModel.ISupportInitialize)(this.numericUpDownBaudrate)).BeginInit();
            this.groupBox1.SuspendLayout();
            this.groupBox2.SuspendLayout();
            ((System.ComponentModel.ISupportInitialize)(this.numericUpDownMotorID)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.numericUpDownMotorPosition)).BeginInit();
            this.SuspendLayout();
            // 
            // comboBoxSerialPorts
            // 
            this.comboBoxSerialPorts.FormattingEnabled = true;
            this.comboBoxSerialPorts.Location = new System.Drawing.Point(63, 42);
            this.comboBoxSerialPorts.Name = "comboBoxSerialPorts";
            this.comboBoxSerialPorts.Size = new System.Drawing.Size(121, 33);
            this.comboBoxSerialPorts.TabIndex = 0;
            this.comboBoxSerialPorts.Click += new System.EventHandler(this.comboBoxSerialPorts_Click);
            // 
            // numericUpDownBaudrate
            // 
            this.numericUpDownBaudrate.Location = new System.Drawing.Point(63, 99);
            this.numericUpDownBaudrate.Maximum = new decimal(new int[] {
            99999,
            0,
            0,
            0});
            this.numericUpDownBaudrate.Name = "numericUpDownBaudrate";
            this.numericUpDownBaudrate.Size = new System.Drawing.Size(186, 31);
            this.numericUpDownBaudrate.TabIndex = 1;
            this.numericUpDownBaudrate.Value = new decimal(new int[] {
            9600,
            0,
            0,
            0});
            // 
            // buttonConnection
            // 
            this.buttonConnection.Location = new System.Drawing.Point(63, 159);
            this.buttonConnection.Name = "buttonConnection";
            this.buttonConnection.Size = new System.Drawing.Size(162, 71);
            this.buttonConnection.TabIndex = 2;
            this.buttonConnection.Text = "Connect";
            this.buttonConnection.UseVisualStyleBackColor = true;
            this.buttonConnection.Click += new System.EventHandler(this.buttonConnect_Click);
            // 
            // buttonSend
            // 
            this.buttonSend.Location = new System.Drawing.Point(398, 423);
            this.buttonSend.Name = "buttonSend";
            this.buttonSend.Size = new System.Drawing.Size(162, 71);
            this.buttonSend.TabIndex = 2;
            this.buttonSend.Text = "Send";
            this.buttonSend.UseVisualStyleBackColor = true;
            this.buttonSend.Click += new System.EventHandler(this.buttonSend_Click);
            // 
            // groupBox1
            // 
            this.groupBox1.Controls.Add(this.radioButtonMotorEnableToggle);
            this.groupBox1.Controls.Add(this.radioButtonMotorDisable);
            this.groupBox1.Controls.Add(this.radioButtonMotorEnable);
            this.groupBox1.Location = new System.Drawing.Point(412, 75);
            this.groupBox1.Name = "groupBox1";
            this.groupBox1.Size = new System.Drawing.Size(186, 184);
            this.groupBox1.TabIndex = 3;
            this.groupBox1.TabStop = false;
            this.groupBox1.Text = "Enable";
            // 
            // radioButtonMotorEnable
            // 
            this.radioButtonMotorEnable.AutoSize = true;
            this.radioButtonMotorEnable.Location = new System.Drawing.Point(38, 52);
            this.radioButtonMotorEnable.Name = "radioButtonMotorEnable";
            this.radioButtonMotorEnable.Size = new System.Drawing.Size(110, 29);
            this.radioButtonMotorEnable.TabIndex = 0;
            this.radioButtonMotorEnable.Text = "Enable";
            this.radioButtonMotorEnable.UseVisualStyleBackColor = true;
            // 
            // radioButtonMotorDisable
            // 
            this.radioButtonMotorDisable.AutoSize = true;
            this.radioButtonMotorDisable.Checked = true;
            this.radioButtonMotorDisable.Location = new System.Drawing.Point(38, 87);
            this.radioButtonMotorDisable.Name = "radioButtonMotorDisable";
            this.radioButtonMotorDisable.Size = new System.Drawing.Size(115, 29);
            this.radioButtonMotorDisable.TabIndex = 0;
            this.radioButtonMotorDisable.TabStop = true;
            this.radioButtonMotorDisable.Text = "Disable";
            this.radioButtonMotorDisable.UseVisualStyleBackColor = true;
            // 
            // radioButtonMotorEnableToggle
            // 
            this.radioButtonMotorEnableToggle.AutoSize = true;
            this.radioButtonMotorEnableToggle.Location = new System.Drawing.Point(38, 122);
            this.radioButtonMotorEnableToggle.Name = "radioButtonMotorEnableToggle";
            this.radioButtonMotorEnableToggle.Size = new System.Drawing.Size(109, 29);
            this.radioButtonMotorEnableToggle.TabIndex = 0;
            this.radioButtonMotorEnableToggle.Text = "Toggle";
            this.radioButtonMotorEnableToggle.UseVisualStyleBackColor = true;
            // 
            // groupBox2
            // 
            this.groupBox2.Controls.Add(this.radioButtonDirToggle);
            this.groupBox2.Controls.Add(this.radioButtonDirCCW);
            this.groupBox2.Controls.Add(this.radioButtonDicCW);
            this.groupBox2.Location = new System.Drawing.Point(629, 75);
            this.groupBox2.Name = "groupBox2";
            this.groupBox2.Size = new System.Drawing.Size(186, 184);
            this.groupBox2.TabIndex = 3;
            this.groupBox2.TabStop = false;
            this.groupBox2.Text = "Dircetion";
            // 
            // radioButtonDirToggle
            // 
            this.radioButtonDirToggle.AutoSize = true;
            this.radioButtonDirToggle.Location = new System.Drawing.Point(24, 122);
            this.radioButtonDirToggle.Name = "radioButtonDirToggle";
            this.radioButtonDirToggle.Size = new System.Drawing.Size(109, 29);
            this.radioButtonDirToggle.TabIndex = 0;
            this.radioButtonDirToggle.Text = "Toggle";
            this.radioButtonDirToggle.UseVisualStyleBackColor = true;
            // 
            // radioButtonDirCCW
            // 
            this.radioButtonDirCCW.AutoSize = true;
            this.radioButtonDirCCW.Location = new System.Drawing.Point(24, 87);
            this.radioButtonDirCCW.Name = "radioButtonDirCCW";
            this.radioButtonDirCCW.Size = new System.Drawing.Size(93, 29);
            this.radioButtonDirCCW.TabIndex = 0;
            this.radioButtonDirCCW.Text = "CCW";
            this.radioButtonDirCCW.UseVisualStyleBackColor = true;
            // 
            // radioButtonDicCW
            // 
            this.radioButtonDicCW.AutoSize = true;
            this.radioButtonDicCW.Checked = true;
            this.radioButtonDicCW.Location = new System.Drawing.Point(24, 52);
            this.radioButtonDicCW.Name = "radioButtonDicCW";
            this.radioButtonDicCW.Size = new System.Drawing.Size(78, 29);
            this.radioButtonDicCW.TabIndex = 0;
            this.radioButtonDicCW.TabStop = true;
            this.radioButtonDicCW.Text = "CW";
            this.radioButtonDicCW.UseVisualStyleBackColor = true;
            // 
            // numericUpDownMotorID
            // 
            this.numericUpDownMotorID.Location = new System.Drawing.Point(412, 292);
            this.numericUpDownMotorID.Name = "numericUpDownMotorID";
            this.numericUpDownMotorID.Size = new System.Drawing.Size(186, 31);
            this.numericUpDownMotorID.TabIndex = 1;
            // 
            // numericUpDownMotorPosition
            // 
            this.numericUpDownMotorPosition.Location = new System.Drawing.Point(629, 292);
            this.numericUpDownMotorPosition.Maximum = new decimal(new int[] {
            4095,
            0,
            0,
            0});
            this.numericUpDownMotorPosition.Name = "numericUpDownMotorPosition";
            this.numericUpDownMotorPosition.Size = new System.Drawing.Size(186, 31);
            this.numericUpDownMotorPosition.TabIndex = 1;
            // 
            // textBoxSendPreview
            // 
            this.textBoxSendPreview.Location = new System.Drawing.Point(389, 360);
            this.textBoxSendPreview.Name = "textBoxSendPreview";
            this.textBoxSendPreview.ReadOnly = true;
            this.textBoxSendPreview.Size = new System.Drawing.Size(426, 31);
            this.textBoxSendPreview.TabIndex = 4;
            // 
            // Form1
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(12F, 25F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.ClientSize = new System.Drawing.Size(647, 301);
            this.Controls.Add(this.textBoxSendPreview);
            this.Controls.Add(this.groupBox2);
            this.Controls.Add(this.groupBox1);
            this.Controls.Add(this.buttonSend);
            this.Controls.Add(this.buttonConnection);
            this.Controls.Add(this.numericUpDownMotorPosition);
            this.Controls.Add(this.numericUpDownMotorID);
            this.Controls.Add(this.numericUpDownBaudrate);
            this.Controls.Add(this.comboBoxSerialPorts);
            this.Name = "Form1";
            this.Text = "Communication";
            ((System.ComponentModel.ISupportInitialize)(this.numericUpDownBaudrate)).EndInit();
            this.groupBox1.ResumeLayout(false);
            this.groupBox1.PerformLayout();
            this.groupBox2.ResumeLayout(false);
            this.groupBox2.PerformLayout();
            ((System.ComponentModel.ISupportInitialize)(this.numericUpDownMotorID)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.numericUpDownMotorPosition)).EndInit();
            this.ResumeLayout(false);
            this.PerformLayout();

        }

        #endregion

        private System.Windows.Forms.ComboBox comboBoxSerialPorts;
        private System.Windows.Forms.NumericUpDown numericUpDownBaudrate;
        private System.Windows.Forms.Button buttonConnection;
        private System.Windows.Forms.Button buttonSend;
        private System.Windows.Forms.GroupBox groupBox1;
        private System.Windows.Forms.RadioButton radioButtonMotorEnableToggle;
        private System.Windows.Forms.RadioButton radioButtonMotorDisable;
        private System.Windows.Forms.RadioButton radioButtonMotorEnable;
        private System.Windows.Forms.GroupBox groupBox2;
        private System.Windows.Forms.RadioButton radioButtonDirToggle;
        private System.Windows.Forms.RadioButton radioButtonDirCCW;
        private System.Windows.Forms.RadioButton radioButtonDicCW;
        private System.Windows.Forms.NumericUpDown numericUpDownMotorID;
        private System.Windows.Forms.NumericUpDown numericUpDownMotorPosition;
        private System.Windows.Forms.TextBox textBoxSendPreview;
    }
}

