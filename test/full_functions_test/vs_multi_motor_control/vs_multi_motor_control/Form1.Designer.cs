namespace vs_multi_motor_control
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
            this.comboBoxJoints = new System.Windows.Forms.ComboBox();
            this.buttonSendDataPackage = new System.Windows.Forms.Button();
            this.trackBarJointPosition = new System.Windows.Forms.TrackBar();
            this.labelJointPosition = new System.Windows.Forms.Label();
            this.textBoxSerialPortReceivedData = new System.Windows.Forms.TextBox();
            ((System.ComponentModel.ISupportInitialize)(this.trackBarJointPosition)).BeginInit();
            this.SuspendLayout();
            // 
            // comboBoxSerialPortName
            // 
            this.comboBoxSerialPortName.FormattingEnabled = true;
            this.comboBoxSerialPortName.Location = new System.Drawing.Point(62, 51);
            this.comboBoxSerialPortName.Name = "comboBoxSerialPortName";
            this.comboBoxSerialPortName.Size = new System.Drawing.Size(177, 33);
            this.comboBoxSerialPortName.TabIndex = 0;
            this.comboBoxSerialPortName.Click += new System.EventHandler(this.comboBoxSerialPortName_Click);
            // 
            // buttonSerialPortConnection
            // 
            this.buttonSerialPortConnection.Location = new System.Drawing.Point(82, 106);
            this.buttonSerialPortConnection.Name = "buttonSerialPortConnection";
            this.buttonSerialPortConnection.Size = new System.Drawing.Size(140, 56);
            this.buttonSerialPortConnection.TabIndex = 1;
            this.buttonSerialPortConnection.Text = "Dis/Connect";
            this.buttonSerialPortConnection.UseVisualStyleBackColor = true;
            this.buttonSerialPortConnection.Click += new System.EventHandler(this.buttonSerialPortConnection_Click);
            // 
            // comboBoxJoints
            // 
            this.comboBoxJoints.FormattingEnabled = true;
            this.comboBoxJoints.Items.AddRange(new object[] {
            "EFE",
            "SFE"});
            this.comboBoxJoints.Location = new System.Drawing.Point(286, 223);
            this.comboBoxJoints.Name = "comboBoxJoints";
            this.comboBoxJoints.Size = new System.Drawing.Size(121, 33);
            this.comboBoxJoints.TabIndex = 2;
            // 
            // buttonSendDataPackage
            // 
            this.buttonSendDataPackage.Enabled = false;
            this.buttonSendDataPackage.Location = new System.Drawing.Point(563, 210);
            this.buttonSendDataPackage.Name = "buttonSendDataPackage";
            this.buttonSendDataPackage.Size = new System.Drawing.Size(140, 56);
            this.buttonSendDataPackage.TabIndex = 3;
            this.buttonSendDataPackage.Text = "Send";
            this.buttonSendDataPackage.UseVisualStyleBackColor = true;
            this.buttonSendDataPackage.Click += new System.EventHandler(this.buttonSendDataPackage_Click);
            // 
            // trackBarJointPosition
            // 
            this.trackBarJointPosition.Location = new System.Drawing.Point(92, 294);
            this.trackBarJointPosition.Maximum = 4095;
            this.trackBarJointPosition.Name = "trackBarJointPosition";
            this.trackBarJointPosition.Size = new System.Drawing.Size(672, 90);
            this.trackBarJointPosition.TabIndex = 4;
            this.trackBarJointPosition.TickFrequency = 256;
            this.trackBarJointPosition.Value = 2047;
            this.trackBarJointPosition.Scroll += new System.EventHandler(this.trackBarJointPosition_Scroll);
            // 
            // labelJointPosition
            // 
            this.labelJointPosition.AutoSize = true;
            this.labelJointPosition.Location = new System.Drawing.Point(34, 315);
            this.labelJointPosition.Name = "labelJointPosition";
            this.labelJointPosition.Size = new System.Drawing.Size(26, 25);
            this.labelJointPosition.TabIndex = 5;
            this.labelJointPosition.Text = "--";
            // 
            // textBoxSerialPortReceivedData
            // 
            this.textBoxSerialPortReceivedData.Location = new System.Drawing.Point(782, 51);
            this.textBoxSerialPortReceivedData.Multiline = true;
            this.textBoxSerialPortReceivedData.Name = "textBoxSerialPortReceivedData";
            this.textBoxSerialPortReceivedData.ReadOnly = true;
            this.textBoxSerialPortReceivedData.ScrollBars = System.Windows.Forms.ScrollBars.Vertical;
            this.textBoxSerialPortReceivedData.Size = new System.Drawing.Size(648, 666);
            this.textBoxSerialPortReceivedData.TabIndex = 6;
            // 
            // Form1
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(12F, 25F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.ClientSize = new System.Drawing.Size(1471, 737);
            this.Controls.Add(this.textBoxSerialPortReceivedData);
            this.Controls.Add(this.labelJointPosition);
            this.Controls.Add(this.trackBarJointPosition);
            this.Controls.Add(this.buttonSendDataPackage);
            this.Controls.Add(this.comboBoxJoints);
            this.Controls.Add(this.buttonSerialPortConnection);
            this.Controls.Add(this.comboBoxSerialPortName);
            this.Name = "Form1";
            this.Text = "Form1";
            ((System.ComponentModel.ISupportInitialize)(this.trackBarJointPosition)).EndInit();
            this.ResumeLayout(false);
            this.PerformLayout();

        }

        #endregion

        private System.Windows.Forms.ComboBox comboBoxSerialPortName;
        private System.Windows.Forms.Button buttonSerialPortConnection;
        private System.Windows.Forms.ComboBox comboBoxJoints;
        private System.Windows.Forms.Button buttonSendDataPackage;
        private System.Windows.Forms.TrackBar trackBarJointPosition;
        private System.Windows.Forms.Label labelJointPosition;
        private System.Windows.Forms.TextBox textBoxSerialPortReceivedData;
    }
}

