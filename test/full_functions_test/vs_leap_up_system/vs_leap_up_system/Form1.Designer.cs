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
            // Form1
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(12F, 25F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.ClientSize = new System.Drawing.Size(800, 450);
            this.Controls.Add(this.buttonSerialPortConnection);
            this.Controls.Add(this.comboBoxSerialPortName);
            this.Name = "Form1";
            this.Text = "Form1";
            this.ResumeLayout(false);

        }

        #endregion

        private System.Windows.Forms.ComboBox comboBoxSerialPortName;
        private System.Windows.Forms.Button buttonSerialPortConnection;
    }
}

