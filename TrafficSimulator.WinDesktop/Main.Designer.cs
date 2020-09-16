namespace TrafficSimulator.WinDesktop
{
    partial class Main
    {
        /// <summary>
        ///  Required designer variable.
        /// </summary>
        private System.ComponentModel.IContainer components = null;

        /// <summary>
        ///  Clean up any resources being used.
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
        ///  Required method for Designer support - do not modify
        ///  the contents of this method with the code editor.
        /// </summary>
        private void InitializeComponent()
        {
            this.components = new System.ComponentModel.Container();
            this.panelBoard = new System.Windows.Forms.Panel();
            this.richTextBoxLog = new System.Windows.Forms.RichTextBox();
            this.panel2 = new System.Windows.Forms.Panel();
            this.buttonGo = new System.Windows.Forms.Button();
            this.timerSimulation = new System.Windows.Forms.Timer(this.components);
            this.panelBoard.SuspendLayout();
            this.panel2.SuspendLayout();
            this.SuspendLayout();
            // 
            // panelBoard
            // 
            this.panelBoard.Controls.Add(this.richTextBoxLog);
            this.panelBoard.Controls.Add(this.panel2);
            this.panelBoard.Dock = System.Windows.Forms.DockStyle.Fill;
            this.panelBoard.Location = new System.Drawing.Point(0, 0);
            this.panelBoard.Name = "panelBoard";
            this.panelBoard.Size = new System.Drawing.Size(967, 614);
            this.panelBoard.TabIndex = 0;
            this.panelBoard.Paint += new System.Windows.Forms.PaintEventHandler(this.panelBoard_Paint);
            // 
            // richTextBoxLog
            // 
            this.richTextBoxLog.Dock = System.Windows.Forms.DockStyle.Right;
            this.richTextBoxLog.Location = new System.Drawing.Point(716, 0);
            this.richTextBoxLog.Name = "richTextBoxLog";
            this.richTextBoxLog.Size = new System.Drawing.Size(251, 564);
            this.richTextBoxLog.TabIndex = 1;
            this.richTextBoxLog.Text = "";
            // 
            // panel2
            // 
            this.panel2.Controls.Add(this.buttonGo);
            this.panel2.Dock = System.Windows.Forms.DockStyle.Bottom;
            this.panel2.Location = new System.Drawing.Point(0, 564);
            this.panel2.Name = "panel2";
            this.panel2.Size = new System.Drawing.Size(967, 50);
            this.panel2.TabIndex = 0;
            // 
            // buttonGo
            // 
            this.buttonGo.Location = new System.Drawing.Point(880, 14);
            this.buttonGo.Name = "buttonGo";
            this.buttonGo.Size = new System.Drawing.Size(75, 23);
            this.buttonGo.TabIndex = 0;
            this.buttonGo.Text = "buttonGo";
            this.buttonGo.UseVisualStyleBackColor = true;
            this.buttonGo.Click += new System.EventHandler(this.buttonGo_Click);
            // 
            // timerSimulation
            // 
            this.timerSimulation.Interval = 10;
            this.timerSimulation.Tick += new System.EventHandler(this.timerSimulation_Tick);
            // 
            // Main
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(7F, 15F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.ClientSize = new System.Drawing.Size(967, 614);
            this.Controls.Add(this.panelBoard);
            this.Name = "Main";
            this.Text = "Form1";
            this.Shown += new System.EventHandler(this.Main_Shown);
            this.panelBoard.ResumeLayout(false);
            this.panel2.ResumeLayout(false);
            this.ResumeLayout(false);

        }

        #endregion

        private System.Windows.Forms.Panel panelBoard;
        private System.Windows.Forms.Panel panel2;
        private System.Windows.Forms.Button buttonGo;
        private System.Windows.Forms.Timer timerSimulation;
        private System.Windows.Forms.RichTextBox richTextBoxLog;
    }
}

