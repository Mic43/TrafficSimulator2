using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Data.Common;
using System.Drawing;
using System.Linq;
using System.Reflection;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;
using TrafficSimulator.Core;
using static TrafficSimulator.Core.BaseTypes;
using static TrafficSimulator.Core.DomainModel;

namespace TrafficSimulator.WinDesktop
{
    public partial class Main : Form
    {
        private SimulationState _simulationState;
        private Renderer renderer;
        private Timer _logTimer = new Timer() {Interval = 500};
        public Main()
        {                   
            InitializeComponent();
            MakePanelDoubleBuffered();


            renderer = new Renderer(new PositionToPixel(100));
            _logTimer.Tick += LogTimer_Tick;
        }

        private void MakePanelDoubleBuffered()
        {
            typeof(Panel).InvokeMember("DoubleBuffered", BindingFlags.SetProperty
                                                         | BindingFlags.Instance | BindingFlags.NonPublic, null,
                panelBoard, new object[] {true});
        }

        private void LogTimer_Tick(object sender, EventArgs e)
        {
            WriteToLog(_simulationState);
        }

        private void buttonGo_Click(object sender, EventArgs e)
        {
            timerSimulation.Start();
            _logTimer.Start();
        }

        private void Main_Shown(object sender, EventArgs e)
        {
            _simulationState = Api.runQuery(Api.Query.Init);
        }

        private void panelBoard_Paint(object sender, PaintEventArgs e)
        {
            renderer.Render(e.Graphics, _simulationState);
        }

        private void timerSimulation_Tick(object sender, EventArgs e)
        {
            _simulationState = Api.handleCommand(Api.Command.NewUpdate(TimeIntervalModule.create (TimeSpan.FromMilliseconds(timerSimulation.Interval).TotalSeconds)), _simulationState);
            panelBoard.Refresh();
        }

        private void WriteToLog(SimulationState simulationState)
        {
            Func<string, Vehicle, string> vehicleLog = (s, vehicle) => $"{s}{Environment.NewLine} " +
                                                                       $"Speed: {vehicle.CurrentMotionParams.Speed:F1} " +
                                                                       $"\t Acceleration: {vehicle.CurrentMotionParams.Acceleration:F1}";
            var s = simulationState.Vehicles.Aggregate(string.Empty,vehicleLog);
            richTextBoxLog.Text = $@"{Environment.NewLine}{s}";
        }
    }



}
