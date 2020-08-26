using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Data.Common;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;
using TrafficSimulator.Core;
using static TrafficSimulator.Core.DomainModel;

namespace TrafficSimulator.WinDesktop
{
    public partial class Main : Form
    {
        private SimulationState _simulationState;
        private Renderer renderer;
        public Main()
        {                   
            InitializeComponent();
            renderer = new Renderer(new PositionToPixel(100));
        }   

        private void buttonGo_Click(object sender, EventArgs e)
        {
            timerSimulation.Start();
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
            _simulationState = Api.handleCommand(Api.Command.NewUpdate(TimeInterval.NewTimeInterval(TimeSpan.FromMilliseconds(timerSimulation.Interval).TotalSeconds)), _simulationState);
            panelBoard.Refresh();
            WriteToLog(_simulationState);
        }

        private void WriteToLog(SimulationState simulationState)
        {
            var s = simulationState.Vehicles.Aggregate(string.Empty,(s, vehicle) => $"{s}{Environment.NewLine} \t Speed: {vehicle.CurrentMotionParams.Speed:F1}");
            richTextBoxLog.Text = $@"{Environment.NewLine}{s}";
        }
    }



}
