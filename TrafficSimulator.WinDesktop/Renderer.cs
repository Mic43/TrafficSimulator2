using System;
using System.Collections.Generic;
using System.Drawing;
using System.Drawing.Drawing2D;
using System.Linq;
using System.Text;
using TrafficSimulator.Core;
using static TrafficSimulator.Core.DomainModel;

namespace TrafficSimulator.WinDesktop
{
    class Renderer
    {
        public Renderer(IPositionToPixel positionToPixel)
        {
            PositionToPixel = positionToPixel ?? throw new ArgumentNullException(nameof(positionToPixel));
        }

        public void Render(Graphics graphics, SimulationState simulationState)
        {
            if (graphics is null)
            {
                throw new ArgumentNullException(nameof(graphics));
            }
            if (simulationState is null)
            {
                throw new ArgumentNullException(nameof(simulationState));
            }

            RenderConnectionsGraph();
            RenderTrafficLights();
            RenderVehicles();

            void RenderConnectionsGraph()
            {
                var crossings = ConnectionsGraphModule.crossings(simulationState.ConnectionsGraph);
                foreach (var item in crossings)
                {
                    BaseTypes.Position2d pos = item.Value.Position.Item;
                    Point point = PositionToPixel.Compute(pos.X, pos.Y);

                    int radius = 4;
                    graphics.FillEllipse(Brushes.Black, point.X - radius, point.Y - radius,
                       radius * 2, radius * 2);
                    foreach (var connection in ConnectionsGraphModule.crossingOutputs(simulationState.ConnectionsGraph,
                                                                                      item.Key))
                    {
                        RenderConnection(connection);
                    }
                }

                void RenderConnection(Connection connection)
                {
                    BaseTypes.Position2d start = crossings[connection.StartId].Position.Item;
                    BaseTypes.Position2d end = crossings[connection.EndId].Position.Item;

                    var startPos = PositionToPixel.Compute(start.X, start.Y);
                    var endPos = PositionToPixel.Compute(end.X, end.Y);

                    if (connection.ConnectionType.IsLinear)
                    {
                        graphics.DrawLine(Pens.Black, startPos,
                                          endPos);
                    }
                    else if (connection.ConnectionType.IsQuadraticBezier)
                    {
                        var controlPoint = ((ConnectionType.QuadraticBezier) connection.ConnectionType).Item
                            .controlPoint.Item;
                        var controlPointPos = PositionToPixel.Compute(controlPoint.X, controlPoint.Y);

                        var p2 = new Point(
                            (int)(controlPointPos.X * 2.0 / 3 + startPos.X * 1.0 / 3),
                            (int)(controlPointPos.Y * 2.0 / 3 + startPos.Y * 1.0 / 3));
                        var p3 = new Point(
                            (int)(controlPointPos.X * 2.0 / 3 + endPos.X * 1.0 / 3),
                            (int)(controlPointPos.Y * 2.0 / 3 + endPos.Y * 1.0 / 3));

                        graphics.DrawBezier(Pens.Black, startPos, p2,p3,endPos);
                      
                    }
                }
            }

            void RenderTrafficLights()
            {
                foreach (var lightSystem in simulationState.TrafficLights)
                {
                    foreach (var trafficLight in lightSystem.getAllTrafficLights())
                    {
                        var pos = Api.locationToPosition.Invoke(simulationState.ConnectionsGraph).Invoke(trafficLight.Location);
                        Point point = PositionToPixel.Compute(pos.X, pos.Y);
                        int radius = 4;
                        graphics.FillEllipse(trafficLight.State.IsGreen ? Brushes.Green : Brushes.Red, 
                            point.X - radius, point.Y - radius,
                            radius * 2, radius * 2);
                    }
                }
            }
            void RenderVehicles()
            {
                foreach (var vehicle in simulationState.Vehicles)
                {
                    var pos = Api.locationToPosition.Invoke(simulationState.ConnectionsGraph).Invoke(vehicle.Location);
                    Point point = PositionToPixel.Compute(pos.X, pos.Y);
                    int radius = 3;
                    graphics.FillRectangle(Brushes.Blue, point.X - radius, point.Y - radius,
                        radius * 2, radius * 2);
                }

            }
        }
        public IPositionToPixel PositionToPixel { get; }
    }
}
