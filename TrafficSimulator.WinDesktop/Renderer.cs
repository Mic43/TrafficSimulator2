using System;
using System.Collections.Generic;
using System.Drawing;
using System.Drawing.Drawing2D;
using System.Text;
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
            RenderVehicles();

            void RenderConnectionsGraph()
            {
                var crossings = ConnectionsGraphModule.crossings(simulationState.ConnectionsGraph);
                foreach (var item in crossings)
                {
                    Position2d pos = item.Value.Position.Item;
                    Point point = PositionToPixel.Compute(pos.X, pos.Y);

                    int radius = 4;
                    graphics.FillEllipse(Brushes.Red, point.X - radius, point.Y - radius,
                       radius * 2, radius * 2);
                    foreach (var connection in ConnectionsGraphModule.crossingOutputs(simulationState.ConnectionsGraph,
                                                                                      item.Key))
                    {
                        RenderConnection(connection);
                    }
                }

                void RenderConnection(Connection connection)
                {
                    Position2d start = crossings[connection.StartId].Position.Item;
                    Position2d end = crossings[connection.EndId].Position.Item;

                    var startPos = PositionToPixel.Compute(start.X, start.Y);
                    var endPos = PositionToPixel.Compute(end.X, end.Y);

                    if (connection.ConnectionType.IsLinear)
                    {
                        graphics.DrawLine(Pens.Black, startPos,
                                          endPos);
                    }
                    else if (connection.ConnectionType.IsQuadraticBezier)
                    {
                        var controlPoint = ((ConnectionType.QuadraticBezier)connection.ConnectionType).Item.Item;
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
            void RenderVehicles()
            {
                foreach (var vehicle in simulationState.Vehicles)
                {
                    var pos = TrafficSimulator.Core.Api.getVehiclePosition.Invoke(simulationState.ConnectionsGraph).Invoke(vehicle);
                    Point point = PositionToPixel.Compute(pos.X, pos.Y);
                    int radius = 3;
                    graphics.FillEllipse(Brushes.Green, point.X - radius, point.Y - radius,
                        radius * 2, radius * 2);
                }

            }
        }
        public IPositionToPixel PositionToPixel { get; }
    }
}
