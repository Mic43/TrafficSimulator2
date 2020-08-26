using System.Drawing;

namespace TrafficSimulator.WinDesktop
{
    class PositionToPixel : IPositionToPixel
    {
        private readonly double scale;

        public PositionToPixel(double unitInPixels)
        {
            this.scale = unitInPixels;
        }

        public Point Compute(double x, double y)
        {
            return new Point((int)(scale * x), (int)(scale * y));
        }
    }
}