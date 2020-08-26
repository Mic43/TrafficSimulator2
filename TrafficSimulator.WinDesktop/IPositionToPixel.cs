using System.Drawing;
using System.Windows.Forms;

namespace TrafficSimulator.WinDesktop
{
    public interface IPositionToPixel
    {
       Point Compute(double x, double y);
    }
}