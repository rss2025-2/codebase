from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker

class VisualizationTools:

    @staticmethod
    def plot_line(x, y, publisher, color = (1., 0., 0.), frame = "/base_link"):
        """
        Publishes the points (x, y) to publisher
        so they can be visualized in rviz as
        connected line segments.
        Args:
            x, y: The x and y values. These arrays
            must be of the same length.
            publisher: the publisher to publish to. The
            publisher must be of type Marker from the
            visualization_msgs.msg class.
            color: the RGB color of the plot.
            frame: the transformation frame to plot in.
        """
        # Construct a line
        line_strip = Marker()
        line_strip.type = Marker.LINE_STRIP
        line_strip.header.frame_id = frame

        # Set the size and color
        line_strip.scale.x = 0.1
        line_strip.scale.y = 0.1
        line_strip.color.a = 1.
        line_strip.color.r = color[0]
        line_strip.color.g = color[1]
        line_strip.color.b = color[2]

        # Fill the line with the desired values
        for xi, yi in zip(x, y):
            p = Point()
            p.x = xi
            p.y = yi
            line_strip.points.append(p)

        # Publish the line
        publisher.publish(line_strip)
    
    @staticmethod
    def visualize_wall(a, b, c, publisher, color = (0.0, 0.0, 1.0), frame = '/base_link'):
        if(abs(b) > 0.0001):
            VisualizationTools.plot_line([-5.0,5.0],[-(a*(-5.0)+c)/b,-(a*5.0+c)/b], publisher, color, frame)
        else:
            VisualizationTools.plot_line([-c/a, -c/a],[-5.0, 5.0], publisher, color, frame)