import rospy
from visualization_msgs.msg import Marker, MarkerArray

class MarkerPublisher:
    def __init__(self):
        self.marker_publisher = rospy.Publisher('/blocks', MarkerArray, queue_size=10)


        self.colors = [(0.6039215686274509,0.6784313725490196,0.7490196078431373), 
                       (0.42745098039215684,0.596078431372549,0.7294117647058823),
                       (0.8274509803921568,0.7254901960784313,0.6235294117647059),
                       (0.7568627450980392,0.4666666666666667,0.403921568627451)]

    def publish_marker(self, blocks_list):
        marker_array = MarkerArray()
        for n in range(len(blocks_list)):
            sTask = blocks_list[n]

            marker = Marker()
            marker.header.frame_id = "map"  # Set the frame in which the marker will be displayed
            marker.header.stamp = rospy.Time.now()
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x = sTask.x  # Set the position of the marker
            marker.pose.position.y = sTask.y
            marker.pose.position.z = 0
            marker.pose.orientation.x = 0  # Set the orientation of the marker
            marker.pose.orientation.y = 0
            marker.pose.orientation.z = 0
            marker.pose.orientation.w = 1
            marker.scale.x = 0.2  # Set the scale of the marker (size)
            marker.scale.y = 0.2
            marker.scale.z = 1
            marker.color.a = 1.0  # Set the alpha (transparency) of the marker
            marker.color.r = self.colors[n][0]  # Set the color of the marker (red)
            marker.color.g = self.colors[n][1]
            marker.color.b = self.colors[n][2]
            marker.id = n
            marker_array.markers.append(marker)

        # Publish the marker
        self.marker_publisher.publish(marker_array)


class LinePublisher:
    def __init__(self, topic_name:str):
        self.line_publisher = rospy.Publisher(topic_name, Marker, queue_size=10)

    def draw_line(self, point1, point2):
        marker = Marker()
        marker.header.frame_id = "map"  # Set the frame in which the points are defined
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.025  # Line width

        # Set the points
        marker.points.append(point1)
        marker.points.append(point2)

        marker.color.a = 1.0  # Alpha
        marker.color.r = 0.17254901960784313  # Red
        marker.color.g = 0.25882352941176473  # Green
        marker.color.b = 0.3176470588235294  # Blue

        self.line_publisher.publish(marker)