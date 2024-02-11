import rospy
from visualization_msgs.msg import Marker, MarkerArray

class MarkerPublisher:
    def __init__(self):
        self.marker_publisher = rospy.Publisher('/blocks', MarkerArray, queue_size=10)

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
            marker.color.r = 1.0  # Set the color of the marker (red)
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.id = n
            marker_array.markers.append(marker)

        # Publish the marker
        self.marker_publisher.publish(marker_array)