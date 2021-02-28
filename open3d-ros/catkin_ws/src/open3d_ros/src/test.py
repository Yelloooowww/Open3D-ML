import sensor_msgs.point_cloud2 as pc2
import open3d

class CameraListner():
    def __init__(self):
        self.pc = None
        self.n = 0
        self.listener()

    ############################################################################
    def callback(self, points):
        self.pc = points
        self.n = self.n + 1

    def listener(self):
        rospy.init_node('ui_config_node', anonymous=True)
        rospy.Subscriber('/camera/depth/color/points', PointCloud2, self.callback)


class ViewerWidget(QtWidgets.QWidget):
    def __init__(self, subscriber, parent=None):
        self.subscriber = subscriber
        rospy.loginfo('initialization')

        self.vis = open3d.visualization.Visualizer()
        self.point_cloud = None
        self.updater()

    ############################################################################
    def updater(self):

        rospy.loginfo('start')
        self.first = False
        while (self.subscriber.pc is None):
            time.sleep(2)
        self.point_cloud = open3d.geometry.PointCloud()
        self.point_cloud.points = open3d.utility.Vector3dVector(ros_numpy.point_cloud2.pointcloud2_to_xyz_array(self.subscriber.pc))
        self.vis.create_window()
        print('get points')
        self.vis.add_geometry(self.point_cloud)
        print ('add points')
        self.vis.update_geometry()
        self.vis.poll_events()

        self.vis.update_renderer()

        while not rospy.is_shutdown():
            self.point_cloud.points =  open3d.utility.Vector3dVector(ros_numpy.point_cloud2.pointcloud2_to_xyz_array(self.subscriber.pc))
            self.vis.update_geometry()
            self.vis.poll_events()
            self.vis.update_renderer()

if __name__ == '__main__':

    listener = CameraListner()
    updater = Viewer(listener)
    rospy.spin()
