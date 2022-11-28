#!/usr/bin/env python3
from visualization_msgs.msg import Marker
import rospy

class MeshMarker(object):
    """
    Class to visualize meshes as markers in RViz

    """
    id = 0

    def __init__(self, packageName, meshName, scale=1):
        """
        Mesh file must be names [meshName] placed in [packageName]/meshes folder.

        """

        self.pkg = packageName
        self.mname = meshName
        self.mesh = f"package://{self.pkg}/meshes/{self.mname}"

        reference_frame = rospy.get_param('reference_frame','base_link')
        self.marker_pub = rospy.Publisher("visualization_marker", Marker, queue_size=10)

        self.marker = Marker()
        self.marker.header.frame_id = reference_frame
        self.marker.ns = "mesh_markers"
        self.marker.id = MeshMarker.id
        MeshMarker.id += 1

        self.marker.mesh_resource = self.mesh
        self.marker.mesh_use_embedded_materials = True  # Need this to use textures for mesh
        self.marker.type = self.marker.MESH_RESOURCE
        self.marker.header.frame_id = reference_frame
        self.marker.action = self.marker.ADD
        #Pose
        self.marker.pose.position.x = 0.0
        self.marker.pose.position.y = 0.0
        self.marker.pose.position.z = 0.0
        self.marker.pose.orientation.x = 0.0
        self.marker.pose.orientation.y = 0.0
        self.marker.pose.orientation.z = 0.0
        self.marker.pose.orientation.w = 1.0
        #Scale
        self.marker.scale.x = scale
        self.marker.scale.y = scale
        self.marker.scale.z = scale
        self.marker.lifetime = rospy.Duration()

    def position(self, position):
        """
        Set position (list) for the mesh and publish it

        """
        self.marker.pose.position.x = position[0]
        self.marker.pose.position.y = position[1]
        self.marker.pose.position.z = position[2]
        self.publish()


    def pose(self, pose):
        """
        Set pose (list) for the mesh and publish it.\n
        list = [x, y, z, ew, ex, ey, ez]

        """
        self.marker.pose.position.x = pose[0]
        self.marker.pose.position.y = pose[1]
        self.marker.pose.position.z = pose[2]
        self.marker.pose.orientation.w = pose[3]
        self.marker.pose.orientation.x = pose[4]
        self.marker.pose.orientation.y = pose[5]
        self.marker.pose.orientation.z = pose[6]
        self.publish()

    def setMesh(self, mesh):
        self.marker.mesh_resource = f"package://{self.pkg}/meshes/{mesh}"
        self.publish()

    def publish(self):
        self.marker_pub.publish(self.marker)