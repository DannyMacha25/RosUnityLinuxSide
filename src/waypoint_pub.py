#!/usr/bin/env python3.7


import rospy
from geometry_msgs.msg import TransformStamped
import argparse
from vtk.util import numpy_support
import google.protobuf.timestamp_pb2
import math
import numpy as np
import numpy.linalg
import os
import sys
import time
import vtk



from bosdyn.api.graph_nav import map_pb2
from bosdyn.api import geometry_pb2
from bosdyn.client.frame_helpers import *
from bosdyn.client.math_helpers import *




topic = '/dumb'
topic_static = "/tf_static"
path = "/mnt/hgfs/Shared Folder/downloaded_graph"

def load_map(path):
    """
    Load a map from the given file path.
    :param path: Path to the root directory of the map.
    :return: the graph, waypoints, waypoint snapshots and edge snapshots.
    """
    with open(os.path.join(path, "graph"), "rb") as graph_file:
        # Load the graph file and deserialize it. The graph file is a protobuf containing only the waypoints and the
        # edges between them.
        data = graph_file.read()
        current_graph = map_pb2.Graph()
        current_graph.ParseFromString(data)

        # Set up maps from waypoint ID to waypoints, edges, snapshots, etc.
        current_waypoints = {}
        current_waypoint_snapshots = {}
        current_edge_snapshots = {}
        current_anchors = {}
        current_anchored_world_objects = {}

        # Load the anchored world objects first so we can look in each waypoint snapshot as we load it.
        for anchored_world_object in current_graph.anchoring.objects:
            current_anchored_world_objects[anchored_world_object.id] = (anchored_world_object,)
        # For each waypoint, load any snapshot associated with it.
        for waypoint in current_graph.waypoints:
            current_waypoints[waypoint.id] = waypoint

            if len(waypoint.snapshot_id) == 0:
                continue
            # Load the snapshot. Note that snapshots contain all of the raw data in a waypoint and may be large.
            file_name = os.path.join(path, "waypoint_snapshots", waypoint.snapshot_id)
            if not os.path.exists(file_name):
                continue
            with open(file_name, "rb") as snapshot_file:
                waypoint_snapshot = map_pb2.WaypointSnapshot()
                waypoint_snapshot.ParseFromString(snapshot_file.read())
                current_waypoint_snapshots[waypoint_snapshot.id] = waypoint_snapshot

                for fiducial in waypoint_snapshot.objects:
                    if not fiducial.HasField("apriltag_properties"):
                        continue

                    str_id = str(fiducial.apriltag_properties.tag_id)
                    if (str_id in current_anchored_world_objects and
                            len(current_anchored_world_objects[str_id]) == 1):

                        # Replace the placeholder tuple with a tuple of (wo, waypoint, fiducial).
                        anchored_wo = current_anchored_world_objects[str_id][0]
                        current_anchored_world_objects[str_id] = (anchored_wo, waypoint, fiducial)

        # Similarly, edges have snapshot data.
        for edge in current_graph.edges:
            if len(edge.snapshot_id) == 0:
                continue
            file_name = os.path.join(path, "edge_snapshots", edge.snapshot_id)
            if not os.path.exists(file_name):
                continue
            with open(file_name, "rb") as snapshot_file:
                edge_snapshot = map_pb2.EdgeSnapshot()
                edge_snapshot.ParseFromString(snapshot_file.read())
                current_edge_snapshots[edge_snapshot.id] = edge_snapshot
        for anchor in current_graph.anchoring.anchors:
            current_anchors[anchor.id] = anchor
        print("Loaded graph with {} waypoints, {} edges, {} anchors, and {} anchored world objects".
              format(len(current_graph.waypoints), len(current_graph.edges),
                     len(current_graph.anchoring.anchors), len(current_graph.anchoring.objects)))
        return (current_graph, current_waypoints, current_waypoint_snapshots,
                current_edge_snapshots, current_anchors, current_anchored_world_objects)


def get_quaternion_from_euler(roll, pitch, yaw):
  """
  Convert an Euler angle to a quaternion.
   
  Input
    :param roll: The roll (rotation around x-axis) angle in radians.
    :param pitch: The pitch (rotation around y-axis) angle in radians.
    :param yaw: The yaw (rotation around z-axis) angle in radians.
 
  Output
    :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
  """
  qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
  qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
  qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
  qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
 
  return [qx, qy, qz, qw]


def createWaypointStampedTransform(wp,wp_tform):
    stampedTMsg = TransformStamped()

    stampedTMsg.child_frame_id = "/" + wp.annotations.name
    stampedTMsg.header.frame_id = '/base_link'
    stampedTMsg.header.stamp.secs = rospy.get_rostime().secs
    stampedTMsg.header.stamp.nsecs = rospy.get_rostime().nsecs

    pos = wp_tform.GetPosition()
    eulerRot = wp_tform.GetOrientation()

    rot = get_quaternion_from_euler(eulerRot[0],eulerRot[1],eulerRot[2])

    stampedTMsg.transform.translation.x = pos[0] * 10
    stampedTMsg.transform.translation.y = pos[1] * 10
    stampedTMsg.transform.translation.z = pos[2] * 10

    stampedTMsg.transform.rotation.x = rot[0]
    stampedTMsg.transform.rotation.y = rot[1]
    stampedTMsg.transform.rotation.z = rot[2]
    stampedTMsg.transform.rotation.w = rot[3]

    return stampedTMsg

def createFudicialStampedTransform(wo,wo_tform):
    stampedTMsg = TransformStamped()

    stampedTMsg.child_frame_id = "/" + wo.name
    stampedTMsg.header.frame_id = '/base_link'
    stampedTMsg.header.stamp.secs = wo.acquisition_time.seconds
    stampedTMsg.header.stamp.nsecs = wo.acquisition_time.nanos

    pos = wo_tform.GetPosition()
    eulerRot = wo_tform.GetOrientation()

    rot = get_quaternion_from_euler(eulerRot[0],eulerRot[1],eulerRot[2])

    stampedTMsg.transform.translation.x = pos[0] * 10
    stampedTMsg.transform.translation.y = pos[1] * 10
    stampedTMsg.transform.translation.z = pos[2] * 1

    stampedTMsg.transform.rotation.x = rot[0]
    stampedTMsg.transform.rotation.y = rot[1]
    stampedTMsg.transform.rotation.z = rot[2]
    stampedTMsg.transform.rotation.w = rot[3]

    return stampedTMsg


def transformFromWorldObject(wo):
    tf = TransformStamped()

    tf.child_frame_id = "/" + wo.name
    tf.header.frame_id = '/base_link'
    tf.header.stamp.secs = wo.acquisition_time.seconds
    tf.header.stamp.nsecs = wo.acquisition_time.nanos

    fiducial_num = wo.name[-3:]
    print(fiducial_num)

    data = wo.transforms_snapshot.child_to_parent_edge_map['fiducial_' + fiducial_num].parent_tform_child

    print(data)

    tf.transform.translation.x =  data.position.x
    tf.transform.translation.y =  data.position.y
    tf.transform.translation.z =  data.position.z

    tf.transform.rotation.x = data.rotation.x
    tf.transform.rotation.y = data.rotation.y
    tf.transform.rotation.z = data.rotation.z
    tf.transform.rotation.w = data.rotation.w

    return tf

def api_to_vtk_se3_pose(se3_pose):
    """
    Convert a bosdyn SDK SE3Pose into a VTK pose.
    :param se3_pose: the bosdyn SDK SE3 Pose.
    :return: A VTK pose representing the bosdyn SDK SE3 Pose.
    """
    return mat_to_vtk(se3_pose.to_matrix())

def mat_to_vtk(mat):
    """
    Converts a 4x4 homogenous transform into a vtk transform object.
    :param mat: A 4x4 homogenous transform (numpy array).
    :return: A VTK transform object representing the transform.
    """
    t = vtk.vtkTransform()
    t.SetMatrix(mat.flatten())
    return t

def vtk_to_mat(transform):
    """
    Converts a VTK transform object to 4x4 homogenous numpy matrix.
    :param transform: an object of type vtkTransform
    : return: a numpy array with a 4x4 matrix representation of the transform.
    """
    tf_matrix = transform.GetMatrix()
    out = np.array(np.eye(4))
    for r in range(4):
        for c in range(4):
            out[r, c] = tf_matrix.GetElement(r, c)
    return out

def main():
    # Load the map from the given file.
    rospy.init_node("e")
    pub = rospy.Publisher(topic,TransformStamped,queue_size=10)

    (current_graph, current_waypoints, current_waypoint_snapshots, current_edge_snapshots,
     current_anchors, current_anchored_world_objects) = load_map(path)

    loop = rospy.Rate(10)

    while not rospy.is_shutdown():
        waypoint_objects = {}
        for waypoint in current_waypoints:
            print(current_waypoints[waypoint].annotations.name)

            wp = current_waypoints[waypoint]
            snapshot = current_waypoint_snapshots[wp.snapshot_id]
            cloud = snapshot.point_cloud
            seed_tform_waypoint = SE3Pose.from_obj(current_anchors[wp.id].seed_tform_waypoint)


            pcl_tform_odom = get_a_tform_b(cloud.source.transforms_snapshot, ODOM_FRAME_NAME,cloud.source.frame_name_sensor)
            wp_tform_odom = SE3Pose.from_obj(wp.waypoint_tform_ko)
            

            pcl_tform_cloud = api_to_vtk_se3_pose(wp_tform_odom * pcl_tform_odom)
            print(pcl_tform_cloud.GetPosition())
            pcl_tform_cloud.Translate(api_to_vtk_se3_pose(seed_tform_waypoint).GetPosition())
            print(pcl_tform_cloud.GetPosition())
            stampedTMsg = createWaypointStampedTransform(wp,pcl_tform_cloud)
            

            waypoint_objects[wp.id] = pcl_tform_cloud
            pub.publish(stampedTMsg)
           
        
        for fiducial in current_anchored_world_objects:
            wo = current_anchored_world_objects[fiducial]
            for e in wo:
                
                if hasattr(e,"apriltag_properties"):
                    
                    fiducial_num = e.name[-3:]
                    data = e.transforms_snapshot.child_to_parent_edge_map['fiducial_' + fiducial_num].parent_tform_child

                    waypoint_tform_odom = SE3Pose(data.position.x,data.position.y,data.position.z,data.rotation)

                    odom_tform_fiducial_filtered = get_a_tform_b(e.transforms_snapshot,ODOM_FRAME_NAME,e.apriltag_properties.frame_name_fiducial_filtered)
                    waypoint_tform_fiducial_filtered = api_to_vtk_se3_pose(waypoint_tform_odom*odom_tform_fiducial_filtered)
                    #print('e')
                    #print(waypoint_tform_fiducial_filtered.GetPosition())
                    #tfmsg = createFudicialStampedTransform(e,waypoint_tform_fiducial_filtered)

                    #pub.publish(tfmsg)


        #test section

        queue = []
        queue.append((current_graph.waypoints[0],np.eye(4)))
        visited = {}

        while len(queue) > 0:
            
            curr_element = queue[0]
            queue.pop(0)
            curr_waypoint = curr_element[0]
            if curr_waypoint.id in visited:
                continue
            visited[curr_waypoint.id] = True

            world_tform_current_waypoint = curr_element[1]

            #waypoint_objects[curr_waypoint.id].Translate(mat_to_vtk(curr_element[1]).GetPosition())
           # print("d")
           # print(waypoint_objects[curr_waypoint.id].GetPosition())
           # print("d")

            stampedTMsg = createWaypointStampedTransform(curr_waypoint,waypoint_objects[curr_waypoint.id])
            #pub.publish(stampedTMsg)

            if(curr_waypoint.snapshot_id in current_waypoint_snapshots):
                snapshot = current_waypoint_snapshots[curr_waypoint.snapshot_id]
                for fiducial in snapshot.objects:
                    if fiducial.HasField("apriltag_properties"):
                        #fiducial,curr_waypoint
                        print(fiducial.name)
                        print(curr_waypoint.annotations.name)
                        waypoint_tform_odom = SE3Pose.from_obj(curr_waypoint.waypoint_tform_ko)
                        odom_tform_fiducial_filtered = get_a_tform_b(fiducial.transforms_snapshot,ODOM_FRAME_NAME,e.apriltag_properties.frame_name_fiducial_filtered)
                        waypoint_tform_fiducial_filtered = api_to_vtk_se3_pose(waypoint_tform_odom*odom_tform_fiducial_filtered)
                        tfstamp = createFudicialStampedTransform(fiducial,waypoint_tform_fiducial_filtered)
                        print(waypoint_tform_odom)
                        print(odom_tform_fiducial_filtered)
                        print(waypoint_tform_fiducial_filtered.GetPosition())
                        the_real_tform_np = np.dot(world_tform_current_waypoint,vtk_to_mat(waypoint_tform_fiducial_filtered))
                        the_real_tform_vtk = mat_to_vtk(the_real_tform_np)
                        print(the_real_tform_vtk.GetPosition())
                        pub.publish(tfstamp)
                
                    


        
                    
        
        print("published!")
        loop.sleep()
        #rospy.spin()    

def main6():
    # Load the map from the given file.
    rospy.init_node("e")
    pub = rospy.Publisher(topic,TransformStamped,queue_size=10)

    (current_graph, current_waypoints, current_waypoint_snapshots, current_edge_snapshots,
     current_anchors, current_anchored_world_objects) = load_map(path)

    rospy.init_node("e")
    pub = rospy.Publisher(topic,TransformStamped,queue_size=10)

    loop = rospy.Rate(10)

    renderer = vtk.vtkRenderer()
    while not rospy.is_shutdown():
        create_graph_objects(current_graph,current_waypoint_snapshots,current_waypoints,renderer,pub)




def getSE3PoseFromFiducial(fiducial):
    print("e")




##Stuff
def create_graph_objects(current_graph, current_waypoint_snapshots, current_waypoints, renderer,pub):
    """
    Creates all the VTK objects associated with the graph.
    :param current_graph: the graph to use.
    :param current_waypoint_snapshots: dict from snapshot id to snapshot.
    :param current_waypoints: dict from waypoint id to waypoint.
    :param renderer: The VTK renderer
    :return: the average position in world space of all the waypoints.
    """
    waypoint_objects = {}
    # Create VTK objects associated with each waypoint.
    for waypoint in current_graph.waypoints:
        waypoint_objects[waypoint.id] = create_waypoint_object(renderer, current_waypoints,
                                                               current_waypoint_snapshots,
                                                               waypoint.id)
    # Now, perform a breadth first search of the graph starting from an arbitrary waypoint. Graph nav graphs
    # have no global reference frame. The only thing we can say about waypoints is that they have relative
    # transformations to their neighbors via edges. So the goal is to get the whole graph into a global reference
    # frame centered on some waypoint as the origin.
    queue = []
    queue.append((current_graph.waypoints[0], np.eye(4)))
    visited = {}
    # Get the camera in the ballpark of the right position by centering it on the average position of a waypoint.
    avg_pos = np.array([0.0, 0.0, 0.0])

    # Breadth first search.
    while len(queue) > 0:
        # Visit a waypoint.
        curr_element = queue[0]
        queue.pop(0)
        curr_waypoint = curr_element[0]
        if curr_waypoint.id in visited:
            continue
        visited[curr_waypoint.id] = True

        # We now know the global pose of this waypoint, so set the pose.
        waypoint_objects[curr_waypoint.id].SetUserTransform(mat_to_vtk(curr_element[1]))
        world_tform_current_waypoint = curr_element[1]

        #my sstuff
        msg = createWaypointStampedTransform(curr_waypoint,waypoint_objects[curr_waypoint.id].GetUserTransform())
        pub.publish(msg)

        #end my stuff

        # For each fiducial in the waypoint's snapshot, add an object at the world pose of that fiducial.
        if (curr_waypoint.snapshot_id in current_waypoint_snapshots):
            snapshot = current_waypoint_snapshots[curr_waypoint.snapshot_id]
            for fiducial in snapshot.objects:
                if fiducial.HasField("apriltag_properties"):
                    (fiducial_object, curr_wp_tform_fiducial) = create_fiducial_object(
                        fiducial, curr_waypoint, renderer)
                    world_tform_fiducial = np.dot(world_tform_current_waypoint,
                                                  vtk_to_mat(curr_wp_tform_fiducial))
                    fiducial_object.SetUserTransform(mat_to_vtk(world_tform_fiducial))
                   


    # Compute the average waypoint position to place the camera appropriately.
    avg_pos /= len(current_waypoints)
    return avg_pos

def create_fiducial_object(world_object, waypoint, renderer):
    """
    Creates a VTK object representing a fiducial.
    :param world_object: A WorldObject representing a fiducial.
    :param waypoint: The waypoint the AprilTag is associated with.
    :param renderer: The VTK renderer
    :return: a tuple of (vtkActor, 4x4 homogenous transform) representing the vtk actor for the fiducial, and its
    transform w.r.t the waypoint.
    """
    fiducial_object = world_object.apriltag_properties
    odom_tform_fiducial_filtered = get_a_tform_b(
        world_object.transforms_snapshot, ODOM_FRAME_NAME,
        world_object.apriltag_properties.frame_name_fiducial_filtered)
    waypoint_tform_odom = SE3Pose.from_obj(waypoint.waypoint_tform_ko)
    waypoint_tform_fiducial_filtered = api_to_vtk_se3_pose(
        waypoint_tform_odom * odom_tform_fiducial_filtered)
    plane_source = vtk.vtkPlaneSource()
    plane_source.SetCenter(0.0, 0.0, 0.0)
    plane_source.SetNormal(0.0, 0.0, 1.0)
    plane_source.Update()
    plane = plane_source.GetOutput()
    mapper = vtk.vtkPolyDataMapper()
    mapper.SetInputData(plane)

    actor = vtk.vtkActor()
    actor.SetMapper(mapper)
    actor.GetProperty().SetColor(0.5, 0.7, 0.9)
    actor.SetScale(fiducial_object.dimensions.x, fiducial_object.dimensions.y, 1.0)
    renderer.AddActor(actor)
    return actor, waypoint_tform_fiducial_filtered

def create_point_cloud_object(waypoints, snapshots, waypoint_id):
    """
    Create a VTK object representing the point cloud in a snapshot. Note that in graph_nav, "point cloud" refers to the
    feature cloud of a waypoint -- that is, a collection of visual features observed by all five cameras at a particular
    point in time. The visual features are associated with points that are rigidly attached to a waypoint.
    :param waypoints: dict of waypoint ID to waypoint.
    :param snapshots: dict of waypoint snapshot ID to waypoint snapshot.
    :param waypoint_id: the waypoint ID of the waypoint whose point cloud we want to render.
    :return: a vtkActor containing the point cloud data.
    """
    wp = waypoints[waypoint_id]
    snapshot = snapshots[wp.snapshot_id]
    cloud = snapshot.point_cloud
    odom_tform_cloud = get_a_tform_b(cloud.source.transforms_snapshot, ODOM_FRAME_NAME,
                                     cloud.source.frame_name_sensor)
    waypoint_tform_odom = SE3Pose.from_obj(wp.waypoint_tform_ko)
    waypoint_tform_cloud = api_to_vtk_se3_pose(waypoint_tform_odom * odom_tform_cloud)

    point_cloud_data = np.frombuffer(cloud.data, dtype=np.float32).reshape(int(cloud.num_points), 3)
    poly_data = numpy_to_poly_data(point_cloud_data)
    arr = vtk.vtkFloatArray()
    for i in range(cloud.num_points):
        arr.InsertNextValue(point_cloud_data[i, 2])
    arr.SetName("z_coord")
    poly_data.GetPointData().AddArray(arr)
    poly_data.GetPointData().SetActiveScalars("z_coord")
    actor = vtk.vtkActor()
    mapper = vtk.vtkPolyDataMapper()
    mapper.SetInputData(poly_data)
    mapper.ScalarVisibilityOn()
    actor.SetMapper(mapper)
    actor.GetProperty().SetPointSize(2)
    actor.SetUserTransform(waypoint_tform_cloud)
    return actor


def create_waypoint_object(renderer, waypoints, snapshots, waypoint_id):
    """
    Creates a VTK object representing a waypoint and its point cloud.
    :param renderer: The VTK renderer.
    :param waypoints: dict of waypoint ID to waypoint.
    :param snapshots: dict of snapshot ID to snapshot.
    :param waypoint_id: the waypoint id of the waypoint object we wish to create.
    :return: A vtkAssembly representing the waypoint (an axis) and its point cloud.
    """
    assembly = vtk.vtkAssembly()
    actor = vtk.vtkAxesActor()
    actor.SetXAxisLabelText("")
    actor.SetYAxisLabelText("")
    actor.SetZAxisLabelText("")
    actor.SetTotalLength(0.2, 0.2, 0.2)
    point_cloud_actor = create_point_cloud_object(waypoints, snapshots, waypoint_id)
    assembly.AddPart(actor)
    assembly.AddPart(point_cloud_actor)
    renderer.AddActor(assembly)
    return assembly

def numpy_to_poly_data(pts):
    """
    Converts numpy array data into vtk poly data.
    :param pts: the numpy array to convert (3 x N).
    :return: a vtkPolyData.
    """
    pd = vtk.vtkPolyData()
    pd.SetPoints(vtk.vtkPoints())
    # Makes a deep copy
    pd.GetPoints().SetData(numpy_support.numpy_to_vtk(pts.copy()))

    f = vtk.vtkVertexGlyphFilter()
    f.SetInputData(pd)
    f.Update()
    pd = f.GetOutput()

    return pd

if __name__ == "__main__":
    main()