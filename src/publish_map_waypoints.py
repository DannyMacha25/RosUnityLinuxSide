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
#Boston Dynamics Function
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


def api_to_vtk_se3_pose(se3_pose):
    """
    Convert a bosdyn SDK SE3Pose into a VTK pose.
    :param se3_pose: the bosdyn SDK SE3 Pose.
    :return: A VTK pose representing the bosdyn SDK SE3 Pose.
    """
    return mat_to_vtk(se3_pose.to_matrix())

def load_map(path):
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
                current_anchors, current_anchored_world_objects)

#//Boston Dynamics Functions
def createFudicialStampedTransform(wo,wo_tform,quat):
    stampedTMsg = TransformStamped()

    stampedTMsg.child_frame_id = "/" + wo.name 
    stampedTMsg.header.frame_id = '/base_link'
    stampedTMsg.header.stamp.secs = wo.acquisition_time.seconds
    stampedTMsg.header.stamp.nsecs = wo.acquisition_time.nanos

    pos = wo_tform.GetPosition()


    stampedTMsg.transform.translation.x = pos[0] * 10
    stampedTMsg.transform.translation.y = pos[1] * 10
    stampedTMsg.transform.translation.z = pos[2] * 10

    stampedTMsg.transform.rotation.x = quat.x 
    stampedTMsg.transform.rotation.y = quat.y  
    stampedTMsg.transform.rotation.z = quat.z  
    stampedTMsg.transform.rotation.w = quat.w 
    return stampedTMsg


def createWaypointStampedTransformQuatPos(wp,pos,quat):
    stampedTMsg = TransformStamped()

    stampedTMsg.child_frame_id = "/" + wp.annotations.name
    stampedTMsg.header.frame_id = '/base_link'
    stampedTMsg.header.stamp.secs = rospy.get_rostime().secs
    stampedTMsg.header.stamp.nsecs = rospy.get_rostime().nsecs





    stampedTMsg.transform.translation.x = pos.x * 10
    stampedTMsg.transform.translation.y = pos.y * 10
    stampedTMsg.transform.translation.z = pos.z * 10

    stampedTMsg.transform.rotation.x = quat.x 
    stampedTMsg.transform.rotation.y = quat.y  
    stampedTMsg.transform.rotation.z = quat.z  
    stampedTMsg.transform.rotation.w = quat.w 

    return stampedTMsg

def create_fiducial_tform(world_object,waypoint):
    fiducial_object = world_object.apriltag_properties
    odom_tform_fiducial_filtered = get_a_tform_b(
        world_object.transforms_snapshot, ODOM_FRAME_NAME,
        world_object.apriltag_properties.frame_name_fiducial_filtered)
    waypoint_tform_odom = SE3Pose.from_obj(waypoint.waypoint_tform_ko)
    waypoint_tform_fiducial_filtered = api_to_vtk_se3_pose(
        waypoint_tform_odom * odom_tform_fiducial_filtered)
    
    return waypoint_tform_fiducial_filtered

def process_graph(current_graph,current_waypoints,current_anchors,current_anchored_world_objects,current_waypoint_snapshots):

    tfmsgs = []

    queue = []
    queue.append((current_graph.waypoints[0], np.eye(4)))
    visited = {}
   
    # Breadth first search.
    while len(queue) > 0:
        # Visit a waypoint.
        curr_element = queue[0]
        queue.pop(0)
        curr_waypoint = curr_element[0]
        if curr_waypoint.id in visited:
            continue
        visited[curr_waypoint.id] = True
        

        world_tform_current_waypoint = curr_element[1]
        
        anchor = current_anchors[curr_waypoint.id]
        tfmsgs.append(createWaypointStampedTransformQuatPos(curr_waypoint,anchor.seed_tform_waypoint.position,anchor.seed_tform_waypoint.rotation))

        
        # For each fiducial in the waypoint's snapshot, make a msg with its position and rotation
        if (curr_waypoint.snapshot_id in current_waypoint_snapshots):
            snapshot = current_waypoint_snapshots[curr_waypoint.snapshot_id]
            for fiducial in snapshot.objects:
                if fiducial.HasField("apriltag_properties"):
                    
                    curr_wp_tform_fiducial = create_fiducial_tform(fiducial,curr_waypoint) 
                    world_tform_fiducial = np.dot(world_tform_current_waypoint,
                                                  vtk_to_mat(curr_wp_tform_fiducial))    

                    for anchor_key in current_anchored_world_objects:
                        for anchor in current_anchored_world_objects[anchor_key]:
                        
                            if isinstance(anchor.id,str): #this should narrow it down to just fd
                                try:
                                    tform = anchor.seed_tform_object
                                    tfmsgs.append(createFudicialStampedTransform(fiducial,mat_to_vtk(world_tform_fiducial),tform.rotation))
                                except AttributeError:
                                    pass
    
        for edge in current_graph.edges:
            # If the edge is directed away from us...
            if edge.id.from_waypoint == curr_waypoint.id and edge.id.to_waypoint not in visited:
                current_waypoint_tform_to_waypoint = SE3Pose.from_obj(
                    edge.from_tform_to).to_matrix()
                world_tform_to_wp = np.dot(world_tform_current_waypoint, current_waypoint_tform_to_waypoint)
                # Add the neighbor to the queue.
                queue.append((current_waypoints[edge.id.to_waypoint], world_tform_to_wp))
            # If the edge is directed toward us...
            elif edge.id.to_waypoint == curr_waypoint.id and edge.id.from_waypoint not in visited:
                current_waypoint_tform_from_waypoint = (SE3Pose.from_obj(
                    edge.from_tform_to).inverse()).to_matrix()
                world_tform_from_wp = np.dot(world_tform_current_waypoint, current_waypoint_tform_to_waypoint)
                # Add the neighbor to the queue.
                queue.append((current_waypoints[edge.id.from_waypoint], world_tform_from_wp))

    return tfmsgs

def main():
    rospy.init_node("GraphNav Publisher")

    topic = '/dumb' #This topic takes our msgs, and a different script will convert to TFmessages and
                    # publish to /tf
    path = "/mnt/hgfs/Shared Folder/downloaded_graph"

    #may make topic and path ros params

    pub = rospy.Publisher(topic,TransformStamped,queue_size=10)

    (current_graph, current_waypoints, current_waypoint_snapshots,
     current_anchors, current_anchored_world_objects) = load_map(path)

    #Get msgs 
    msgs = process_graph(current_graph,current_waypoints,current_anchors,current_anchored_world_objects,current_waypoint_snapshots)

    loop = rospy.Rate(1)

    #publish
    while not rospy.is_shutdown():

        for msg in msgs:
            pub.publish(msg)
        
        loop.sleep()






if __name__ == "__main__":
    main()