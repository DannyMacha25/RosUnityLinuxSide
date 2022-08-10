
# Dan's Linux Side of all of the Spot Vr Stuff he did

## Summary
Hello, this is a collection of all the nodes I created
and used during my time with the HRI Lab. This collection
includes some useful nodes that can be discected, or just
complete garbage. I'll explain each node so that you don't 
have to figure all of that out yourself ;)

## color_cube_topic.py
This was the first project I worked on just to get my bearings
with ROS in Unity. All this does is publish and RGBA to a topic
so that unity could listen in and change the color of a cube 
accordingly.

## tutorial_service.py
Another quick demo for ROS Unity. This was to test services
on Unity.

## waypoint_pub.py & view_map.py
Dont worry about it, unfinished version of publish_map_waypoints.py

## tf_tester.py
Makes and publishes frames to simulate a tf tree from a robot that
could exist. Used to test Transform Cache in Unity. Contains a function
that allows for quick and easy creation of frames!

## publish_map_waypoints.py
This was my attempt at extracting data from Spot's Graphnav file and
importing it into Unity. This was made by taking parts of Boston Dynamic'
file 'view_map.py' (which I have a similar copy of in here) and adding
publishers and messages. To use it yourself just change the path in the file
use the launch file!

## convert_stamped_tf.py
Reads from a topic that has StampedTransforms, then adds them to a TFMessage
and publishes the TFMessage. I made this so it would work in tandem with 
publish_map_waypoints

## unity_transform_reciever.py
Recieves a tranfrom from Unity. This was made to take in transforms
that would be sent from Unity when the user would right-click and
drag on the map. Once the user let go it would create a Transform 
and publish it to a topic.
