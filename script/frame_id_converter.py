import rosbag
import argparse
import rospy
import tf
import sys

parser = argparse.ArgumentParser(description='frame id converter for rosbag')
parser.add_argument('--input', help='input bag')    
args = parser.parse_args() 

listener = tf.TransformListener()
try:
    (trans,rot) = listener.lookupTransform('/odom', '/base_link', rospy.Time(0))
except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
    "Can't get tf from odom to base_link"
    sys.exit()

with rosbag.Bag(args.input.split('.')[1] + 'converted.bag', 'w') as outbag:
     for topic, msg, t in rosbag.Bag(args.input).read_messages():
         # This also replaces tf timestamps under the assumption 
         # that all transforms in the message share the same timestamp
         if topic == "/ypspur/twist":
             msg.frame_id = ""
             outbag.write(topic, msg, msg.transforms[0].header.stamp)
         else:
             outbag.write(topic, msg, msg.header.stamp if msg._has_header else t)