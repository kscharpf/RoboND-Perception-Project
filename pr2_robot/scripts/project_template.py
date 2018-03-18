#!/usr/bin/env python

# Import modules
import numpy as np
import sklearn
from sklearn.preprocessing import LabelEncoder
import pickle
from sensor_stick.srv import GetNormals
from sensor_stick.features import compute_color_histograms
from sensor_stick.features import compute_normal_histograms
from visualization_msgs.msg import Marker
from sensor_stick.marker_tools import *
from sensor_stick.msg import DetectedObjectsArray
from sensor_stick.msg import DetectedObject
from sensor_stick.pcl_helper import *

import rospy
import tf
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64
from std_msgs.msg import Int32
from std_msgs.msg import String
from pr2_robot.srv import *
from rospy_message_converter import message_converter
import yaml
import os.path


# Helper function to get surface normals
def get_normals(cloud):
    get_normals_prox = rospy.ServiceProxy('/feature_extractor/get_normals', GetNormals)
    return get_normals_prox(cloud).cluster

# Helper function to create a yaml friendly dictionary from ROS messages
def make_yaml_dict(test_scene_num, arm_name, object_name, pick_pose, place_pose):
    yaml_dict = {}
    yaml_dict["test_scene_num"] = test_scene_num.data
    yaml_dict["arm_name"]  = arm_name.data
    yaml_dict["object_name"] = object_name.data
    yaml_dict["pick_pose"] = message_converter.convert_ros_message_to_dictionary(pick_pose)
    yaml_dict["place_pose"] = message_converter.convert_ros_message_to_dictionary(place_pose)
    return yaml_dict

# Helper function to output to yaml file
def send_to_yaml(yaml_filename, dict_list):
    print "Opening file: %s" % yaml_filename
    data_dict = {"object_list": dict_list}
    with open(yaml_filename, 'w') as outfile:
        yaml.dump(data_dict, outfile, default_flow_style=False)

# Callback function for your Point Cloud Subscriber
def pcl_callback(pcl_msg):
    first_exec = not os.path.isfile("downsampled.pcd")
    # TODO: Convert ROS msg to PCL data
    cloud = ros_to_pcl(pcl_msg)

    # Much like the previous filters, we start by creating a filter object: 
    outlier_filter = cloud.make_statistical_outlier_filter()

    # Set the number of neighboring points to analyze for any given point
    outlier_filter.set_mean_k(100)

    # Set threshold scale factor - orig 1.0
    x = 0.5

    # Any point with a mean distance larger than global (mean distance+x*std_dev) will be considered outlier
    outlier_filter.set_std_dev_mul_thresh(x)

    # Finally call the filter function for magic
    cloud = outlier_filter.filter()

    # TODO: Voxel Grid Downsampling
    LEAF_SIZE = 0.01
    #vox = cloud_filtered.make_voxel_grid_filter()
    vox = cloud.make_voxel_grid_filter()
    vox.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE) 
 
    # Call the filter function to obtain the resultant downsampled point cloud
    cloud_filtered = vox.filter()

    if first_exec:
        pcl.save(cloud_filtered, "downsampled.pcd")

    # TODO: PassThrough Filter
    passthrough = cloud_filtered.make_passthrough_filter()
    filter_axis = 'z'
    passthrough.set_filter_field_name(filter_axis)
    axis_min = 0.6
    axis_max = 0.9
    passthrough.set_filter_limits(axis_min,axis_max)
  
    cloud_filtered = passthrough.filter()
    passthrough = cloud_filtered.make_passthrough_filter()
    filter_axis = 'y'
    passthrough.set_filter_field_name(filter_axis)
    axis_min = -0.5
    axis_max =  0.5
    passthrough.set_filter_limits(axis_min,axis_max)
  
    cloud_filtered = passthrough.filter()
    if first_exec:
        pcl.save(cloud_filtered, "passthrough.pcd")

    passthrough_cloud = cloud_filtered

    # TODO: RANSAC Plane Segmentation

    seg = cloud_filtered.make_segmenter()
    seg.set_model_type(pcl.SACMODEL_PLANE)
    seg.set_method_type(pcl.SAC_RANSAC)
 
    max_distance = 0.01
    seg.set_distance_threshold(max_distance)
  
    # TODO: Extract inliers and outliers
    inliers, coefficients = seg.segment()
    cloud_table = cloud_filtered.extract(inliers, negative=False)
    if first_exec:
        pcl.save(cloud_table, "inliers.pcd")
    cloud_objects = cloud_filtered.extract(inliers, negative=True)
    if first_exec:
        pcl.save(cloud_objects, "outliers.pcd")

    # TODO: Euclidean Clustering
    white_cloud = XYZRGB_to_XYZ(cloud_objects)
    tree = white_cloud.make_kdtree()
    ec = white_cloud.make_EuclideanClusterExtraction()
    ec.set_ClusterTolerance(0.015)
    ec.set_MinClusterSize(50)
    ec.set_MaxClusterSize(2500)
    ec.set_SearchMethod(tree)

    cluster_indices = ec.Extract()

    # TODO: Create Cluster-Mask Point Cloud to visualize each cluster separately
    cluster_color = get_color_list(len(cluster_indices))
    color_cluster_point_list = []

    for j, indices in enumerate(cluster_indices):
        for i, indice in enumerate(indices):
            color_cluster_point_list.append([white_cloud[indice][0],
                                             white_cloud[indice][1],
                                             white_cloud[indice][2],
                                             rgb_to_float(cluster_color[j])])

    cluster_cloud = pcl.PointCloud_PointXYZRGB()
    cluster_cloud.from_list(color_cluster_point_list)

    # TODO: Convert PCL data to ROS messages
    ros_passthrough_cloud = pcl_to_ros(passthrough_cloud)
    ros_cloud_objects = pcl_to_ros(cloud_objects)
    ros_cloud_table   = pcl_to_ros(cloud_table)
    ros_cluster_cloud = pcl_to_ros(cluster_cloud)

    # TODO: Publish ROS messages
    pcl_passthrough_pub.publish(ros_passthrough_cloud)
    pcl_objects_pub.publish(ros_cloud_objects)
    pcl_table_pub.publish(ros_cloud_table)
    pcl_cluster_pub.publish(ros_cluster_cloud)

# Exercise-3 TODOs: 

    # Classify the clusters! (loop through each detected cluster one at a time)
    detected_objects_labels = []
    detected_objects_list = []

    for index, pts_list in enumerate(cluster_indices):

        # Grab the points for the cluster from the extracted outliers (cloud_objects)
        pcl_cluster = cloud_objects.extract(pts_list)
        # TODO: convert the cluster from pcl to ROS using helper function
        ros_cluster = pcl_to_ros(pcl_cluster)
        
        # Extract histogram features
        # TODO: complete this step just as in capture_features.py
        chists = compute_color_histograms(ros_cluster, using_hsv=True)
        normals = get_normals(ros_cluster)
        nhists = compute_normal_histograms(normals)
        feature = np.concatenate((chists, nhists))
        #detected_objects_labels.append([feature, model_name])

        # Make the prediction, retrieve the label for the result
        # and add it to detected_objects_labels list
        prediction = clf.predict(scaler.transform(feature.reshape(1,-1)))
        print "prediction: %s" % str(prediction)
        label = encoder.inverse_transform(prediction)[0]
        detected_objects_labels.append(label)

        # Publish a label into Rviz
        label_pos = list(white_cloud[pts_list[0]])
        label_pos[2] += .4
        object_markers_pub.publish(make_label(label, label_pos, index))

        # Add the detected object to the list of detected objects.
        do = DetectedObject()
        do.label = label
        do.cloud = ros_cluster
        detected_objects_list.append(do)




    rospy.loginfo('Detected {} objects: {}'.format(len(detected_objects_labels), detected_objects_labels))

    # Publish the list of detected objects
    # This is the output you'll need to complete the upcoming project!
    detected_objects_pub.publish(detected_objects_list)

    # Suggested location for where to invoke your pr2_m.ver() function within pcl_callback()
    # Could add some logic to determine whether or not your object detections are robust
    # before calling pr2_mover()
    try:
        pr2_mover(detected_objects_list)
        pass
    except rospy.ROSInterruptException:
        pass

# function to load parameters and request PickPlace service
def pr2_mover(object_list):

    # TODO: Initialize variables

    # TODO: Get/Read parameters

    # TODO: Parse parameters into individual variables

    # TODO: Rotate PR2 in place to capture side tables for the collision map

    object_list_param = rospy.get_param('/object_list')
    dropbox_list_param = rospy.get_param('/dropbox')


    # TODO: Loop through the pick list
    dict_list = []
    centroids = {}
    for object in object_list:
        points_arr = ros_to_pcl(object.cloud).to_array()
        centroids[object.label] = np.mean(points_arr, axis=0)[:3]

    scene_num = 1
    if len(object_list_param)==3:
        scene_num = 1
    elif len(object_list_param)==5:
        scene_num = 2
    elif len(object_list_param)==8:
        scene_num = 3
    else:
        print "ERROR"

    test_scene_num = Int32()
    test_scene_num.data = scene_num
 
    for i in range(0, len(object_list_param)):
        object_name = String()
        object_name.data = object_list_param[i]['name']
        object_group = object_list_param[i]['group']
        if not centroids.has_key(object_name.data):
            continue

        # TODO: Create 'place_pose' for the object
        pick_pose = Pose()
        pick_pose.position.x = np.asscalar(centroids[object_name.data][0])
        pick_pose.position.y = np.asscalar(centroids[object_name.data][1])
        pick_pose.position.z = np.asscalar(centroids[object_name.data][2])
        pick_pose.orientation.x = 0.
        pick_pose.orientation.y = 0.
        pick_pose.orientation.z = 0.
        pick_pose.orientation.w = 0.
        place_pose = Pose()
        place_pose.orientation.x = 0.
        place_pose.orientation.y = 0.
        place_pose.orientation.z = 0.
        place_pose.orientation.w = 0.

        arm_name = String()
        # TODO: Assign the arm to be used for pick_place
        for d in dropbox_list_param:
            if d['group'] == object_group:
                arm_name.data = d['name']
                place_pose.position.x = d['position'][0]
                place_pose.position.y = d['position'][1]
                place_pose.position.z = d['position'][2]
                break

        # TODO: Create a list of dictionaries (made with make_yaml_dict()) for later output to yaml format
        # Populate various ROS messages
        yaml_dict = make_yaml_dict(test_scene_num, arm_name, object_name, pick_pose, place_pose)
        dict_list.append(yaml_dict)

        # Wait for 'pick_place_routine' service to come up
        #rospy.wait_for_service('pick_place_routine')

        #try:
            #pick_place_routine = rospy.ServiceProxy('pick_place_routine', PickPlace)

            # TODO: Insert your message variables to be sent as a service request
            #resp = pick_place_routine(test_scene_num, object_name, arm_name, pick_pose, place_pose)

            #print ("Response: ",resp.success)

        #except rospy.ServiceException, e:
            #print "Service call failed: %s"%e

    # TODO: Output your request parameters into output yaml file
    fname = 'output_%d.yaml' % scene_num
    send_to_yaml(fname, dict_list)



if __name__ == '__main__':

    # TODO: ROS node initialization
    rospy.init_node("pr2_project", anonymous=True)

    # TODO: Create Subscribers
    pcl_sub = rospy.Subscriber("/pr2/world/points", pc2.PointCloud2, pcl_callback, queue_size=1)

    # TODO: Create Publishers
    pcl_passthrough_pub = rospy.Publisher("/pr2/world/passthrough", PointCloud2, queue_size=1)
    pcl_objects_pub = rospy.Publisher("/pr2/world/pcl_objects", PointCloud2, queue_size=1)
    pcl_table_pub = rospy.Publisher("/pr2/world/pcl_table", PointCloud2, queue_size=1)
    pcl_cluster_pub = rospy.Publisher("/pr2/world/pcl_cluster", PointCloud2, queue_size=1)
    object_markers_pub = rospy.Publisher("/object_markers", Marker, queue_size=1)
    detected_objects_pub = rospy.Publisher("/detected_objects", DetectedObjectsArray, queue_size=1)

    
    model = pickle.load(open('model.sav', 'rb'))
    clf = model['classifier']
    encoder = LabelEncoder()
    encoder.classes_ = model['classes']
    scaler = model['scaler']

    # TODO: Load Model From disk

    # Initialize color_list
    get_color_list.color_list = []

    # TODO: Spin while node is not shutdown
    while not rospy.is_shutdown():
        rospy.spin()
