## Project: Perception Pick & Place
### Writeup Template: You can use this file as a template for your writeup if you want to submit it as a markdown file, but feel free to use some other method and submit a pdf if you prefer.

---


# Required Steps for a Passing Submission:
1. Extract features and train an SVM model on new objects (see `pick_list_*.yaml` in `/pr2_robot/config/` for the list of models you'll be trying to identify). 
2. Write a ROS node and subscribe to `/pr2/world/points` topic. This topic contains noisy point cloud data that you must work with.
3. Use filtering and RANSAC plane fitting to isolate the objects of interest from the rest of the scene.
4. Apply Euclidean clustering to create separate clusters for individual items.
5. Perform object recognition on these objects and assign them labels (markers in RViz).
6. Calculate the centroid (average in x, y and z) of the set of points belonging to that each object.
7. Create ROS messages containing the details of each object (name, pick_pose, etc.) and write these messages out to `.yaml` files, one for each of the 3 scenarios (`test1-3.world` in `/pr2_robot/worlds/`).  [See the example `output.yaml` for details on what the output should look like.](https://github.com/udacity/RoboND-Perception-Project/blob/master/pr2_robot/config/output.yaml)  
8. Submit a link to your GitHub repo for the project or the Python code for your perception pipeline and your output `.yaml` files (3 `.yaml` files, one for each test world).  You must have correctly identified 100% of objects from `pick_list_1.yaml` for `test1.world`, 80% of items from `pick_list_2.yaml` for `test2.world` and 75% of items from `pick_list_3.yaml` in `test3.world`.
9. Congratulations!  Your Done!

# Extra Challenges: Complete the Pick & Place
7. To create a collision map, publish a point cloud to the `/pr2/3d_map/points` topic and make sure you change the `point_cloud_topic` to `/pr2/3d_map/points` in `sensors.yaml` in the `/pr2_robot/config/` directory. This topic is read by Moveit!, which uses this point cloud input to generate a collision map, allowing the robot to plan its trajectory.  Keep in mind that later when you go to pick up an object, you must first remove it from this point cloud so it is removed from the collision map!
8. Rotate the robot to generate collision map of table sides. This can be accomplished by publishing joint angle value(in radians) to `/pr2/world_joint_controller/command`
9. Rotate the robot back to its original state.
10. Create a ROS Client for the “pick_place_routine” rosservice.  In the required steps above, you already created the messages you need to use this service. Checkout the [PickPlace.srv](https://github.com/udacity/RoboND-Perception-Project/tree/master/pr2_robot/srv) file to find out what arguments you must pass to this service.
11. If everything was done correctly, when you pass the appropriate messages to the `pick_place_routine` service, the selected arm will perform pick and place operation and display trajectory in the RViz window
12. Place all the objects from your pick list in their respective dropoff box and you have completed the challenge!
13. Looking for a bigger challenge?  Load up the `challenge.world` scenario and see if you can get your perception pipeline working there!

## [Rubric](https://review.udacity.com/#!/rubrics/1067/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it!

### Exercise 1, 2 and 3 pipeline implemented
#### 1. Complete Exercise 1 steps. Pipeline for filtering and RANSAC plane fitting implemented.

I implemented the first layer of filtering I used was the statistical outlier filter using an average of 100 nearest points with the tolerance set to 0.5. Both of these were derived via experimentation. The statistical outlier filter is applied in the pcl_callback function of project_TEMPLATE.PY.

	    outlier_filter = cloud.make_statistical_outlier_filter()
	
	    # Set the number of neighboring points to analyze for any given point
	    outlier_filter.set_mean_k(100)
	
	    # Set threshold scale factor - orig 1.0
	    x = 0.5
	
	    # Any point with a mean distance larger than global (mean distance+x*std_dev) will be considered outlier
	    outlier_filter.set_std_dev_mul_thresh(x)
	
	    # Finally call the filter function for magic
	    cloud = outlier_filter.filter()


The second step was to apply voxel grid downsampling. I used a leaf size of 0.005. The downsampling is defined in the pcl_callback function of project_template.py

	    # TODO: Voxel Grid Downsampling
	    LEAF_SIZE = 0.005
	    #vox = cloud_filtered.make_voxel_grid_filter()
	    vox = cloud.make_voxel_grid_filter()
	    vox.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE) 
	 
	    # Call the filter function to obtain the resultant downsampled point cloud
	    cloud_filtered = vox.filter()

The third step I used was to apply two passthrough filters. The first of these is a z-axis passthrough from 0.6 to 0.9 and the second was a y-axis passthrough from -0.5 to 0.5. The second filter was necessary to avoid the pipeline from attempting to recognize the robot arms. The passthrough filters are defined in the pcl_callback function of project_template.py.

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
	    
Here is the output from the passthrough filter:
![pass through filter](./output_3_passthrough.png  "Passthrough Filter Image")

#### 2. Complete Exercise 2 steps: Pipeline including clustering for segmentation implemented.  
RANSAC plane segmentation and the identification of inliers and outliers is implemented in the pcl_callback function of project_template.py.

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
The objects are the outliers following RANSAC plane segmentation and are shown below. 
![segmented objects](./output_3_objects.png  "Objects After Plane Segmentation")
The inliers following RANSAC plane segmentation forms the table and is shown below.
![segmented table](./output_3_table.png "Table After Plane Segmentation")
Next, the pipeline takes the objects just identified and performs euclidean clustering using a tolerance of 0.015, a minimum cluster size of 50 and a maximum cluster size of 2500. The clustering is completed within the pcl_callback function of project_template.py.

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
![clusters](./output_3_clusters.png "Clusters")
	
#### 2. Complete Exercise 3 Steps.  Features extracted and SVM trained.  Object recognition implemented.

I trained the SVM using 1000 random samples of each of the 8 objects defined in pick_place_3.yaml. The features were extracted using HSV rather than RGB. The classifier uses a linear kernel. Snippet indicating capture logic from capture_features.py:

	    for model_name in models:
	        spawn_model(model_name)
	
	        for i in range(1000):
	            print "Showing %s i: %d" % (model_name, i)
	            # make five attempts to get a valid a point cloud then give up
	            sample_was_good = False
	            try_count = 0
	            while not sample_was_good and try_count < 5:
	                sample_cloud = capture_sample()
	                sample_cloud_arr = ros_to_pcl(sample_cloud).to_array()
	
	                # Check for invalid clouds.
	                if sample_cloud_arr.shape[0] == 0:
	                    print('Invalid cloud detected')
	                    try_count += 1
	                else:
	                    sample_was_good = True
	
	            # Extract histogram features
	            chists = compute_color_histograms(sample_cloud, using_hsv=True)
	            normals = get_normals(sample_cloud)
	            nhists = compute_normal_histograms(normals)
	            feature = np.concatenate((chists, nhists))
	            labeled_features.append([feature, model_name])


The color histograms and normals are calculated in features.py: compute_color_histograms and compute_normal_histograms.
Color histogram calculation:

		    channel_1_hist = np.histogram(channel_1_vals, bins=64, range=(0,256))
		    channel_2_hist = np.histogram(channel_2_vals, bins=64, range=(0,256))
		    channel_3_hist = np.histogram(channel_3_vals, bins=64, range=(0,256))
		
		    # TODO: Concatenate and normalize the histograms
		    #print "channel_1_hist[0]: %s" % channel_1_hist[0]
		    hist_features = np.concatenate((channel_1_hist[0], channel_2_hist[0], channel_3_hist[0])).astype(np.float64)
		    # Generate random features for demo mode.  
		    # Replace normed_features with your feature vector
		    #normed_features = np.random.random(96) 
		    normed_features = hist_features / np.sum(hist_features)
		    return normed_features 
	  
Surface normal histogram calculation:

	    for norm_component in pc2.read_points(normal_cloud,
	                                          field_names = ('normal_x', 'normal_y', 'normal_z'),
	                                          skip_nans=True):
	        #print "norm_component: %f %f %f" % (norm_component[0], norm_component[1], norm_component[2])
	        norm_x_vals.append(norm_component[0])
	        norm_y_vals.append(norm_component[1])
	        norm_z_vals.append(norm_component[2])
	
	    # TODO: Compute histograms of normal values (just like with color)
	    x_hist = np.histogram(norm_x_vals, bins=64, range=(-1,1))
	    y_hist = np.histogram(norm_y_vals, bins=64, range=(-1,1))
	    z_hist = np.histogram(norm_z_vals, bins=64, range=(-1,1))
	
	    # TODO: Concatenate and normalize the histograms
	    hist_features = np.concatenate((x_hist[0], y_hist[0], z_hist[0])).astype(np.float64)
	    normed_features = hist_features / np.sum(hist_features)
	    return normed_features
	
Here are the results of training:

	kevin@kevin-XPS-13-9365:~/catkin_ws$ python ./src/RoboND-Perception-Project/pr2_robot/scripts/train_svm.py 
	/home/kevin/.local/lib/python2.7/site-packages/sklearn/cross_validation.py:41: DeprecationWarning: This module was deprecated in version 0.18 in favor of the model_selection module into which all the refactored classes and functions are moved. Also note that the interface of the new CV iterators are different from that of this module. This module will be removed in 0.20.
	  "This module will be removed in 0.20.", DeprecationWarning)
	Features in Training Set: 8000
	Invalid Features in Training set: 0
	Scores: [0.971875 0.974375 0.965    0.978125 0.9725  ]
	Accuracy: 0.97 (+/- 0.01)


Here is the unnormalized confusion matrix
![unnormalized confusion matrix](./confusion_matrix_unnormalized.png  "Unnormalized Confusion Matrix")

And here is the resulting normalized confusion matrix:
![normalized confusion matrix](./confusion_matrix_normalized.png "Normalized Confusion Matrix")

As an experiment, I tried to see the effect of the rbf kernel instead but the results on the project were worse despite producing better output in the confusion matrix. 
*Note that capture_features.py, features.py, and train_svm.py were run out of the sensor_stick environment. I have copied these scripts into the pr2_robot/scripts directory for the purposes of review.*

The complete perception pipeline exists in the pcl_callback function in project_template.py. The calculation of centroid, identification of target box, and output to yaml is defined in the pr2_mover function in project_template.py.

    centroids = {}
    for object in object_list:
        points_arr = ros_to_pcl(object.cloud).to_array()
        centroids[object.label] = np.mean(points_arr, axis=0)[:3]
        
and the assignment of centroid / box to each object in order

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
	

### Pick and Place Setup

#### 1. For all three tabletop setups (`test*.world`), perform object recognition, then read in respective pick list (`pick_list_*.yaml`). Next construct the messages that would comprise a valid `PickPlace` request output them to `.yaml` format.

Test world 1 has three objects and the system correctly recognized all three.

	kevin@kevin-XPS-13-9365:~/catkin_ws$ cat output_1.yaml 
	object_list:
	- arm_name: right
	  object_name: biscuits
	  pick_pose:
	    orientation:
	      w: 0.0
	      x: 0.0
	      y: 0.0
	      z: 0.0
	    position:
	      x: 0.5416092872619629
	      y: -0.2414039522409439
	      z: 0.7053749561309814
	  place_pose:
	    orientation:
	      w: 0.0
	      x: 0.0
	      y: 0.0
	      z: 0.0
	    position:
	      x: 0
	      y: -0.71
	      z: 0.605
	  test_scene_num: 1
	- arm_name: right
	  object_name: soap
	  pick_pose:
	    orientation:
	      w: 0.0
	      x: 0.0
	      y: 0.0
	      z: 0.0
	    position:
	      x: 0.5450282692909241
	      y: -0.018413478508591652
	      z: 0.6766939759254456
	  place_pose:
	    orientation:
	      w: 0.0
	      x: 0.0
	      y: 0.0
	      z: 0.0
	    position:
	      x: 0
	      y: -0.71
	      z: 0.605
	  test_scene_num: 1
	- arm_name: left
	  object_name: soap2
	  pick_pose:
	    orientation:
	      w: 0.0
	      x: 0.0
	      y: 0.0
	      z: 0.0
	    position:
	      x: 0.4449080228805542
	      y: 0.22129249572753906
	      z: 0.6762093901634216
	  place_pose:
	    orientation:
	      w: 0.0
	      x: 0.0
	      y: 0.0
	      z: 0.0
	    position:
	      x: 0
	      y: 0.71
	      z: 0.605
	  test_scene_num: 1
	

	
and produced the following labeled image
![test world 1](./output_1.png  "Test World 1 Labels")

Test world 2 consists of 5 objects that must be labeled. The script correctly labeled 5 of the 5 objects. 

	kevin@kevin-XPS-13-9365:~/catkin_ws$ cat output_2.yaml 
	object_list:
	- arm_name: right
	  object_name: biscuits
	  pick_pose:
	    orientation:
	      w: 0.0
	      x: 0.0
	      y: 0.0
	      z: 0.0
	    position:
	      x: 0.5712485909461975
	      y: -0.2478954792022705
	      z: 0.7054320573806763
	  place_pose:
	    orientation:
	      w: 0.0
	      x: 0.0
	      y: 0.0
	      z: 0.0
	    position:
	      x: 0
	      y: -0.71
	      z: 0.605
	  test_scene_num: 2
	- arm_name: right
	  object_name: soap
	  pick_pose:
	    orientation:
	      w: 0.0
	      x: 0.0
	      y: 0.0
	      z: 0.0
	    position:
	      x: 0.5601434707641602
	      y: 0.003147885436192155
	      z: 0.6766631007194519
	  place_pose:
	    orientation:
	      w: 0.0
	      x: 0.0
	      y: 0.0
	      z: 0.0
	    position:
	      x: 0
	      y: -0.71
	      z: 0.605
	  test_scene_num: 2
	- arm_name: left
	  object_name: book
	  pick_pose:
	    orientation:
	      w: 0.0
	      x: 0.0
	      y: 0.0
	      z: 0.0
	    position:
	      x: 0.5789046883583069
	      y: 0.280473917722702
	      z: 0.7228950262069702
	  place_pose:
	    orientation:
	      w: 0.0
	      x: 0.0
	      y: 0.0
	      z: 0.0
	    position:
	      x: 0
	      y: 0.71
	      z: 0.605
	  test_scene_num: 2
	- arm_name: left
	  object_name: soap2
	  pick_pose:
	    orientation:
	      w: 0.0
	      x: 0.0
	      y: 0.0
	      z: 0.0
	    position:
	      x: 0.44437068700790405
	      y: 0.22667980194091797
	      z: 0.6764086484909058
	  place_pose:
	    orientation:
	      w: 0.0
	      x: 0.0
	      y: 0.0
	      z: 0.0
	    position:
	      x: 0
	      y: 0.71
	      z: 0.605
	  test_scene_num: 2
	- arm_name: left
	  object_name: glue
	  pick_pose:
	    orientation:
	      w: 0.0
	      x: 0.0
	      y: 0.0
	      z: 0.0
	    position:
	      x: 0.6309707760810852
	      y: 0.13058094680309296
	      z: 0.6787865161895752
	  place_pose:
	    orientation:
	      w: 0.0
	      x: 0.0
	      y: 0.0
	      z: 0.0
	    position:
	      x: 0
	      y: 0.71
	      z: 0.605
	  test_scene_num: 2

	
and produced the following image:

![Test World 2](./output_2.png  "World 2 Labels")

Finally, world3 consists of 8 objects and the system correctly labeled 8 of the 8 objects. Here is the yaml output:

		kevin@kevin-XPS-13-9365:~/catkin_ws$ cat output_3.yaml 
		object_list:
		- arm_name: left
		  object_name: sticky_notes
		  pick_pose:
		    orientation:
		      w: 0.0
		      x: 0.0
		      y: 0.0
		      z: 0.0
		    position:
		      x: 0.43965840339660645
		      y: 0.21510088443756104
		      z: 0.6861762404441833
		  place_pose:
		    orientation:
		      w: 0.0
		      x: 0.0
		      y: 0.0
		      z: 0.0
		    position:
		      x: 0
		      y: 0.71
		      z: 0.605
		  test_scene_num: 3
		- arm_name: left
		  object_name: book
		  pick_pose:
		    orientation:
		      w: 0.0
		      x: 0.0
		      y: 0.0
		      z: 0.0
		    position:
		      x: 0.4925074875354767
		      y: 0.08395363390445709
		      z: 0.7267417907714844
		  place_pose:
		    orientation:
		      w: 0.0
		      x: 0.0
		      y: 0.0
		      z: 0.0
		    position:
		      x: 0
		      y: 0.71
		      z: 0.605
		  test_scene_num: 3
		- arm_name: right
		  object_name: snacks
		  pick_pose:
		    orientation:
		      w: 0.0
		      x: 0.0
		      y: 0.0
		      z: 0.0
		    position:
		      x: 0.42672255635261536
		      y: -0.33367329835891724
		      z: 0.7533994317054749
		  place_pose:
		    orientation:
		      w: 0.0
		      x: 0.0
		      y: 0.0
		      z: 0.0
		    position:
		      x: 0
		      y: -0.71
		      z: 0.605
		  test_scene_num: 3
		- arm_name: right
		  object_name: biscuits
		  pick_pose:
		    orientation:
		      w: 0.0
		      x: 0.0
		      y: 0.0
		      z: 0.0
		    position:
		      x: 0.588325023651123
		      y: -0.218833789229393
		      z: 0.7053137421607971
		  place_pose:
		    orientation:
		      w: 0.0
		      x: 0.0
		      y: 0.0
		      z: 0.0
		    position:
		      x: 0
		      y: -0.71
		      z: 0.605
		  test_scene_num: 3
		- arm_name: left
		  object_name: eraser
		  pick_pose:
		    orientation:
		      w: 0.0
		      x: 0.0
		      y: 0.0
		      z: 0.0
		    position:
		      x: 0.6080653071403503
		      y: 0.2820051908493042
		      z: 0.6472211480140686
		  place_pose:
		    orientation:
		      w: 0.0
		      x: 0.0
		      y: 0.0
		      z: 0.0
		    position:
		      x: 0
		      y: 0.71
		      z: 0.605
		  test_scene_num: 3
		- arm_name: right
		  object_name: soap2
		  pick_pose:
		    orientation:
		      w: 0.0
		      x: 0.0
		      y: 0.0
		      z: 0.0
		    position:
		      x: 0.45350995659828186
		      y: -0.043533604592084885
		      z: 0.6761820912361145
		  place_pose:
		    orientation:
		      w: 0.0
		      x: 0.0
		      y: 0.0
		      z: 0.0
		    position:
		      x: 0
		      y: -0.71
		      z: 0.605
		  test_scene_num: 3
		- arm_name: right
		  object_name: soap
		  pick_pose:
		    orientation:
		      w: 0.0
		      x: 0.0
		      y: 0.0
		      z: 0.0
		    position:
		      x: 0.6790318489074707
		      y: 0.004095396958291531
		      z: 0.6763603091239929
		  place_pose:
		    orientation:
		      w: 0.0
		      x: 0.0
		      y: 0.0
		      z: 0.0
		    position:
		      x: 0
		      y: -0.71
		      z: 0.605
		  test_scene_num: 3
		- arm_name: left
		  object_name: glue
		  pick_pose:
		    orientation:
		      w: 0.0
		      x: 0.0
		      y: 0.0
		      z: 0.0
		    position:
		      x: 0.6115730404853821
		      y: 0.14189209043979645
		      z: 0.6838186383247375
		  place_pose:
		    orientation:
		      w: 0.0
		      x: 0.0
		      y: 0.0
		      z: 0.0
		    position:
		      x: 0
		      y: 0.71
		      z: 0.605
		  test_scene_num: 3

	

And, finally, here is the labeled output from test world 3.
![test world 3](./output_3b.png  "Test World 3 Labels")

##Results Discussion
The object recognition worked well but was not perfect. In particular, a single run of the process will continuously generate new labels. Sometimes not all objects were detected. Sometimes an object was mislabled. Most notably, world 3 produced the glue object in a partially occluded setting. Sometimes this object would be mislabeled. In such an instance, the results of the object recognition would probably be better had I completed the delivery of the objects into the respective boxes. After delivery of the occluding object, a clear view of the occluded object is possible resulting in likely a much better point cloud. 

Another area of potential improvement is to have the algorithm make the assumption that there is only 1 of each object available within the world. This assumption is certainly true in the test scenarios. With this assumption, we would pick the best example of the given target object and then exclude that object as a possible answer for all other labels. I took a step towards this solution by training the classifier with the probability flag set and generating the output of the predict_proba rather than just the predict output. However, with correct labels for all objects I determined this last step was unnecessary and left it as an avenue for future exploration.


