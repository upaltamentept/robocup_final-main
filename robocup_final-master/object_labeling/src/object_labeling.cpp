#include <object_labeling/object_labeling.h>

ObjectLabeling::ObjectLabeling(
    const std::string& objects_cloud_topic_, 
    const std::string& camera_info_topic,
    const std::string& camera_frame) :
  is_cloud_updated_(false),
  has_camera_info_(false),
  objects_cloud_topic_(objects_cloud_topic_),
  camera_info_topic_(camera_info_topic),
  camera_frame_(camera_frame),
  K_(Eigen::Matrix3d::Zero())
{
}

ObjectLabeling::~ObjectLabeling()
{
}

bool ObjectLabeling::initalize(ros::NodeHandle& nh)
{
  //subscribe to objects pointcloud and table point cloud published by the plane_segmentation_node
  object_point_cloud_sub_ = nh.subscribe<sensor_msgs::PointCloud2>(objects_cloud_topic_, 100, &ObjectLabeling::cloudCallback, this);
  table_point_cloud_sub_ = nh.subscribe<sensor_msgs::PointCloud2>("/table_plane_pointcloud", 100, &ObjectLabeling::tablecloudCallback, this);

  //subscribe to bounding boxes from yolo (object_labeling_node)
  object_detections_sub_ = nh.subscribe<darknet_ros_msgs::BoundingBoxes>("/darknet_ros/bounding_boxes", 100, &ObjectLabeling::detectionCallback, this);
  //subscribe to camera info from robot to obtain the camera matrix K
  camera_info_sub_ = nh.subscribe<sensor_msgs::CameraInfo>(camera_info_topic_, 100, &ObjectLabeling::cameraInfoCallback, this);
  
  //euecute once in order to avoid programm die
  execute_sub_ = nh.subscribe("/find_target", 1, &ObjectLabeling::ExecuteCallback, this);

  //publish the labled objects as LabeledCentroid type (see typedefs in header)
  labeled_object_centroid_pub_ = nh.advertise<project_msgs::LabeledCentroid>("/labeled_objects", 10);

  //publish remainder object and table as obstacle
  labeled_remainder_cloud_pub_ = nh.advertise<project_msgs::CollisionGeometry>("/remainder_objects", 10);

  // publish the LABELED object names as visulaization marker (http://wiki.ros.org/rviz/DisplayTypes/Marker)
  //text_marker_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/visualization_marker_array",10);

  // init internal pointclouds for processing (again pcl uses pointers)
  object_point_cloud_.reset(new PointCloud);    // holds unlabled object point cloud
  //labeled_point_cloud_.reset(new PointCloudl);  // holds labled object point cloud
  remaind_point_cloud_.reset(new PointCloud);
  table_point_cloud_.reset(new PointCloud);

  //#category 
  dict_["sports ball"] = 2;
  dict_["bottle"] = 1;
  dict_["cup"] = 1;
  dict_["banana"] = 2;
  dict_["apple"] = 2;
  dict_["bowl"] = 1;
  dict_["traffic light"] = 2;
  dict_["orange"] = 2;
  dict_["remote"] = 1;
  dict_["vase"] = 2;
  dict_["cell phone"] = 1;
  // ... bananna, cup, apple, ...
   

  return true;
}

void ObjectLabeling::ExecuteCallback(const std_msgs::Bool::ConstPtr& msg)
{
  // camera info and point cloud available
  //ROS_INFO_STREAM("running: " << is_cloud_updated_);   
  //ros::Duration(4.0).sleep();
  if(is_cloud_updated_ && has_camera_info_)
  {
    is_cloud_updated_ = false;
    // label the objects in pointcloud based on 2d bounding boxes 
    if(!labelObjects(object_point_cloud_, labeled_centroid_, remaind_point_cloud_))
      return;

    //publish labeled_point_cloud_ to ros
    labeled_object_centroid_pub_.publish(labeled_centroid_);
    
    //  publish remainder pointcloud
    labeled_remainder_cloud_pub_.publish(collision_objects);
    //publish text_markers_ to ros
    
    //text_marker_pub_.publish<visualization_msgs::MarkerArray>(text_markers_);

  }
}

bool ObjectLabeling::labelObjects(CloudPtr& input, project_msgs::LabeledCentroid& output, CloudPtr& remainders)
{
  //#>>>>GOAL: Split input pointcloud into seperate blobs, compute centroid,
  //#>>>>GOAL: project centorid into the camera image and match with bounding box
  //#>>>>GOAL: finally label the pointcloud with object type

  // First we need to cluster the input cloud into seperated clusters,
  // each of them represents an object on the table.

  //Use EuclideanClusterExtraction to seperate the pointcloud into clusters
  pcl::EuclideanClusterExtraction<PointT> ec;
  
  // General settings
  ec.setClusterTolerance (0.1); // 10cm
  ec.setMinClusterSize (30);
  ec.setMaxClusterSize (500);

  pcl::search::Search<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
  tree -> setInputCloud(input);
  ec.setSearchMethod (tree);

  ec.setInputCloud(input);

  // holds the extracted cluster indices (just a integer for identifiction)
  std::vector<pcl::PointIndices> cluster_indices;

  ec.extract(cluster_indices);
  // ROS_INFO_STREAM("Number of clusters = " << cluster_indices.size());

  // Iterate over each cluster and compute its centroid point (= mean)
  //Push the centroid into the vector of centroids
  std::vector<Eigen::Vector3d> centroids;

  std::vector<pcl::PointIndices>::const_iterator cit;
  for (cit = cluster_indices.begin(); cit != cluster_indices.end(); ++cit)
  {
    Eigen::Vector3d centroid(0.0, 0.0, 0.0);
    std::vector<int>::const_iterator iterator;
    for (iterator = cit->indices.begin(); iterator != cit->indices.end(); ++iterator)
    {
      PointT point = input->points[*iterator];
      centroid[0] += point.x;
      centroid[1] += point.y;
      centroid[2] += point.z;
    }
    int num_points = cit->indices.size();
    centroid /= num_points;
    centroids.push_back(centroid);
    // ROS_INFO_STREAM("centroid pos: " << centroid.transpose());
  }
  // Next we need to find the pixel coordinates of the centroids within the 2d
  // camera image. This projection is handled by the camera matrix
  // First, the centorids need to be transformed from the pointcloud frame into the
  // camera frame.

  //Get the homogenous transformation matrix of the base frame with respect
  //to the camera frame.
  tf::StampedTransform transform;
  try
  {
    tfListener_.lookupTransform(camera_frame_,input->header.frame_id , ros::Time(0), transform);
  }
  catch (tf::TransformException& ex)
  {
    ROS_ERROR("%s", ex.what());
    return false;
  }

  //Convert the tf::StampedTransform into an Eigen::Affine3d
  Eigen::Affine3d T_base_camera; // = ?;
  tf::transformTFToEigen(transform, T_base_camera);

  //Transform the centorids into the  camera frame by multiplying them 
  //with the transformation that takes a point in the pointcloud frame and turns it
  //into a point in the camera frame. Do this for all centroids.
  std::vector<Eigen::Vector3d> centroids_camera;
  for (size_t i = 0; i < centroids.size(); ++i)
  {
    Eigen::Vector3d centroid_camera = T_base_camera * centroids[i];
    //ROS_INFO_STREAM("centroid cam pos: " << centroid_camera.transpose());

    centroids_camera.push_back(centroid_camera);
  }

  //Project the transformed centorids into the camera plane by using the camera matrix K
 
  std::vector<Eigen::Vector2d> pixel_centroids; // = ?
  for (size_t i = 0; i < centroids_camera.size(); ++i)
  {
    Eigen::Vector3d centroid_3d = centroids_camera[i];
    Eigen::Vector3d pixel_coord = K_ * centroid_3d;
    Eigen::Vector2d pixel_coord_2d(pixel_coord[0] / pixel_coord[2], pixel_coord[1] / pixel_coord[2]);
    pixel_centroids.push_back(pixel_coord_2d);
    //ROS_INFO_STREAM("Pixel coord" << pixel_coord_2d.transpose());
  }

  // Now the centorids of each cluster are given as pixel coordinates in the 2d image
  // plane of the camera. What remains is to find the bounding box that matches to each of 
  // those controids. 

  //Find the best match between pixel_centroids and detections_
  // Use the euclidian distance between the pixel_centroids and the boundingbox centers
  //For each bounding box find the closest cenroid 
  //If a cluster cant be matched (no bounding boxes left) assign 0 as label

  std::vector<int> assigned_labels(cluster_indices.size(), 0);                  // lables of each centroid
  std::vector<std::string> assigned_classes(cluster_indices.size(), "unknown"); // class names of each centroid

  // ROS_INFO_STREAM("Number of bounding boxes = " << detections_.size());
  for(size_t i = 0; i < detections_.size(); ++i)
  {
    // get the bounding box we want to find the closest cenroid 
    darknet_ros_msgs::BoundingBox& bounding_box = detections_[i];
    double bounding_box_center_x = (bounding_box.xmax + bounding_box.xmin) / 2;
    double bounding_box_center_y = (bounding_box.ymax + bounding_box.ymin) / 2;
    Eigen::Vector2d box_center(bounding_box_center_x, bounding_box_center_y);
    // ROS_INFO_STREAM("Box center: " << box_center.transpose());
    //For all cenroids compute the distance to the boudning box
    //select the clostes as match and get its index in pixel_centroids
    double minDistance = std::numeric_limits<double>::max(); 
    int match = -1; 

    for (size_t j = 0; j < pixel_centroids.size(); ++j) 
    { 
      //ROS_INFO_STREAM("pixel: " << pixel_centroids.size());
      
      double distance = std::sqrt((pixel_centroids[j][0]-bounding_box_center_x)*(pixel_centroids[j][0]-bounding_box_center_x) + (pixel_centroids[j][1]-bounding_box_center_y)*(pixel_centroids[j][1]-bounding_box_center_y));
     
      if (distance < minDistance) 
      {
        minDistance = distance;
        match = j;
      }
    }
    
    // remember the label of match
    if(dict_.find(bounding_box.Class) != dict_.end())
    {
      assigned_labels[match] = dict_[bounding_box.Class]; // set match to defined class index
      assigned_classes[match] = bounding_box.Class;       // set match to class name
      
    }
  }

  // ROS_INFO_STREAM("Assigned output: " << assigned_classes.size());

  // relabel the point cloud
  //output->points.clear();
  //output.pose.header  input->header;
  
  remainders->points.clear();
  remainders->header = input->header;
  PointT pt;
  PointT minPt, maxPt;//define points to store min and max values
  int i = 0;
  //initial the collision object
  collision_objects.x.clear();
  collision_objects.y.clear();
  collision_objects.z.clear();
  collision_objects.length.clear();
  collision_objects.width.clear();
  collision_objects.height.clear();
  cit = cluster_indices.begin();
  for(; cit != cluster_indices.end(); ++cit, ++i ) 
  {
    // relabel all the points inside cluster
    ROS_INFO_STREAM("num_collision: " << i);    

    std::vector<int>::const_iterator it = cit->indices.begin();
    for(; it != cit->indices.end(); ++it ) 
    { 
      PointT& cpt = input->points[*it];
      pt.x = cpt.x;
      pt.y = cpt.y;
      pt.z = cpt.z;
      //ROS_INFO_STREAM("running" << pixel_centroids.size());

      //the centorid of the target object
      if(i==pixel_centroids.size()-1)
      {
        output.pose.pose.position.x = centroids[i](0);
        output.pose.pose.position.y = centroids[i](1);
        output.pose.pose.position.z = centroids[i](2);
        output.label = assigned_labels[i]; 
        continue;
        }
      else
      {
        //obstacle objects
        remainders->points.push_back( pt );
        }
      
    }
    // store the remainder object
      if(i==pixel_centroids.size()-1)
      {
        continue;
      }
      pcl::getMinMax3D(*remainders, minPt, maxPt);
      collision_objects.x.push_back((maxPt.x + minPt.x)/2);
      collision_objects.y.push_back((maxPt.y + minPt.y)/2);
      collision_objects.z.push_back((maxPt.z + minPt.z)/2);
      collision_objects.length.push_back((maxPt.x - minPt.x));
      collision_objects.width.push_back((maxPt.y - minPt.y));
      collision_objects.height.push_back((maxPt.z - minPt.z));
      remainders->points.clear();


  }
  // calculate the min and max value of remainder object 
  pcl::getMinMax3D(*table_point_cloud_, minPt, maxPt);
  collision_objects.x.push_back((maxPt.x + minPt.x)/2);
  collision_objects.y.push_back((maxPt.y + minPt.y)/2);
  collision_objects.z.push_back((maxPt.z + minPt.z)/2);
  collision_objects.length.push_back((maxPt.x - minPt.x));
  collision_objects.width.push_back((maxPt.y - minPt.y));
  collision_objects.height.push_back((maxPt.z - minPt.z));
  // create a text marker that displays the assigned class name (assigned_classes) 
  // at the 3d position of the corresponding centroid
/*   text_markers_.markers.resize(assigned_classes.size());
  for(size_t i = 0; i < assigned_classes.size(); ++i)
  {
    visualization_msgs::Marker marker;
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.text = assigned_classes[i];
    marker.pose.position.x = centroids[i][0];
    marker.pose.position.y = centroids[i][1];
    marker.pose.position.z = centroids[i][2] + 0.1;
    marker.color.a = 1.0;
    marker.scale.z = 0.1;
    marker.id = i;
    marker.header.frame_id = input->header.frame_id;
    marker.header.stamp = ros::Time::now();
    text_markers_.markers[i] = marker;
  } */

  return true;
}


void ObjectLabeling::cloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
  // convert to pcl
  //is_cloud_updated_ = true;
  //convert to pcl and store in object_point_cloud_
  pcl::fromROSMsg(*msg, *object_point_cloud_);
}

void ObjectLabeling::tablecloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
  // convert to pcl
  is_cloud_updated_ = true;

  pcl::fromROSMsg(*msg, *table_point_cloud_);
}

void ObjectLabeling::detectionCallback(const darknet_ros_msgs::BoundingBoxesConstPtr &msg)
{
  //copy the YOLO bounding boxes
  detections_.clear();
  for (const darknet_ros_msgs::BoundingBox& b_box : msg->bounding_boxes)
  {
    detections_.push_back(b_box);
  }
}

void ObjectLabeling::cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr &msg)
{
  // copy camera info
  has_camera_info_ = true;
  Eigen::Matrix3d K = Eigen::Matrix3d::Zero();

  //copy the 3x3 camera matrix to K_
  for(size_t i = 0; i < 9; ++i)
  {
    K(i) = msg->K[i];
  }
  K_ = K.transpose();
  //ROS_INFO_STREAM("K = " << K);
}