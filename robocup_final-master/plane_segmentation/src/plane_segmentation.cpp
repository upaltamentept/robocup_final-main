
#include <plane_segmentation/plane_segmentation.h>

PlaneSegmentation::PlaneSegmentation(
    const std::string& pointcloud_topic, 
    const std::string& base_frame) :
  pointcloud_topic_(pointcloud_topic),
  base_frame_(base_frame),
  is_cloud_updated_(false)
{
}

PlaneSegmentation::~PlaneSegmentation()
{
}

bool PlaneSegmentation::initalize(ros::NodeHandle& nh)
{
  // subscribe to the pointcloud_topic_ and link it to the right callback
  point_cloud_sub_ = nh.subscribe<sensor_msgs::PointCloud2>(pointcloud_topic_, 100, &PlaneSegmentation::cloudCallback, this);

  // advertise the pointcloud for the table plane
  plane_cloud_pub_ = nh.advertise<PointCloud>("/table_plane_pointcloud", 100);

  // advertise the pointcloud for the remaining points (objects)
  objects_cloud_pub_ = nh.advertise<PointCloud>("/object_pointcloud", 100);
  object_labeled_sub_ = nh.subscribe<project_msgs::LabeledCentroid>("/labeled_objects", 1, &PlaneSegmentation::ObjectPoseCallback, this);
  pick_or_place_sub_ = nh.subscribe<std_msgs::Bool>("/place_done", 1, &PlaneSegmentation::PlacedoneCallback, this);
    
  grasp_done_sub_ = nh.subscribe<std_msgs::Bool>("/gripper_succeed", 1, &PlaneSegmentation::GraspdoneCallback, this);

  // Most PCL functions accept pointers as their arguments, as such we first set
  // initalize these pointers, otherwise we will run into segmentation faults...
  raw_cloud_.reset(new PointCloud);
  preprocessed_cloud_.reset(new PointCloud);
  plane_cloud_.reset(new PointCloud);
  objects_cloud_.reset(new PointCloud);

  //initial parameters
  filter_high = std::numeric_limits<float>::max();
  filter_low = 0.4;
  filter_x = 1.4;
  ROS_INFO_STREAM("Initialized");
  return true;
}

void PlaneSegmentation::update(const ros::Time& time)
{
  // update as soon as new pointcloud is available
  if(is_cloud_updated_)
  {
    is_cloud_updated_ = false;

    //  To check preProcessCloud() you can publish its output for testing
    // apply all preprocessing steps
    if(!preProcessCloud(raw_cloud_, preprocessed_cloud_))
      return;

    // segment cloud into table and objects
    if(!segmentCloud(preprocessed_cloud_, plane_cloud_, objects_cloud_))
      return;

    // publish both pointclouds obtained by segmentCloud()
    objects_cloud_pub_.publish(*objects_cloud_);
    plane_cloud_pub_.publish(*plane_cloud_);
  }
}

bool PlaneSegmentation::preProcessCloud(CloudPtr& input, CloudPtr& output)
{
  //#>>>>Goal: Subsample and Filter the pointcloud

  //  Raw pointclouds are typically to dense and need to be made sparse
  // Down sample the pointcloud using VoxelGrid, save result in ds_cloud
  // See https://pcl.readthedocs.io/projects/tutorials/en/master/voxel_grid.html#voxelgrid 
  // Set useful parameters

  pcl::VoxelGrid<PointT> voxel_grid;
  voxel_grid.setInputCloud(input);
  voxel_grid.setLeafSize(0.01f, 0.01f, 0.01f);

  PointCloud::Ptr ds_cloud(new PointCloud);            // downsampled pointcloud
  
  voxel_grid.filter(*ds_cloud);

  //  Its allways a good idea to get rid of useless points first (e.g. floor, ceiling, walls, etc.)
  // Transform the point cloud to the base_frame of the robot. (A frame with z=0 at ground level)
  // Transform the point cloud to the base_frame and store the result in transf_cloud
  // use pcl_ros::transformPointCloud

  CloudPtr transf_cloud(new PointCloud);        // transformed pointcloud (expressed in base frame)
  CloudPtr filtered_output(new PointCloud);
  // Transform the point cloud to the base_frame link.
  pcl_ros::transformPointCloud(base_frame_, *ds_cloud, *transf_cloud, tfListener_);

  // Trim points lower than some z_min to remove the floor from the point cloud.
  // use pcl::PassThrough filter and save result in output
  // https://pcl.readthedocs.io/projects/tutorials/en/master/passthrough.html#passthrough
  ROS_INFO_STREAM("para high"<<filter_high);
  ROS_INFO_STREAM("para low"<<filter_low);

  //set different parameter for  different parameter
  pcl::PassThrough<PointT> pass_through;
  pass_through.setInputCloud(transf_cloud);
  pass_through.setFilterFieldName("z");
  pass_through.setFilterLimits(filter_low, filter_high);
  pass_through.filter(*filtered_output);
  pass_through.setInputCloud(filtered_output);
  pass_through.setFilterFieldName("x"); 
  pass_through.setFilterLimits(0.1, filter_x);
  pass_through.filter(*output);
  // /*Uncomment only for testing*/ ROS_INFO_STREAM("Preprocess done");
  return true;
}

bool PlaneSegmentation::segmentCloud(CloudPtr& input, CloudPtr& plane_cloud, CloudPtr& objects_cloud)
{
  //#>>>>Goal: Remove every point that is not an object from the objects_cloud cloud

  // We will use Ransac to segment the pointcloud, here we setup the objects we need for this
  pcl::SACSegmentation<PointT> seg;
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

  // set parameters of the SACS segmentation
  // set correct model, play with DistanceThreshold and the free outlier probability
  // then segment the input point cloud
  //  Checkout the pcl tutorials on plane segmentation
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(100);
  seg.setDistanceThreshold(0.01); //0.01
  seg.setProbability(0.95);

  seg.setInputCloud(input);
  seg.segment(*inliers, *coefficients);


  // save inliers in plane_cloud 
  //  These sould be point that belong to the table 
  pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud(input);
  extract.setIndices(inliers);
  extract.filter(*plane_cloud);

  // save outliers in the objects_cloud
  //  This should be the rest
  extract.setNegative(true);
  extract.filter(*objects_cloud);

  // Next, we further refine the the objects_cloud by transforming it into the coordinate frame
  // of the fitted plane. In this transformed frame we remove everything below the table plane and 
  // everything more than 20 cm above the table.
  // Basically, a table aligned bounding box

  // if the plane fit is correct it will result in the coefficients = [nx, ny, nz, d]
  // where n = [nx, ny, nz] is the 3d normal vector perpendicular to the plane
  // and d the distance to the origin
  if(coefficients->values.empty())
    return false;

  // extract the normal vector 'n' perpendicular to the plane and the scalar distance 'd'
  // to the origin from the plane coefficions.
  //  As always we use Eigen to represent vectors and matices 
  //  https://eigen.tuxfamily.org/dox/GettingStarted.html
  Eigen::Vector3f n(coefficients->values[0], coefficients->values[1], coefficients->values[2]); // = ?
  double d = coefficients->values[3]; // = ?
  
  // Now we construct an Eigen::Affine3f transformation T_plane_base that describes the table plane 
  // with respect to the base_link frame using n and d

  // Build the Rotation (Quaterion) from the table's normal vector n
  // And the floor (world) normal vector: [0,0,1]
  // Use Eigen::Quaternionf::FromTwoVectors()
  Eigen::Quaternionf Q_plane_base = Eigen::Quaternionf::FromTwoVectors(n, Eigen::Vector3f(0, 0, 1)); // = ?

  // Build the translation (Vector3) from the table's normal vector n and distance
  // to the origin 
  Eigen::Vector3f t_plane_base = d*n; // = ?

  // Finally we create the Homogenous transformation of the table
  Eigen::Affine3f T_plane_base = Eigen::Affine3f::Identity();
  T_plane_base.rotate(Q_plane_base.toRotationMatrix());
  T_plane_base.translate(t_plane_base);
  //ROS_INFO_STREAM("table"<<T_plane_base);
  // Transform the objects_cloud into the table frame and store in transf_cloud
  // Use the function pcl::transformPointCloud() with T_plane_base as input
  //#>>>>https://pcl.readthedocs.io/projects/tutorials/en/latest/matrix_transform.html
  CloudPtr transf_cloud(new PointCloud);
  pcl::transformPointCloud(*objects_cloud, *transf_cloud, T_plane_base);

  // filter everything directly below the table and above it (z > 0.01 && < 0.15) 
  // using pcl::PassThrough filter (same as before)
  pcl::PassThrough<PointT> pass;
  pass.setInputCloud(transf_cloud);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(0.01, 0.15);
  
  CloudPtr filterd_cloud(new PointCloud);

  pass.filter(*filterd_cloud);

  // transform back to base_link frame using the inverse transformation
  // and store result in objects_cloud. Object cloud should only contain points associated to objects
  // Eigen::Affine3f has an inverse function
  CloudPtr filtered_object(new PointCloud);
  pcl::transformPointCloud(*filterd_cloud, *filtered_object, T_plane_base.inverse());
  pcl::PassThrough<PointT> pass_through;
  pass_through.setInputCloud(filtered_object);
  pass_through.setFilterFieldName("x"); // filter the wall
  pass_through.setFilterLimits(0.1,1.2);
  pass_through.filter(*objects_cloud);
  return true;
}

void PlaneSegmentation::cloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
  // /*Uncomment only for testing*/ ROS_INFO_STREAM("Calling callback");
  // convert ros msg to pcl raw_cloud
  is_cloud_updated_ = true;

  // Convert the msg to the internal variable raw_cloud_ that holds the raw input pointcloud 
  pcl::fromROSMsg(*msg, *raw_cloud_);
  // pcl::fromROSMsg() can do the job
}

void PlaneSegmentation::ObjectPoseCallback(const project_msgs::LabeledCentroid::ConstPtr& msg)//messsage type)
{
  object_label = msg->label; //target label
  
}

//when grasp done, update the parameter
void PlaneSegmentation::GraspdoneCallback(const std_msgs::Bool::ConstPtr& msg)//messsage type)
{
  bool grasp_done = msg->data;
  if (grasp_done==true)
    {
      if(object_label == 1)
    {
      filter_high = std::numeric_limits<float>::max();
      filter_low = 0.75;
      filter_x = 1.5;
    }
    else if (object_label == 2)
    {
      filter_high = 0.7;
      filter_low = 0.35;
      filter_x = 1.5;
    }
    else
    {
      filter_high = std::numeric_limits<float>::max();
      filter_low = 0.4;
      filter_x = 1.4;
    }
    }
}
//after placing, update the parameter
void PlaneSegmentation::PlacedoneCallback(const std_msgs::Bool::ConstPtr& msg)//messsage type)
{
    filter_high = std::numeric_limits<float>::max();
    filter_low = 0.4;
    filter_x = 1.4;
}