// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"


//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
    typename pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
    // Create the filtering object
    pcl::VoxelGrid<PointT> vg;
    vg.setInputCloud (cloud);
    vg.setLeafSize (filterRes, filterRes, filterRes); //x,y,z are some as its voxel grid
    vg.filter (*cloud_filtered);

    typename pcl::PointCloud<PointT>::Ptr cloud_roi (new pcl::PointCloud<PointT>);
    pcl::CropBox<PointT> roi;
    roi.setMin(minPoint);
    roi.setMax(maxPoint);
    roi.setInputCloud(cloud_filtered);
    roi.filter(*cloud_roi);

    //create a box to filter out the roof
    std::vector<int> roof_indices;
    pcl::CropBox<PointT> roof_roi;
    roof_roi.setMin(Eigen::Vector4f(-3,-2,-1,1));
    roof_roi.setMax(Eigen::Vector4f(3,2,1,1));
    roof_roi.setInputCloud(cloud_roi);
    roof_roi.filter(roof_indices);

    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    for (int point : roof_indices)
        inliers->indices.push_back(point);

    // Create the filtering object
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud_roi);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloud_roi);


    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloud_roi;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{

    // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr obstacle_cloud (new pcl::PointCloud<PointT> ());
    typename pcl::PointCloud<PointT>::Ptr road_cloud (new pcl::PointCloud<PointT> ());

    // Create the filtering object
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*road_cloud);
    extract.setNegative(true);
    extract.filter(*obstacle_cloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstacle_cloud, road_cloud);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	//pcl::PointIndices::Ptr inliers;
    // TODO:: Fill in this function to find inliers for the cloud.

    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<PointT> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (distanceThreshold);
    seg.setMaxIterations (maxIterations);

    // Segment the largest planar component from the cloud
    seg.setInputCloud (cloud);
    seg.segment (*inliers, *coefficients); // we are not going to use the coefficients

    if (inliers->indices.size () == 0)
    {
        PCL_ERROR ("Could not estimate a planar model for the given dataset.\n");
    }
    else
    {
        std::cout << "There are " <<  inliers->indices.size () << " inliers" << std::endl;
    }

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    return segResult;
}

template<typename PointT>
std::unordered_set<int> ProcessPointClouds<PointT>::Ransac3d(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
    std::unordered_set<int> inliersResult;
    srand(time(NULL));
    
    // TODO: Fill in this function
    //find the total number of cloud points
    auto no_of_cloud_points = cloud->points.size();

    //std::cout << "no_of_cloud_points" << no_of_cloud_points <<std::endl;

    while(maxIterations) { // For max iterations 

        std::unordered_set<int> inliers;
        auto max_inliers = 0;

        while(inliers.size() < 3)
            inliers.insert(rand() % no_of_cloud_points);

        //point coordinates for the line
        auto index_ptr = inliers.begin();
        auto x1 = cloud->points[*index_ptr].x;
        auto y1 = cloud->points[*index_ptr].y;
        auto z1 = cloud->points[*index_ptr].z;
        index_ptr++;
        auto x2 = cloud->points[*index_ptr].x;
        auto y2 = cloud->points[*index_ptr].y;
        auto z2 = cloud->points[*index_ptr].z;
        index_ptr++;
        auto x3 = cloud->points[*index_ptr].x;
        auto y3 = cloud->points[*index_ptr].y;
        auto z3 = cloud->points[*index_ptr].z;

        //generate cross product values
        auto i = ((y2-y1)*(z3-z1))-((z2-z1)*(y3-y1));
        auto j = ((z2-z1)*(x3-x1))-((x2-x1)*(z3-z1));
        auto k = ((x2-x1)*(y3-y1))-((y2-y1)*(x3-x1));

        //generate the palne parametes Ax+By+Cz+D=0
        auto A = i;
        auto B = j;
        auto C = k;
        auto D = -((i*x1)+(j*y1)+(k*z1));

        //iterate through all the points and find if they are inliers
        for(int index = 0; index < cloud->points.size(); index++)
        {
            if(inliers.count(index)>0) //if the point(index - passed as count parameter) is already in our inlier list
                continue;

            auto i = cloud->points[index];
            //std::cout<< "Selected point for distance measurement is x=" << i.x << " and y=" << i.y << std::endl;

            //find the distance d = |Ax+By+C|/sqrt(A^2+B^2)
            auto distance = fabs((A*i.x) + (B*i.y) + (C*i.z) + D)/ sqrt((A*A)+(B*B)+(C*C));

            //std::cout << "distance from the point to the plane is " << distance << std::endl;

            if( distance < distanceTol) //inside tolerance
            {
                inliers.insert(index); //save the index
            }
        }

        if (inliers.size() > inliersResult.size())
        {
            inliersResult = inliers; //this is the plane with maximum inliers. So save it
        }
        
        // Measure distance between every point and fitted line
        // If distance is smaller than threshold count it as inlier

        maxIterations--; //decrease the iteration
    }

    // Return indicies of inliers from fitted line with most inliers
    
    return inliersResult;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SFNDprj_SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    std::unordered_set<int> inliersPlane;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult;
    //SFND_Ransac<PointT> Ransac;

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    
    //inliersPlane = Ransac.Ransac3d(cloud, maxIterations, distanceThreshold);
    inliersPlane = Ransac3d(cloud, maxIterations, distanceThreshold);

    /* solution using SeparateClouds function
    
    for (auto i : inliersPlane) {
        inliers->indices.push_back(i);
    }

    segResult = SeparateClouds(inliers,cloud);
    */

    typename pcl::PointCloud<PointT>::Ptr  cloudPlane (new pcl::PointCloud<PointT>());  // The plane points
	typename pcl::PointCloud<PointT>::Ptr cloudObstacle (new pcl::PointCloud<PointT>());  // The points above the plane i.e obstacles

    //iterate through all the points and find if they are in inliersPlane, yes then move to Plane cloud
    for(int index = 0; index < cloud->points.size(); index++)
    {
        PointT point = cloud->points[index];

        if(inliersPlane.count(index)) //if the point is in the RANSAC plane inliers
            cloudPlane->points.push_back(point);
        else
            cloudObstacle->points.push_back(point);
    }

    segResult.first = cloudObstacle;
    segResult.second = cloudPlane;

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "SFNDprj plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    return segResult;   

}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    // Creating the KdTree object for the search method of the extraction
    typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    tree->setInputCloud (cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance (clusterTolerance);
    ec.setMinClusterSize (minSize);
    ec.setMaxClusterSize (maxSize);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud);
    ec.extract (cluster_indices);

    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {

        typename pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>);

        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
            cloud_cluster->push_back ((*cloud)[*pit]); //*
        cloud_cluster->width = cloud_cluster->points.size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        clusters.push_back(cloud_cluster);

        std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
  }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SFNDprj_Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles

    //1) do the clustering based on the input parameters

    //2) return the clusters

    KdTree* tree = new KdTree;
    std::vector<std::vector<float>> points;

    for(int index = 0; index < cloud->points.size(); index++)
    {
        //PointT point = cloud->points[index];
        std::vector<float> point({cloud->points[index].x, cloud->points[index].y, cloud->points[index].z});
        points.push_back(point);
        tree->insert(point,index);

        //tree->insert(cloud->points[index], index);
    }

    std::vector<std::vector<int>> clusters_indexes = euclideanCluster(points, tree, clusterTolerance);

    for (std::vector<int> indexes: clusters_indexes)
    {
        typename pcl::PointCloud<PointT>::Ptr single_cluster (new pcl::PointCloud<PointT>);

        for(int index: indexes)
        {
            single_cluster->points.push_back(cloud->points[index]);
        }
        single_cluster->width = single_cluster->points.size ();
        single_cluster->height = 1;
        single_cluster->is_dense = true;

        if((single_cluster->width >= minSize) && (single_cluster->width <= maxSize))
        {
            clusters.push_back(single_cluster);
        }
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "SFNDprj clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}


template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}