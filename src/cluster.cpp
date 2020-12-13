#include "cluster.h"

/*copied from cluster.cpp*/
std::vector<std::vector<int>> euclideanCluster(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol)
{

	// TODO: Fill out this function to return list of indices for each cluster

	std::vector<std::vector<int>> clusters; //this stores the point id

	std::vector<bool> processed_points(points.size(), false); //fill all the locations(point size) with FALSE

    //iterate through each of the points
    for(int i=0; i< points.size(); i++)
    {
        if(processed_points[i] == false)
        {
            std::vector<int> cluster; //create a new cluster 
            ProximityScanner(cluster, i, points,processed_points, tree, distanceTol);
            clusters.push_back(cluster);
        }
    }

	return clusters;

}

/*copied from cluster.cpp*/
void ProximityScanner(std::vector<int> &cluster, int index, const std::vector<std::vector<float>> points,std::vector<bool>  &processed_points, KdTree* tree, float distanceTol)
{
    processed_points[index] = true; //mark the current processed point as processed
    cluster.push_back(index);
    
	std::vector<int> nearby_ids = tree->search(points[index], distanceTol);

	for(int id: nearby_ids)
	{
		if(processed_points[id] == false)
		{
			ProximityScanner(cluster, id, points,processed_points, tree, distanceTol);
		}
	}
}