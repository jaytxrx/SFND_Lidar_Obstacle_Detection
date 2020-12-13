#ifndef CLUSTER_H_
#define CLUSTER_H_

#include <vector>
#include <algorithm> //had to include this to remove the std::find error
#include "kdtree3d.h"

bool IsElemInVector(std::vector<float> point ,std::vector<std::vector<float>> points);
void ProximityScanner(std::vector<int> &cluster, int index, const std::vector<std::vector<float>> points,std::vector<bool>  &processed_points, KdTree* tree, float distanceTol);
std::vector<std::vector<int>> euclideanCluster(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol);

#endif /* CLUSTER_H_ */