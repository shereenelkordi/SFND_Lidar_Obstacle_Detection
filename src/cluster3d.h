#include <chrono>
#include <string>
#include "kd3dtree.h"

template<typename PointT>
class Cluster3dExtension {
private:
  int size;
  std::vector<bool> processed;
  std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

public:
    Cluster3dExtension(int points_size) :
      size(points_size) {
        processed.assign(size, false);
      }
    ~Cluster3dExtension();

  void proximity(int i, std::vector<int>& cluster, const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol);
  std::vector<typename pcl::PointCloud<PointT>::Ptr> euclideanCluster(typename pcl::PointCloud<PointT>::Ptr cloud, float distanceTol, int minSize, int maxSize);
};


template<typename PointT>
Cluster3dExtension<PointT>::~Cluster3dExtension() {}

template<typename PointT>
void Cluster3dExtension<PointT>::proximity(int i, std::vector<int>& cluster, const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol){
	processed[i] = true;
	cluster.push_back(i);

	std::vector<int> nearby_ids = tree->search(points[i], distanceTol);
	for(int id : nearby_ids){
		if(!processed[id]){
			proximity(id, cluster, points, tree, distanceTol);
		}
	}
}
template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> Cluster3dExtension<PointT>::euclideanCluster(typename pcl::PointCloud<PointT>::Ptr cloud, float distanceTol, int minSize, int maxSize)
{
    KdTree* tree = new KdTree;
  
    std::vector<std::vector<float>> points;
    for(int i = 0; i<cloud->points.size(); i++){
        std::vector<float> pointVec {cloud->points[i].x, cloud->points[i].y, cloud->points[i].z}; 
    	  tree->insert(pointVec,i);
        points.push_back(pointVec);
    }        
    

	int i = 0;
	while(i<points.size()){
		if(!processed[i]){
			processed[i] = true;
			std::vector<int> cluster_indices;
      typename pcl::PointCloud<PointT>::Ptr cloudCluster(new pcl::PointCloud<PointT>);
			proximity(i, cluster_indices, points, tree, distanceTol);
      if(cluster_indices.size() >= minSize && cluster_indices.size()<= maxSize){
          for (int j=0; j<cluster_indices.size(); j++) {
              cloudCluster->points.push_back(cloud->points[cluster_indices[j]]);
          }

          cloudCluster->width = cloudCluster->points.size();
          cloudCluster->is_dense = true;
          cloudCluster->height = 1;

          clusters.push_back(cloudCluster);
      }
		}
		i++;
	}
	
	return clusters;

}