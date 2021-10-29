/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "render/render.h"


// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}

	~Node()
	{
		delete left;
		delete right;
	}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	~KdTree()
	{
		delete root;
	}
	void insertHelper(Node** node, int depth, std::vector<float> point, int id){
		if(*node == NULL){
			*node = new Node(point, id);
		}else{
			int level = depth%3;
			if(level == 0){
				if((*node)->point[0] > point[0]) insertHelper(&(*node)->left,depth+1,point,id);
				else insertHelper(&(*node)->right,depth+1,point,id);
			}else if(level == 1){
				if((*node)->point[1] > point[1]) insertHelper(&(*node)->left,depth+1,point,id);
				else insertHelper(&(*node)->right,depth+1,point,id);
			}else{
                if((*node)->point[2] > point[2]) insertHelper(&(*node)->left,depth+1,point,id);
				else insertHelper(&(*node)->right,depth+1,point,id);
            }
            
		}
	}
	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		insertHelper(&root, 0, point, id);
	}
	void searchHelper(std::vector<float> target, Node* node, int depth, float distanceTol, std::vector<int>& ids){
		if (node != NULL){
			float left_boundary = target[0] - distanceTol;
			float right_boundary = target[0] + distanceTol;
			float upper_boundary = target[1] + distanceTol;
			float lower_boundary = target[1] - distanceTol;
			float x = node-> point[0];
			float y = node-> point[1];
			if((x >= left_boundary && x <= right_boundary)&&(y >= lower_boundary && y <= upper_boundary)){
				float distance = sqrt((x - target[0])*(x - target[0])+(y - target[1])*(y - target[1]));
				if(distance <= distanceTol){
					ids.push_back(node->id);
				}
			}

			int level = depth%2;
			if(level == 0){
				if(left_boundary < x) searchHelper(target, node->left, depth+1, distanceTol, ids);
				if(right_boundary > x) searchHelper(target, node->right, depth+1, distanceTol, ids);
			}else{
				if(lower_boundary < y) searchHelper(target, node->left, depth+1, distanceTol, ids);
				if(upper_boundary > y) searchHelper(target, node->right, depth+1, distanceTol, ids);
			}	
		}
	}
	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		searchHelper(target, root, 0, distanceTol, ids);
		return ids;
	}
	

};
