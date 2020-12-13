/* \author Aaron Brown */
// Quiz on implementing kd tree

#ifndef KDTREE3D_H_
#define KDTREE3D_H_

//#include "render/render.h"
#include <iostream>
#include <math.h>


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
};

struct KdTree
{
	Node* root;
	uint depth;

	KdTree()
	: root(NULL)
	{}

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 

		//iterate through the node until we reach the insertion point
		depth=0;
		//traverse_insert(&root, point, id );
		traverse_insert(&root, 0, point, id );
	}


	void traverse_insert(Node** node, uint depth ,std::vector<float> point, int id )
	{
		//std::cout<<"depth " << depth << " point " << point[0]<<" , "<<point[1] << std::endl;
		if(*node == NULL) //empty tree
		{
			//TODO: Wont this memory vanish when the function is left ? If not what is the difference to stack and heap ?
			*node = new Node(point, id);
			//std::cout<<"end node"<<std::endl;
		}
		else
		{
			uint dpm = depth%3; //dpm==0 is X axis, dpm==1 is Y axis and dpm==2 is Z axis

			if(point[dpm] < (*node)->point[dpm]) {
				traverse_insert(&(*node)->left, depth+1,point,id);
			}
				
				
			else{ //greater than or equal to values will be sent to the right node
				traverse_insert(&(*node)->right, depth+1,point,id);
			}
				
		}
	}

/*
	void traverse_insert(Node** node, std::vector<float> point, int id )
	{
		//std::cout<<"depth " << depth << " point " << point[0]<<" , "<<point[1] << std::endl;
		if(*node == NULL) //empty tree
		{
			//TODO: Wont this memory vanish when the function is left ? If not what is the difference to stack and heap ?
			*node = new Node(point, id);
			depth = 0;
			//std::cout<<"end node"<<std::endl;
		}
		else
		{
			if((depth % 2) == 0){ //Even depth - X axis compare - index 0 in 2D array element
				if(point[0] >= (*node)->point[0]) {
					depth++;
					traverse_insert(&(*node)->right,point,id);
				}
				else {
					depth++;
					traverse_insert(&(*node)->left,point,id);
				}
			}
			else { //Odd depthe - X axis compare - index 1 in 2D array element
				if(point[1] >= (*node)->point[1]) {
					depth++;
					traverse_insert(&(*node)->right,point,id);
				}
				else {
					depth++;
					traverse_insert(&(*node)->left,point,id);
				}
			}
		}
	}
*/

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;

		traverse_search(target, root,distanceTol, 0, ids);

		return ids;
	}



/*
This funtion takes the bounding box value and the bounding box. Also it takes the node as input one at a time
This is a recrusive function so it takes one node as input at a time.
*/
	void traverse_search(std::vector<float> target, Node *node,float dt, uint depth, std::vector<int> &ids)
	{
		if(node != NULL)
		{
			//std::cout<< "current node " << node->point[0] << " and " << node->point[1] << std::endl;

			/*if the present node is inside the bounding box of the traget, 
			save the id after checking if its inside the radius
			- this is done to reduce the radius computation load and making the algorithm fast*/
			if(   (node->point[0] <= target[0]+dt) // X
			   && (node->point[0] >= target[0]-dt) // X
			   && (node->point[1] <= target[1]+dt) // Y
			   && (node->point[1] >= target[1]-dt) // Y
			   && (node->point[2] <= target[2]+dt) // Z
			   && (node->point[2] >= target[2]-dt) // Z
			)
			{   //d = ((x2 - x1)2 + (y2 - y1)2 + (z2 - z1)2)1/2 
				auto x1x2 = (node->point[0] - target[0]);
				auto y1y2 = (node->point[1] - target[1]);
				auto z1z2 = (node->point[2] - target[2]);
				auto dist = sqrt((x1x2*x1x2)+(y1y2*y1y2)+(z1z2*z1z2));

				if(dist <= dt)
				{
					ids.push_back(node->id);
					//std::cout<< "node found" << std::endl;
				}
			}

			uint dpm = depth%3; //dpm==0 is X axis, dpm==1 is Y axis and dpm==2 is Z axis

			//the next node to be analysed is decided here
			if((target[dpm]-dt) < node->point[dpm])
			{
				traverse_search(target, node->left,dt, depth+1, ids);
			}

			if((target[dpm]+dt) >= node->point[dpm]) //greater than or equal to, got to right node
			{
				traverse_search(target, node->right,dt, depth+1, ids);
			}
		}
	}
};

#endif /* KDTREE3D_H_ */