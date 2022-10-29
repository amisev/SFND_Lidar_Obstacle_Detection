/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"


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

	void insert(std::vector<float> point, int id)
	{
        insert(&this->root, 0, point, id);
		// the function should create a new node and place correctly with in the root
	}
    void insert(Node** node, int level, std::vector<float> point, int id) {
        if (*node == nullptr) {
            *node = new Node(point, id);
        } else {
            if ((*node)->point[level % point.size()] < point[level % point.size()]) {
                insert(&(*node)->right, level + 1, point, id);
            } else {
                insert(&(*node)->left, level + 1, point, id);
            }
        }
    }

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
        search(this->root, 0, ids, target, distanceTol);
		return ids;
	}
    void search(Node* node, size_t level, std::vector<int> &ids, std::vector<float> target, float distanceTol) {
        if (!node) return;
        if (get_dist(target, node->point) <= distanceTol) {
            ids.push_back(node->id);
        }
        if (std::abs(node->point[level % target.size()] - target[level % target.size()])) {
            search(node->right, level + 1, ids, target, distanceTol);
            search(node->left, level + 1, ids, target, distanceTol);
        } else if (target[level % target.size()] <= node->point[level % target.size()]) {
            search(node->left, level + 1, ids, target, distanceTol);
        } else {
            search(node->left, level + 1, ids, target, distanceTol);
        }
    }
private:
    float get_dist(std::vector<float>& point1, std::vector<float>& point2) {
        float res = 0.0;
        for (int dim = 0; dim < point1.size(); dim++) {
            res += (point1[dim] - point2[dim])*(point1[dim] - point2[dim]);
        }
        return std::sqrt(res);
    };
};




