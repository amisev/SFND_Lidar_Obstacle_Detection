//
// Created by Alexey on 04/11/2022.
//
#include "search.h"

using namespace hw::search;

template<typename PointT>
hw::search::Node<PointT>::Node(PointT points, int setId) : point(std::move(points)), id(setId), left(nullptr),
                                                right(nullptr) {};

template<typename PointT>
Node<PointT>::~Node() {
    delete this->left;
    delete this->right;
};

template<typename PointT>
KdTree<PointT>::KdTree() : root(nullptr) {};

template<typename PointT>
KdTree<PointT>::~KdTree() { delete this->root; };

template<typename PointT>
void KdTree<PointT>::insert(PointT point, int id) {
    KdTree::insert(&this->root, 0, std::move(point), id);
}

template<typename PointT>
void KdTree<PointT>::insert(Node<PointT> **node, int level, PointT point, int id) {
    if (*node == nullptr) {
        *node = new Node<PointT>(point, id);
    } else {
        if ((*node)->point[level % point.size()] < point[level % point.size()]) {
            insert(&(*node)->right, level + 1, point, id);
        } else {
            insert(&(*node)->left, level + 1, point, id);
        }
    }
}

template<typename PointT>
std::vector<int> KdTree<PointT>::search(PointT target, float distanceTol) {
    std::vector<int> ids;
    search(this->root, 0, ids, std::move(target), distanceTol);
    return ids;
}

template<typename PointT>
void KdTree<PointT>::search(Node<PointT> *node, size_t level, std::vector<int> &ids, PointT target, float distanceTol) {
    if (!node) return;
    if (get_dist(target, node->point) <= distanceTol) {
        ids.push_back(node->id);
    }
    if (std::abs(node->point[level % target.size()] - target[level % target.size()]) < distanceTol) {
        search(node->right, level + 1, ids, target, distanceTol);
        search(node->left, level + 1, ids, target, distanceTol);
    } else if (target[level % target.size()] <= node->point[level % target.size()]) {
        search(node->left, level + 1, ids, target, distanceTol);
    } else {
        search(node->right, level + 1, ids, target, distanceTol);
    }
}

template<typename PointT>
float KdTree<PointT>::get_dist(PointT &point1, PointT &point2) {
    float res = 0.0;
    for (int dim = 0; dim < point1.size(); dim++) {
        res += (point1[dim] - point2[dim]) * (point1[dim] - point2[dim]);
    }
    return std::sqrt(res);
};
