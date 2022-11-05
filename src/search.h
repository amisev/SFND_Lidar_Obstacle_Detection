//
// Created by Alexey on 02/11/2022.
//
#ifndef PLAYBACK_SEARCH_H
#define PLAYBACK_SEARCH_H

#include <vector>
#include <iostream>
#include <cmath>

namespace hw {
    namespace search {
        template<typename PointT>
        struct Node {
            PointT point;
            int id;
            Node *left;
            Node *right;

            Node(PointT points, int setId);

            ~Node();
        };

        template<typename PointT>
        struct KdTree {
            Node<PointT> *root;

            KdTree();

            ~KdTree();

            void insert(PointT point, int id);

            // return a list of point ids in the tree that are within distance of target
            std::vector<int> search(PointT target, float distanceTol);


        private:
            void insert(Node<PointT> **node, int level, PointT point, int id);

            void search(Node<PointT> *node, size_t level, std::vector<int> &ids, PointT target, float distanceTol);

            static float get_dist(PointT &point1, PointT &point2);
        };
    }
}
#endif //PLAYBACK_SEARCH_H
