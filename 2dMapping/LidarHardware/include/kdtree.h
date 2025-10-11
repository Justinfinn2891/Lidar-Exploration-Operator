#ifndef KDTREE_H
#define KDTREE_H

#include <vector>
#include <Eigen/Dense>
#include <algorithm>
#include <limits>

// Make sure PointCloud is defined:
using PointCloud = std::vector<Eigen::Vector3f>;

struct KDNode {
    Eigen::Vector3f point;
    int index;
    KDNode* left;
    KDNode* right;

    KDNode(const Eigen::Vector3f& pt, int idx) : point(pt), index(idx), left(nullptr), right(nullptr) {}
};

class KDTree {
public:
    KDTree(const PointCloud& points) {
        std::vector<int> indices(points.size());
        for (int i = 0; i < (int)points.size(); ++i) indices[i] = i;
        root = build(points, indices, 0);
    }

    ~KDTree() { freeNode(root); }

    int nearest(const Eigen::Vector3f& q) {
        best_dist = std::numeric_limits<float>::max();
        best_index = -1;
        nearestSearch(root, q, 0);
        return best_index;
    }

private:
    KDNode* root;
    float best_dist;
    int best_index;

    KDNode* build(const PointCloud& points, std::vector<int>& indices, int depth) {
        if (indices.empty()) return nullptr;

        int axis = depth % 3;
        int mid = indices.size() / 2;

        auto comp = [axis, &points](int lhs, int rhs) {
            return points[lhs][axis] < points[rhs][axis];
        };
        std::nth_element(indices.begin(), indices.begin() + mid, indices.end(), comp);

        KDNode* node = new KDNode(points[indices[mid]], indices[mid]);

        std::vector<int> left_indices(indices.begin(), indices.begin() + mid);
        std::vector<int> right_indices(indices.begin() + mid + 1, indices.end());

        node->left = build(points, left_indices, depth + 1);
        node->right = build(points, right_indices, depth + 1);

        return node;
    }

    void freeNode(KDNode* node) {
        if (!node) return;
        freeNode(node->left);
        freeNode(node->right);
        delete node;
    }

    void nearestSearch(KDNode* node, const Eigen::Vector3f& q, int depth) {
        if (!node) return;

        float dist = (node->point - q).squaredNorm();
        if (dist < best_dist) {
            best_dist = dist;
            best_index = node->index;
        }

        int axis = depth % 3;
        float diff = q[axis] - node->point[axis];

        KDNode* first = diff < 0 ? node->left : node->right;
        KDNode* second = diff < 0 ? node->right : node->left;

        nearestSearch(first, q, depth + 1);
        if (diff * diff < best_dist) {
            nearestSearch(second, q, depth + 1);
        }
    }
};

#endif // KDTREE_H
