//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/9/15.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOLIO_DISJOINT_SET_UNION_H
#define GEOLIO_DISJOINT_SET_UNION_H
#include <vector>
#include <numeric>
#include <geogram/basic/numeric.h>

namespace geolio
{
    /**
     * @brief Disjoint-set union data structure for maintaining connected components.
     */
    class DisjointSetUnion {
    public:
        /**
         * @brief Constructs a disjoint-set union with n elements.
         * @param[in] n The total number of elements.
         */
        explicit DisjointSetUnion(const GEO::index_t n) {
            parent_.resize(n);
            size_.assign(n, 1);
            component_count_ = n;

            std::iota(parent_.begin(), parent_.end(), 0);
        }

        /**
         * @brief Finds the representative of the set containing element p.
         * @param[in] p The index of the element to query.
         * @return The representative index of the set containing p.
         */
        GEO::index_t find(const GEO::index_t p) {
            if (p == parent_[p]) {
                return p;
            }
            return parent_[p] = find(parent_[p]);
        }

        /**
         * @brief Unites the sets containing p and q.
         * @param[in] p The first element index.
         * @param[in] q The second element index.
         * @return true if the two elements were in different sets and were merged;
         *         false if they were already connected.
         */
        bool unite(const GEO::index_t p, const GEO::index_t q) {
            GEO::index_t root_p = find(p);
            GEO::index_t root_q = find(q);

            if (root_p == root_q)
                return false;

            if (size_[root_p] < size_[root_q])
                std::swap(root_p, root_q);

            parent_[root_q] = root_p;
            size_[root_p] += size_[root_q];

            --component_count_;

            return true;
        }

        /**
         * @brief Checks whether p and q belong to the same set.
         * @param[in] p The first element index.
         * @param[in] q The second element index.
         * @return true if p and q are connected; false otherwise.
         */
        [[nodiscard]] bool connected(const GEO::index_t p, const GEO::index_t q) {
            return find(p) == find(q);
        }

        /**
         * @brief Gets the size of the set containing element p.
         * @param[in] p The element index.
         * @return The cardinality of the set containing p.
         */
        [[nodiscard]] GEO::index_t get_size(const GEO::index_t p) {
            return size_[find(p)];
        }

        /**
         * @brief Gets the number of connected components.
         * @return The total number of connected components.
         */
        [[nodiscard]] GEO::index_t get_component_count() const {
            return component_count_;
        }

    private:
        std::vector<GEO::index_t> parent_;
        std::vector<GEO::index_t> size_;
        GEO::index_t component_count_;
    };
} // namespace geolio
#endif // GEOLIO_DISJOINT_SET_UNION_H
