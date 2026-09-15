//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/9/15.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#include <gtest/gtest.h>
#include <geolio/common/disjoint_set_union.h>

namespace geolio::test
{
    // Test 1: Verify the initial state of the DisjointSetUnion
    TEST(DSUTest, Initialization) {
        constexpr GEO::index_t n = 5;
        DisjointSetUnion dsu(n);

        // Initially, there should be 'n' disconnected components
        EXPECT_EQ(dsu.component_count(), n);

        for (GEO::index_t i = 0; i < n; ++i) {
            // Each element should be its own root
            EXPECT_EQ(dsu.find(i), i);
            // Each component should have a size of 1
            EXPECT_EQ(dsu.get_size(i), 1);

            // Elements should not be connected to each other
            for (GEO::index_t j = i + 1; j < n; ++j) {
                EXPECT_FALSE(dsu.connected(i, j));
            }
        }
    }

    // Test 2: Verify basic union operations and returned root index
    TEST(DSUTest, BasicUnion) {
        DisjointSetUnion dsu(5);

        // Unite 0 and 1, store the returned root index
        const GEO::index_t root = dsu.unite(0, 1);

        // The returned root should match the result of find() for both elements
        EXPECT_EQ(dsu.find(0), root);
        EXPECT_EQ(dsu.find(1), root);

        // They should now be connected
        EXPECT_TRUE(dsu.connected(0, 1));

        // The size of their component should be 2
        EXPECT_EQ(dsu.get_size(0), 2);
        EXPECT_EQ(dsu.get_size(1), 2);

        // Component count should decrease by 1
        EXPECT_EQ(dsu.component_count(), 4);

        // Element 2 should still be isolated
        EXPECT_FALSE(dsu.connected(0, 2));
        EXPECT_EQ(dsu.get_size(2), 1);
    }

    // Test 3: Verify redundant unions return the same root index
    TEST(DSUTest, RedundantUnion) {
        DisjointSetUnion dsu(5);

        const GEO::index_t original_root = dsu.unite(0, 1);

        // Attempting to unite 0 and 1 again should return the exact same root
        const GEO::index_t new_root = dsu.unite(0, 1);
        EXPECT_EQ(original_root, new_root);

        // Attempting to unite 0 and 1 again should return false
        EXPECT_FALSE(dsu.unite(0, 1));

        // Attempting to unite through an already connected path (1 and 0)
        EXPECT_FALSE(dsu.unite(1, 0));

        // Component count and size should remain unchanged
        EXPECT_EQ(dsu.component_count(), 4);
        EXPECT_EQ(dsu.get_size(0), 2);
    }

    // Test 4: Verify transitivity and consistent root assignment
    TEST(DSUTest, Transitivity) {
        DisjointSetUnion dsu(5);

        const GEO::index_t root01 = dsu.unite(0, 1);
        const GEO::index_t root12 = dsu.unite(1, 2);

        // After transitive union, all elements should share the same root
        EXPECT_EQ(dsu.find(0), root12);
        EXPECT_EQ(dsu.find(2), root12);

        // 0 and 2 should be connected implicitly
        EXPECT_TRUE(dsu.connected(0, 2));

        // Size of the component containing 0, 1, and 2 should be 3
        EXPECT_EQ(dsu.get_size(0), 3);
        EXPECT_EQ(dsu.get_size(1), 3);
        EXPECT_EQ(dsu.get_size(2), 3);

        // Component count should be exactly 3 (Set{0,1,2}, Set{3}, Set{4})
        EXPECT_EQ(dsu.component_count(), 3);
    }

    // Test 5: Verify merging multiple disjoint components with union-by-size
    TEST(DSUTest, MergeMultipleComponents) {
        DisjointSetUnion dsu(6);

        // Create a set of size 3: {0, 1, 2}
        dsu.unite(0, 1);
        const GEO::index_t root_large = dsu.unite(1, 2);

        // Create a set of size 2: {3, 4}
        const GEO::index_t root_small = dsu.unite(3, 4);

        // Merge the small set into the large set
        const GEO::index_t final_root = dsu.unite(2, 4);

        // Because of union-by-size, the larger set's root should become the new root
        EXPECT_EQ(final_root, root_large);

        // Verify that elements from the smaller set now point to the larger set's root
        EXPECT_EQ(dsu.find(3), root_large);
        EXPECT_EQ(dsu.find(4), root_large);

        // Check final states
        EXPECT_TRUE(dsu.connected(0, 4));
        EXPECT_EQ(dsu.component_count(), 2); // Set {0,1,2,3,4} and Set {5}
        EXPECT_EQ(dsu.get_size(final_root), 5);
    }
}