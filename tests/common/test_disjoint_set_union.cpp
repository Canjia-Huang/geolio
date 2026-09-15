//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/9/15.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#include <gtest/gtest.h>
#include <geolio/common/disjoint_set_union.h>

namespace geolio::test
{
    // Test 1: Verify the initial state of the DSU
    TEST(DSUTest, Initialization) {
        constexpr GEO::index_t n = 5;
        DisjointSetUnion dsu(n);

        // Initially, there should be 'n' disconnected components
        EXPECT_EQ(dsu.get_component_count(), n);

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

    // Test 2: Verify basic union operations
    TEST(DSUTest, BasicUnion) {
        DisjointSetUnion dsu(5);

        // Unite 0 and 1 successfully
        EXPECT_TRUE(dsu.unite(0, 1));

        // They should now be connected
        EXPECT_TRUE(dsu.connected(0, 1));

        // The size of their component should be 2
        EXPECT_EQ(dsu.get_size(0), 2);
        EXPECT_EQ(dsu.get_size(1), 2);

        // Component count should decrease by 1
        EXPECT_EQ(dsu.get_component_count(), 4);

        // Element 2 should still be isolated
        EXPECT_FALSE(dsu.connected(0, 2));
        EXPECT_EQ(dsu.get_size(2), 1);
    }

    // Test 3: Verify redundant unions (uniting already connected elements)
    TEST(DSUTest, RedundantUnion) {
        DisjointSetUnion dsu(5);

        dsu.unite(0, 1);

        // Attempting to unite 0 and 1 again should return false
        EXPECT_FALSE(dsu.unite(0, 1));

        // Attempting to unite through an already connected path (1 and 0)
        EXPECT_FALSE(dsu.unite(1, 0));

        // Component count and size should remain unchanged
        EXPECT_EQ(dsu.get_component_count(), 4);
        EXPECT_EQ(dsu.get_size(0), 2);
    }

    // Test 4: Verify transitivity (If A~B and B~C, then A~C)
    TEST(DSUTest, Transitivity) {
        DisjointSetUnion dsu(5);

        dsu.unite(0, 1);
        dsu.unite(1, 2);

        // 0 and 2 should be connected implicitly
        EXPECT_TRUE(dsu.connected(0, 2));

        // Size of the component containing 0, 1, and 2 should be 3
        EXPECT_EQ(dsu.get_size(0), 3);
        EXPECT_EQ(dsu.get_size(1), 3);
        EXPECT_EQ(dsu.get_size(2), 3);

        // Component count should be exactly 3 (Set{0,1,2}, Set{3}, Set{4})
        EXPECT_EQ(dsu.get_component_count(), 3);
    }

    // Test 5: Verify merging multiple disjoGEO::index_t components
    TEST(DSUTest, MergeMultipleComponents) {
        DisjointSetUnion dsu(6);

        // Create three separate pairs
        dsu.unite(0, 1);
        dsu.unite(2, 3);
        dsu.unite(4, 5);

        EXPECT_EQ(dsu.get_component_count(), 3);
        EXPECT_EQ(dsu.get_size(0), 2);
        EXPECT_EQ(dsu.get_size(2), 2);

        // Merge the first two pairs
        dsu.unite(1, 3);

        // Now {0, 1, 2, 3} are connected, {4, 5} is isolated from them
        EXPECT_TRUE(dsu.connected(0, 2));
        EXPECT_TRUE(dsu.connected(1, 2));
        EXPECT_FALSE(dsu.connected(0, 4));

        // Check final states
        EXPECT_EQ(dsu.get_component_count(), 2);
        EXPECT_EQ(dsu.get_size(0), 4);
        EXPECT_EQ(dsu.get_size(5), 2);
    }
}