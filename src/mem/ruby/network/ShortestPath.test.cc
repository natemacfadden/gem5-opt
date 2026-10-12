/*
 * Copyright (c) 2026 The gem5-opt contributors (see the git history).
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <gtest/gtest.h>

#include <random>

#include "mem/ruby/network/ShortestPath.hh"

using namespace gem5::ruby;

namespace
{

// Topology's weight for "no link"
const int NoLink = 10000;

DistMatrix
noLinks(int n, int no_link = NoLink)
{
    DistMatrix d(n, std::vector<int>(n, no_link));
    for (int i = 0; i < n; i++) {
        d[i][i] = 0;
    }
    return d;
}

// Both methods must give the same matrix. Returns it.
DistMatrix
checkBoth(const DistMatrix &weights, int no_link = NoLink)
{
    DistMatrix ref = weights;
    relaxAllPairs(ref);
    DistMatrix fast = weights;
    EXPECT_TRUE(dijkstraAllPairs(fast, no_link));
    EXPECT_EQ(fast, ref);
    return fast;
}

} // anonymous namespace

TEST(ShortestPathTest, Empty)
{
    DistMatrix d;
    EXPECT_TRUE(dijkstraAllPairs(d, NoLink));
    EXPECT_TRUE(d.empty());
    checkBoth(noLinks(1));
}

TEST(ShortestPathTest, Chain)
{
    // 0 -> 1 -> 2 -> 3, plus a slower direct link 0 -> 3
    DistMatrix w = noLinks(4);
    w[0][1] = 1;
    w[1][2] = 2;
    w[2][3] = 3;
    w[0][3] = 7;
    DistMatrix d = checkBoth(w);
    EXPECT_EQ(d[0][1], 1);
    EXPECT_EQ(d[0][2], 3);
    EXPECT_EQ(d[0][3], 6);
    EXPECT_EQ(d[1][3], 5);
    EXPECT_EQ(d[3][0], NoLink);
    EXPECT_EQ(d[2][1], NoLink);
}

TEST(ShortestPathTest, ZeroWeightLinks)
{
    DistMatrix w = noLinks(3);
    w[0][1] = 0;
    w[1][0] = 0;
    w[1][2] = 4;
    DistMatrix d = checkBoth(w);
    EXPECT_EQ(d[0][2], 4);
    EXPECT_EQ(d[1][0], 0);
}

TEST(ShortestPathTest, LongPathsCappedAtNoLink)
{
    // Each hop costs 6, so two hops reach no_link and three exceed it.
    const int no_link = 12;
    DistMatrix w = noLinks(4, no_link);
    for (int i = 0; i < 3; i++) {
        w[i][i + 1] = 6;
    }
    DistMatrix d = checkBoth(w, no_link);
    EXPECT_EQ(d[0][1], 6);
    EXPECT_EQ(d[0][2], no_link);
    EXPECT_EQ(d[0][3], no_link);
}

/**
 * A 4x4 mesh with the weights of Mesh_XY (1 on X links, 2 on Y links),
 * with one endpoint in and out of every router, laid out as Topology
 * does: input endpoints, then output endpoints, then routers.
 */
TEST(ShortestPathTest, MeshXY)
{
    const int rows = 4;
    const int cols = 4;
    const int routers = rows * cols;
    const int nodes = routers;
    DistMatrix w = noLinks(2 * nodes + routers);
    auto router = [&](int r, int c) { return 2 * nodes + r * cols + c; };
    for (int r = 0; r < rows; r++) {
        for (int c = 0; c < cols; c++) {
            int n = r * cols + c;
            w[n][router(r, c)] = 1;
            w[router(r, c)][nodes + n] = 1;
            if (c + 1 < cols) {
                w[router(r, c)][router(r, c + 1)] = 1;
                w[router(r, c + 1)][router(r, c)] = 1;
            }
            if (r + 1 < rows) {
                w[router(r, c)][router(r + 1, c)] = 2;
                w[router(r + 1, c)][router(r, c)] = 2;
            }
        }
    }
    DistMatrix d = checkBoth(w);
    // From the input of node (0, 0) to the output of node (3, 3)
    EXPECT_EQ(d[0][nodes + routers - 1], 1 + 3 * 1 + 3 * 2 + 1);
    // Inputs are not reachable from anywhere
    EXPECT_EQ(d[router(1, 1)][0], NoLink);
}

/**
 * Random sparse and dense graphs, with zero weights, ties and links of
 * weight no_link, compared against the relaxation.
 */
TEST(ShortestPathTest, RandomGraphs)
{
    std::mt19937 rng(1);
    for (int no_link : {NoLink, 20}) {
        for (int iter = 0; iter < 200; iter++) {
            int n = 1 + rng() % 40;
            double density = (rng() % 100) / 100.0;
            int max_weight = (iter % 2) ? 3 : no_link;
            DistMatrix w = noLinks(n, no_link);
            for (int i = 0; i < n; i++) {
                for (int j = 0; j < n; j++) {
                    if (i != j && (rng() % 1000) < density * 1000) {
                        w[i][j] = rng() % (max_weight + 1);
                    }
                }
            }
            checkBoth(w, no_link);
        }
    }
}

/**
 * Outside its domain dijkstraAllPairs refuses, leaving the matrix
 * untouched, and the caller falls back to relaxAllPairs.
 */
TEST(ShortestPathTest, Refuses)
{
    DistMatrix w = noLinks(3);
    w[0][1] = 1;

    DistMatrix neg = w;
    neg[1][2] = -1;
    DistMatrix big = w;
    big[1][2] = NoLink + 1;
    DistMatrix self = w;
    self[2][2] = 1;
    DistMatrix ragged = w;
    ragged[1].push_back(1);

    for (const DistMatrix &m : {neg, big, self, ragged}) {
        DistMatrix d = m;
        EXPECT_FALSE(dijkstraAllPairs(d, NoLink));
        EXPECT_EQ(d, m);
    }
}
