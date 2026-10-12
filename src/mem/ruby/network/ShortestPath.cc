/*
 * Copyright (c) 2026 The gem5-opt contributors (see the git history).
 * Copyright (c) 2020 Advanced Micro Devices, Inc.
 * Copyright (c) 1999-2008 Mark D. Hill and David A. Wood
 * All rights reserved.
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

#include "mem/ruby/network/ShortestPath.hh"

#include <algorithm>
#include <functional>
#include <limits>
#include <utility>

namespace gem5
{

namespace ruby
{

// The following all-pairs shortest path algorithm is based on the
// discussion from Cormen et al., Chapter 26.1.
void
relaxAllPairs(DistMatrix &d)
{
    int nodes = d.size();
    bool change = true;
    while (change) {
        change = false;
        for (int i = 0; i < nodes; i++) {
            auto &d_i = d[i];
            for (int j = 0; j < nodes; j++) {
                int minimum = d_i[j];
                for (int k = 0; k < nodes; k++) {
                    minimum = std::min(minimum, d_i[k] + d[k][j]);
                }
                if (d_i[j] != minimum) {
                    change = true;
                    d_i[j] = minimum;
                }
            }
        }
    }
}

// With a zero diagonal and every link weight in [0, no_link), any path
// using a no_link entry costs at least no_link, so the fixed point of
// relaxAllPairs is min(shortest path over the real links, no_link).
bool
dijkstraAllPairs(DistMatrix &d, int no_link)
{
    const int n = d.size();
    std::vector<std::vector<std::pair<int, int>>> adj(n);
    for (int i = 0; i < n; i++) {
        if ((int)d[i].size() != n || d[i][i] != 0) {
            return false;
        }
        for (int j = 0; j < n; j++) {
            int w = d[i][j];
            if (j == i || w == no_link) {
                continue;
            }
            if (w < 0 || w > no_link) {
                return false;
            }
            adj[i].emplace_back(j, w);
        }
    }

    typedef std::pair<int, int> QEntry; // (distance, node)
    std::vector<int> dist(n);
    std::vector<QEntry> heap;
    for (int s = 0; s < n; s++) {
        std::fill(dist.begin(), dist.end(), std::numeric_limits<int>::max());
        dist[s] = 0;
        heap.clear();
        heap.emplace_back(0, s);
        while (!heap.empty()) {
            std::pop_heap(heap.begin(), heap.end(), std::greater<QEntry>());
            QEntry top = heap.back();
            heap.pop_back();
            int u = top.second;
            if (top.first > dist[u]) {
                continue;
            }
            for (const auto &e : adj[u]) {
                int nd = top.first + e.second;
                if (nd < dist[e.first]) {
                    dist[e.first] = nd;
                    heap.emplace_back(nd, e.first);
                    std::push_heap(heap.begin(), heap.end(),
                                   std::greater<QEntry>());
                }
            }
        }
        auto &d_s = d[s];
        for (int t = 0; t < n; t++) {
            if (t != s) {
                d_s[t] = std::min(dist[t], no_link);
            }
        }
    }
    return true;
}

} // namespace ruby
} // namespace gem5
