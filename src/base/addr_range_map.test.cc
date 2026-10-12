/*
 * Copyright (c) 2012, 2018 ARM Limited
 * All rights reserved
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
 *
 * Copyright (c) 2006 The Regents of The University of Michigan
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

#include <gtest/gtest.h>

#include <vector>

#include "base/addr_range_map.hh"

using namespace gem5;

// Converted from legacy unit test framework
TEST(AddrRangeMapTest, LegacyTests)
{
    AddrRangeMap<int> r;
    AddrRangeMap<int>::const_iterator i;

    i = r.insert(RangeIn(10, 40), 5);
    ASSERT_NE(i, r.end());

    i = r.insert(RangeIn(60, 90), 3);
    ASSERT_NE(i, r.end());

    EXPECT_NE(r.intersects(RangeIn(20, 30)), r.end());
    EXPECT_EQ(r.contains(RangeIn(55, 55)), r.end());
    EXPECT_EQ(r.intersects(RangeIn(55, 55)), r.end());

    i = r.insert(RangeIn(0, 12), 1);
    EXPECT_EQ(i, r.end());

    i = r.insert(RangeIn(0, 9), 1);
    ASSERT_NE(i, r.end());

    EXPECT_NE(r.contains(RangeIn(20, 30)), r.end());
}

/**
 * Test AddrRangeMap with interleaved address ranges defined by bitmasks.
 * An AddrRangeMap containing a set of N interleaved address ranges,
 * defined with the same start and end address, and including all possible
 * intlvMatch values 0..N-1, must contain the start address.
 * For N-way interleaving, log2(N) selection masks are needed.
 * For N = 16, define the masks as follows,
 *
 * masks[0] = 1 << 6
 * masks[1] = 1 << 7
 * masks[2] = 1 << 8
 * masks[3] = 1 << 9
 *
 */
TEST(AddrRangeMapTest, InterleavedTest1)
{
    const auto N = 16;
    const auto masks = std::vector<Addr>{
        0x40,
        0x80,
        0x100,
        0x200
    };
    const Addr start = 0x80000000;
    const Addr end   = 0xc0000000;

    AddrRangeMap<int> r;
    AddrRangeMap<int>::const_iterator i;

    // populate AddrRangeMap with N-way interleaved address ranges
    // for all intlvMatch values 0..N-1
    for (int k=0; k < N; k++) {
        r.insert(AddrRange(start, end, masks, k), k);
    }
    // find AddrRange element containing start address
    i = r.contains(start);
    // i must not be the past-the-end iterator
    ASSERT_NE(i, r.end()) << "start address not found in AddrRangeMap";
    // intlvMatch = 0 for start = 0x80000000
    EXPECT_EQ(i->second, 0);
}

/**
 * Test AddrRangeMap with interleaved address ranges defined by bitmasks.
 * An AddrRangeMap containing a set of N interleaved address ranges,
 * defined with the same start and end address, and including all possible
 * intlvMatch values 0..N-1, must contain the start address.
 * For N-way interleaving, log2(N) selection masks are needed.
 * For N = 16, define the masks as described in the
 * CMN-600 Technical Reference Manual [1], section 2.17.3
 *
 * masks[0] = 1 << 6 | 1 << 10 | 1 << 14 | .. | 1 << 50
 * masks[1] = 1 << 7 | 1 << 11 | 1 << 15 | .. | 1 << 51
 * masks[2] = 1 << 8 | 1 << 12 | 1 << 16 | .. | 1 << 48
 * masks[3] = 1 << 9 | 1 << 13 | 1 << 17 | .. | 1 << 49
 *
 * [1] https://developer.arm.com/documentation/100180/0302
 */
TEST(AddrRangeMapTest, InterleavedTest2)
{
    const auto N = 16;
    const auto masks = std::vector<Addr>{
        0x4444444444440,
        0x8888888888880,
        0x1111111111100,
        0x2222222222200
    };
    const Addr start = 0x80000000;
    const Addr end   = 0xc0000000;

    AddrRangeMap<int> r;
    AddrRangeMap<int>::const_iterator i;

    // populate AddrRangeMap with N-way interleaved address ranges
    // for all intlvMatch values 0..N-1
    for (int k=0; k < N; k++) {
        r.insert(AddrRange(start, end, masks, k), k);
    }
    // find AddrRange element containing start address
    i = r.contains(start);
    // i must not be the past-the-end iterator
    ASSERT_NE(i, r.end()) << "start address not found in AddrRangeMap";
    // intlvMatch = 2 for start = 0x80000000
    EXPECT_EQ(i->second, 2);
}

/**
 * Test contains() with ranges that wrap around, i.e. whose end is 2^64
 * (stored as 0): both as entries of the map and as the input range.
 */
TEST(AddrRangeMapTest, WrapAround)
{
    const Addr top = 0xffff000000000000;

    AddrRangeMap<int> r;
    AddrRangeMap<int>::const_iterator i;

    ASSERT_NE(r.insert(RangeIn(0, 0xfff), 1), r.end());
    ASSERT_NE(r.insert(AddrRange(top, 0), 2), r.end());

    // A normal input range in a wrapping entry
    i = r.contains(RangeIn(top + 0x1000, top + 0x1fff));
    ASSERT_NE(i, r.end());
    EXPECT_EQ(i->second, 2);

    // A wrapping input range in a wrapping entry
    i = r.contains(AddrRange(top + 0x1000, 0));
    ASSERT_NE(i, r.end());
    EXPECT_EQ(i->second, 2);
    i = r.contains(MaxAddr);
    ASSERT_NE(i, r.end());
    EXPECT_EQ(i->second, 2);

    // Starts below the wrapping entry
    EXPECT_EQ(r.contains(AddrRange(top - 1, 0)), r.end());
    EXPECT_EQ(r.contains(RangeIn(top - 1, top)), r.end());

    // A normal entry still works next to a wrapping one
    i = r.contains(RangeIn(0x800, 0xfff));
    ASSERT_NE(i, r.end());
    EXPECT_EQ(i->second, 1);
    EXPECT_EQ(r.contains(RangeIn(0x800, 0x1000)), r.end());
    EXPECT_EQ(r.contains(AddrRange(0x800, 0)), r.end());

    // An entry that ends at MaxAddr does not wrap, so it does not contain
    // MaxAddr or a wrapping input range.
    AddrRangeMap<int> s;
    ASSERT_NE(s.insert(AddrRange(top, MaxAddr), 3), s.end());
    i = s.contains(MaxAddr - 1);
    ASSERT_NE(i, s.end());
    EXPECT_EQ(i->second, 3);
    EXPECT_EQ(s.contains(MaxAddr), s.end());
    EXPECT_EQ(s.contains(AddrRange(top, 0)), s.end());
}

/**
 * Test contains() with a non-interleaved input range on a map that holds
 * both 4-way interleaved ranges (64-byte stripes) and a plain range. An
 * input range is contained only if it lies within one stripe of the
 * matching entry.
 */
TEST(AddrRangeMapTest, InterleavedContains)
{
    const auto N = 4;
    const auto masks = std::vector<Addr>{0x40, 0x80};
    const Addr start = 0x80000000;
    const Addr end = 0xc0000000;

    // Also with a cache, so that cached entries are tested first
    AddrRangeMap<int, 2> r;
    AddrRangeMap<int, 2>::const_iterator i;

    for (int k = 0; k < N; k++) {
        ASSERT_NE(r.insert(AddrRange(start, end, masks, k), k), r.end());
    }
    ASSERT_NE(r.insert(RangeIn(0x1000, 0x1fff), N), r.end());

    for (int pass = 0; pass < 2; pass++) {
        // One whole stripe, for each intlvMatch
        for (int k = 0; k < N; k++) {
            i = r.contains(RangeSize(start + k * 0x40, 0x40));
            ASSERT_NE(i, r.end());
            EXPECT_EQ(i->second, k);
        }
        // Part of a stripe and a single byte
        i = r.contains(RangeSize(start + 0x1c8, 0x10));
        ASSERT_NE(i, r.end());
        EXPECT_EQ(i->second, 3);
        i = r.contains(RangeIn(start + 0x47, start + 0x47));
        ASSERT_NE(i, r.end());
        EXPECT_EQ(i->second, 1);

        // Across two stripes, or larger than one
        EXPECT_EQ(r.contains(RangeSize(start + 0x20, 0x40)), r.end());
        EXPECT_EQ(r.contains(RangeSize(start, 0x80)), r.end());
        // Outside the interleaved ranges
        EXPECT_EQ(r.contains(RangeSize(end, 0x40)), r.end());
        EXPECT_EQ(r.contains(RangeSize(start - 0x40, 0x40)), r.end());

        // The plain entry
        i = r.contains(RangeIn(0x1800, 0x18ff));
        ASSERT_NE(i, r.end());
        EXPECT_EQ(i->second, N);
        EXPECT_EQ(r.contains(RangeIn(0x1800, 0x2000)), r.end());
    }
}
