/*
 * Copyright (c) 2012, 2014, 2018 ARM Limited
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

#include "mem/physical.hh"

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/user.h>
#include <unistd.h>
#include <zlib.h>

#include <algorithm>
#include <cerrno>
#include <climits>
#include <cstdio>
#include <cstring>
#include <functional>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "base/intmath.hh"
#include "base/trace.hh"
#include "debug/AddrRanges.hh"
#include "debug/Checkpoint.hh"
#include "mem/abstract_mem.hh"
#include "sim/serialize.hh"
#include "sim/sim_exit.hh"

/**
 * On Linux, MAP_NORESERVE allow us to simulate a very large memory
 * without committing to actually providing the swap space on the
 * host. On FreeBSD or OSX the MAP_NORESERVE flag does not exist,
 * so simply make it 0.
 */
#if defined(__APPLE__) || defined(__FreeBSD__)
#ifndef MAP_NORESERVE
#define MAP_NORESERVE 0
#endif
#endif

namespace gem5
{

namespace memory
{

PhysicalMemory::PhysicalMemory(const std::string& _name,
                               const std::vector<AbstractMemory*>& _memories,
                               bool mmap_using_noreserve,
                               const std::string& shared_backstore,
                               bool auto_unlink_shared_backstore) :
    _name(_name), size(0), mmapUsingNoReserve(mmap_using_noreserve),
    sharedBackstore(shared_backstore), sharedBackstoreSize(0),
    pageSize(sysconf(_SC_PAGE_SIZE))
{
    // Register cleanup callback if requested.
    if (auto_unlink_shared_backstore && !sharedBackstore.empty()) {
        registerExitCallback([=]() { shm_unlink(shared_backstore.c_str()); });
    }

    if (mmap_using_noreserve)
        warn("Not reserving swap space. May cause SIGSEGV on actual usage\n");

    // add the memories from the system to the address map as
    // appropriate
    for (const auto& m : _memories) {
        // only add the memory if it is part of the global address map
        if (m->isInAddrMap()) {
            memories.push_back(m);

            // calculate the total size once and for all
            size += m->size();

            // add the range to our interval tree and make sure it does not
            // intersect an existing range
            fatal_if(addrMap.insert(m->getAddrRange(), m) == addrMap.end(),
                     "Memory address range for %s is overlapping\n",
                     m->name());
        } else {
            // this type of memory is used e.g. as reference memory by
            // Ruby, and they also needs a backing store, but should
            // not be part of the global address map
            DPRINTF(AddrRanges,
                    "Skipping memory %s that is not in global address map\n",
                    m->name());

            // sanity check
            fatal_if(m->getAddrRange().interleaved(),
                     "Memory %s that is not in the global address map cannot "
                     "be interleaved\n", m->name());

            // simply do it independently, also note that this kind of
            // memories are allowed to overlap in the logic address
            // map
            std::vector<AbstractMemory*> unmapped_mems{m};
            createBackingStore(m->getAddrRange(), unmapped_mems,
                               m->isConfReported(), m->isInAddrMap(),
                               m->isKvmMap());
        }
    }

    // iterate over the increasing addresses and chunks of contiguous
    // space to be mapped to backing store, create it and inform the
    // memories
    std::vector<AddrRange> intlv_ranges;
    std::vector<AbstractMemory*> curr_memories;
    for (const auto& r : addrMap) {
        // simply skip past all memories that are null and hence do
        // not need any backing store
        if (!r.second->isNull()) {
            // if the range is interleaved then save it for now
            if (r.first.interleaved()) {
                // if we already got interleaved ranges that are not
                // part of the same range, then first do a merge
                // before we add the new one
                if (!intlv_ranges.empty() &&
                    !intlv_ranges.back().mergesWith(r.first)) {
                    AddrRange merged_range(intlv_ranges);

                    AbstractMemory *f = curr_memories.front();
                    for (const auto& c : curr_memories)
                        if (f->isConfReported() != c->isConfReported() ||
                            f->isInAddrMap() != c->isInAddrMap() ||
                            f->isKvmMap() != c->isKvmMap())
                            fatal("Inconsistent flags in an interleaved "
                                  "range\n");

                    createBackingStore(merged_range, curr_memories,
                                       f->isConfReported(), f->isInAddrMap(),
                                       f->isKvmMap());

                    intlv_ranges.clear();
                    curr_memories.clear();
                }
                intlv_ranges.push_back(r.first);
                curr_memories.push_back(r.second);
            } else {
                std::vector<AbstractMemory*> single_memory{r.second};
                createBackingStore(r.first, single_memory,
                                   r.second->isConfReported(),
                                   r.second->isInAddrMap(),
                                   r.second->isKvmMap());
            }
        }
    }

    // if there is still interleaved ranges waiting to be merged, go
    // ahead and do it
    if (!intlv_ranges.empty()) {
        AddrRange merged_range(intlv_ranges);

        AbstractMemory *f = curr_memories.front();
        for (const auto& c : curr_memories)
            if (f->isConfReported() != c->isConfReported() ||
                f->isInAddrMap() != c->isInAddrMap() ||
                f->isKvmMap() != c->isKvmMap())
                fatal("Inconsistent flags in an interleaved "
                      "range\n");

        createBackingStore(merged_range, curr_memories,
                           f->isConfReported(), f->isInAddrMap(),
                           f->isKvmMap());
    }
}

void
PhysicalMemory::createBackingStore(
        AddrRange range, const std::vector<AbstractMemory*>& _memories,
        bool conf_table_reported, bool in_addr_map, bool kvm_map)
{
    panic_if(range.interleaved(),
             "Cannot create backing store for interleaved range %s\n",
              range.to_string());

    // perform the actual mmap
    DPRINTF(AddrRanges, "Creating backing store for range %s with size %d\n",
            range.to_string(), range.size());

    int shm_fd;
    int map_flags;
    off_t map_offset;

    if (sharedBackstore.empty()) {
        shm_fd = -1;
        map_flags =  MAP_ANON | MAP_PRIVATE;
        map_offset = 0;
    } else {
        // Newly create backstore will be located after previous one.
        map_offset = sharedBackstoreSize;
        // mmap requires the offset to be multiple of page, so we need to
        // upscale the range size.
        sharedBackstoreSize += roundUp(range.size(), pageSize);
        DPRINTF(AddrRanges, "Sharing backing store as %s at offset %llu\n",
                sharedBackstore.c_str(), (uint64_t)map_offset);
        shm_fd = shm_open(sharedBackstore.c_str(), O_CREAT | O_RDWR, 0666);
        if (shm_fd == -1)
               panic("Shared memory failed");
        if (ftruncate(shm_fd, sharedBackstoreSize))
               panic("Setting size of shared memory failed");
        map_flags = MAP_SHARED;
    }

    // to be able to simulate very large memories, the user can opt to
    // pass noreserve to mmap
    if (mmapUsingNoReserve) {
        map_flags |= MAP_NORESERVE;
    }

    uint8_t* pmem = (uint8_t*) mmap(NULL, range.size(),
                                    PROT_READ | PROT_WRITE,
                                    map_flags, shm_fd, map_offset);

    if (pmem == (uint8_t*) MAP_FAILED) {
        perror("mmap");
        fatal("Could not mmap %d bytes for range %s!\n", range.size(),
              range.to_string());
    }

    // remember this backing store so we can checkpoint it and unmap
    // it appropriately
    backingStore.emplace_back(range, pmem,
                              conf_table_reported, in_addr_map, kvm_map,
                              shm_fd, map_offset);

    // point the memories to their backing store
    for (const auto& m : _memories) {
        DPRINTF(AddrRanges, "Mapping memory %s to backing store\n",
                m->name());
        m->setBackingStore(pmem);
    }
}

PhysicalMemory::~PhysicalMemory()
{
    // unmap the backing store
    for (auto& s : backingStore)
        munmap((char*)s.pmem, s.range.size());
}

bool
PhysicalMemory::isMemAddr(Addr addr) const
{
    return addrMap.contains(addr) != addrMap.end();
}

AddrRangeList
PhysicalMemory::getConfAddrRanges() const
{
    // this could be done once in the constructor, but since it is unlikely to
    // be called more than once the iteration should not be a problem
    AddrRangeList ranges;
    std::vector<AddrRange> intlv_ranges;
    for (const auto& r : addrMap) {
        if (r.second->isConfReported()) {
            // if the range is interleaved then save it for now
            if (r.first.interleaved()) {
                // if we already got interleaved ranges that are not
                // part of the same range, then first do a merge
                // before we add the new one
                if (!intlv_ranges.empty() &&
                    !intlv_ranges.back().mergesWith(r.first)) {
                    ranges.push_back(AddrRange(intlv_ranges));
                    intlv_ranges.clear();
                }
                intlv_ranges.push_back(r.first);
            } else {
                // keep the current range
                ranges.push_back(r.first);
            }
        }
    }

    // if there is still interleaved ranges waiting to be merged,
    // go ahead and do it
    if (!intlv_ranges.empty()) {
        ranges.push_back(AddrRange(intlv_ranges));
    }

    return ranges;
}

void
PhysicalMemory::access(PacketPtr pkt)
{
    assert(pkt->isRequest());
    const auto& m = addrMap.contains(pkt->getAddrRange());
    assert(m != addrMap.end());
    m->second->access(pkt);
}

void
PhysicalMemory::functionalAccess(PacketPtr pkt)
{
    assert(pkt->isRequest());
    const auto& m = addrMap.contains(pkt->getAddrRange());
    assert(m != addrMap.end());
    m->second->functionalAccess(pkt);
}

void
PhysicalMemory::serialize(CheckpointOut &cp) const
{
    // serialize all the locked addresses and their context ids
    std::vector<Addr> lal_addr;
    std::vector<ContextID> lal_cid;

    for (auto& m : memories) {
        const std::list<LockedAddr>& locked_addrs = m->getLockedAddrList();
        for (const auto& l : locked_addrs) {
            lal_addr.push_back(l.addr);
            lal_cid.push_back(l.contextId);
        }
    }

    SERIALIZE_CONTAINER(lal_addr);
    SERIALIZE_CONTAINER(lal_cid);

    // serialize the backing stores
    unsigned int nbr_of_stores = backingStore.size();
    SERIALIZE_SCALAR(nbr_of_stores);

    unsigned int store_id = 0;
    // store each backing store memory segment in a file
    for (auto& s : backingStore) {
        ScopedCheckpointSection sec(cp, csprintf("store%d", store_id));
        serializeStore(cp, store_id++, s.range, s.pmem);
    }
}

void
PhysicalMemory::serializeStore(CheckpointOut &cp, unsigned int store_id,
                               AddrRange range, uint8_t* pmem) const
{
    // we cannot use the address range for the name as the
    // memories that are not part of the address map can overlap
    std::string filename =
        name() + ".store" + std::to_string(store_id) + ".pmem";
    Addr range_size = range.size();

    DPRINTF(Checkpoint, "Serializing physical memory %s with size %d\n",
            filename, range_size);

    SERIALIZE_SCALAR(store_id);
    SERIALIZE_SCALAR(filename);
    SERIALIZE_SCALAR(range_size);

    // write memory file
    std::string filepath = CheckpointIn::dir() + "/" + filename.c_str();
    gzFile compressed_mem = gzopen(filepath.c_str(), "wb");
    if (compressed_mem == NULL)
        fatal("Can't open physical memory checkpoint file '%s'\n",
              filename);

    uint64_t pass_size = 0;

    // gzwrite fails if (int)len < 0 (gzwrite returns int)
    for (uint64_t written = 0; written < range.size();
         written += pass_size) {
        pass_size = (uint64_t)INT_MAX < (range.size() - written) ?
            (uint64_t)INT_MAX : (range.size() - written);

        if (gzwrite(compressed_mem, pmem + written,
                    (unsigned int) pass_size) != (int) pass_size) {
            fatal("Write failed on physical memory checkpoint file '%s'\n",
                  filename);
        }
    }

    // close the compressed stream and check that the exit status
    // is zero
    if (gzclose(compressed_mem))
        fatal("Close failed on physical memory checkpoint file '%s'\n",
              filename);

}

namespace
{
/** Where unserializeStore() queues its work while unserialize() runs. */
std::vector<std::function<std::string()>> *pendingRestores = nullptr;
} // anonymous namespace

void
PhysicalMemory::unserialize(CheckpointIn &cp)
{
    // unserializeStore() queues the decompression of each gzip image
    // here instead of doing it; they are independent (separate files,
    // disjoint destination memory), so they run in parallel below.
    std::vector<std::function<std::string()>> jobs;
    pendingRestores = &jobs;

    // unserialize the locked addresses and map them to the
    // appropriate memory controller
    std::vector<Addr> lal_addr;
    std::vector<ContextID> lal_cid;
    UNSERIALIZE_CONTAINER(lal_addr);
    UNSERIALIZE_CONTAINER(lal_cid);
    for (size_t i = 0; i < lal_addr.size(); ++i) {
        const auto& m = addrMap.contains(lal_addr[i]);
        m->second->addLockedAddr(LockedAddr(lal_addr[i], lal_cid[i]));
    }

    // unserialize the backing stores
    unsigned int nbr_of_stores;
    UNSERIALIZE_SCALAR(nbr_of_stores);

    for (unsigned int i = 0; i < nbr_of_stores; ++i) {
        ScopedCheckpointSection sec(cp, csprintf("store%d", i));
        unserializeStore(cp);
    }
    pendingRestores = nullptr;

    std::vector<std::string> errs(jobs.size());
    std::vector<std::thread> threads;
    for (size_t i = 1; i < jobs.size(); i++) {
        threads.emplace_back([&, i] { errs[i] = jobs[i](); });
    }
    if (!jobs.empty()) {
        errs[0] = jobs[0]();
    }
    for (auto &t : threads) {
        t.join();
    }
    for (const auto &e : errs) {
        if (!e.empty()) {
            fatal("%s", e);
        }
    }
}

namespace
{

constexpr size_t restorePageSize = 4096;

/**
 * CRC-32 of runs of zero bytes, used to keep the gzip integrity check
 * without feeding every (mostly zero) restored byte through crc32().
 * Entry k holds the CRC of (4096 << k) zero bytes and the zlib
 * combine operator for that length.
 */
struct ZeroPageCrc
{
    static constexpr int levels = 40;
    uLong crc[levels];
    uLong op[levels];

    ZeroPageCrc()
    {
        static const uint8_t zeros[restorePageSize] = {};
        crc[0] = crc32(0L, zeros, sizeof(zeros));
        for (int k = 0; k < levels; k++) {
            op[k] = crc32_combine_gen64((z_off64_t)restorePageSize << k);
            if (k + 1 < levels) {
                crc[k + 1] = crc32_combine_op(crc[k], crc[k], op[k]);
            }
        }
    }

    /** Extend running crc @p c by @p pages pages of zeros. */
    uLong
    extend(uLong c, uint64_t pages) const
    {
        for (int k = 0; pages; k++, pages >>= 1) {
            if (pages & 1) {
                c = crc32_combine_op(c, crc[k], op[k]);
            }
        }
        return c;
    }
};

bool
pageIsZero(const uint8_t *p)
{
    uint64_t q[8];
    for (size_t i = 0; i < restorePageSize; i += sizeof(q)) {
        std::memcpy(q, p + i, sizeof(q));
        if (q[0] | q[1] | q[2] | q[3] | q[4] | q[5] | q[6] | q[7]) {
            return false;
        }
    }
    return true;
}

/**
 * Receives the decompressed image in order and stores it to the
 * backing store. All-zero pages are not written: if the destination
 * is private anonymous memory they are dropped with MADV_DONTNEED
 * (which reads back as zero, and is a no-op on never-touched memory),
 * otherwise cleared with memset. This keeps the untouched part of a
 * large, mostly empty memory from ever being faulted in. Also keeps
 * the gzip CRC, hashing only the non-zero data.
 */
class RestoreSink
{
  public:
    RestoreSink(uint8_t *dst, uint64_t dst_len, bool anon)
        : dst(dst), dstLen(dst_len), anon(anon)
    {}

    uint64_t
    produced() const
    {
        return out;
    }
    uint64_t
    room() const
    {
        return dstLen - out;
    }
    uLong
    crc() const
    {
        return crcVal;
    }

    void
    reset()
    {
        crcVal = crc32(0L, Z_NULL, 0);
    }

    /** Append @p len bytes (at most room()). */
    void
    put(const uint8_t *p, size_t len)
    {
        while (len) {
            size_t n = std::min<uint64_t>(len, restorePageSize -
                                                   (out % restorePageSize));
            if (n == restorePageSize && pageIsZero(p)) {
                if (!zeroRun) {
                    zeroStart = out;
                }
                zeroRun += n;
                crcVal = zeroCrc.extend(crcVal, 1);
            } else {
                flushZeroRun();
                std::memcpy(dst + out, p, n);
                crcVal = crc32(crcVal, p, n);
            }
            out += n;
            p += n;
            len -= n;
        }
    }

    /**
     * Append @p pages whole pages of zeros (at most room()). Only valid
     * while produced() is page aligned.
     */
    void
    putZeroPages(uint64_t pages)
    {
        if (!pages) {
            return;
        }
        if (!zeroRun) {
            zeroStart = out;
        }
        zeroRun += pages * restorePageSize;
        crcVal = zeroCrc.extend(crcVal, pages);
        out += pages * restorePageSize;
    }

    void
    finish()
    {
        flushZeroRun();
    }

  private:
    void
    flushZeroRun()
    {
        if (!zeroRun) {
            return;
        }
        // zero runs are always page aligned (see put()).
        if (!anon || madvise(dst + zeroStart, zeroRun, MADV_DONTNEED)) {
            std::memset(dst + zeroStart, 0, zeroRun);
        }
        zeroRun = 0;
    }

    static const ZeroPageCrc zeroCrc;

    uint8_t *dst;
    uint64_t dstLen;
    bool anon;
    uint64_t out = 0;
    uLong crcVal = crc32(0L, Z_NULL, 0);
    uint64_t zeroRun = 0;
    uint64_t zeroStart = 0;
};

const ZeroPageCrc RestoreSink::zeroCrc;

uint32_t
le32(const uint8_t *t)
{
    return t[0] | t[1] << 8 | t[2] << 16 | (uint32_t)t[3] << 24;
}

/**
 * Inflate a gzip image with zlib. Gives exactly what gzread() into dst
 * would produce (including concatenated members and truncation), and
 * fails on corrupt data. Returns bytes produced, or -1 with @p err set.
 */
int64_t
zlibInflateImage(const uint8_t *in, size_t in_len, RestoreSink &sink,
                 std::string &err)
{
    constexpr size_t buf_size = 256 * 1024;
    std::unique_ptr<uint8_t[]> buf(new uint8_t[buf_size]);

    z_stream strm = {};
    if (inflateInit2(&strm, 16 + MAX_WBITS) != Z_OK) {
        err = "inflateInit2 failed";
        return -1;
    }
    // We check the gzip trailer ourselves (see below).
    inflateValidate(&strm, 0);

    strm.next_in = const_cast<Bytef *>(in);
    strm.avail_in = 0;
    size_t in_left = in_len;
    uint64_t member_out = 0;

    bool done = false;
    while (!done && sink.room()) {
        if (strm.avail_in == 0) {
            size_t n = std::min<size_t>(in_left, 1u << 30);
            strm.avail_in = n;
            in_left -= n;
        }
        size_t want = std::min<uint64_t>(buf_size, sink.room());
        strm.next_out = buf.get();
        strm.avail_out = want;
        int ret = inflate(&strm, Z_NO_FLUSH);
        size_t got = want - strm.avail_out;
        sink.put(buf.get(), got);
        member_out += got;

        if (ret == Z_STREAM_END) {
            // gzip trailer (CRC-32, ISIZE) is just before next_in.
            const uint8_t *t = strm.next_in - 8;
            if (le32(t) != (uint32_t)sink.crc() ||
                le32(t + 4) != (uint32_t)member_out) {
                err = "gzip CRC/length check failed";
                inflateEnd(&strm);
                return -1;
            }
            // Like gzread(), continue with a concatenated gzip member,
            // and ignore anything else that follows.
            size_t rest = strm.avail_in + in_left;
            if (rest < 2 || strm.next_in[0] != 0x1f ||
                strm.next_in[1] != 0x8b) {
                done = true;
            } else {
                inflateReset(&strm);
                sink.reset();
                member_out = 0;
            }
        } else if (ret == Z_BUF_ERROR && strm.avail_in == 0 && in_left == 0) {
            done = true; // truncated file: keep what we have
        } else if (ret != Z_OK && ret != Z_BUF_ERROR) {
            err = strm.msg ? strm.msg : "inflate failed";
            inflateEnd(&strm);
            return -1;
        }
    }
    sink.finish();
    inflateEnd(&strm);
    return sink.produced();
}

/*
 * FastInflater is based on puff.c, the reference inflate in zlib's
 * contrib/puff (version 2.3, Copyright (C) 2002-2013 Mark Adler).
 * build() is based on the structure of puff's construct(), the slow path
 * of decode() on puff's decode(), and codes(), stored(), fixedBlock() and
 * dynamicBlock() on puff's codes(), stored(), fixed() and dynamic(). This
 * is an altered version, not the original software: the bit buffer, the
 * table-driven fast decode, the handling of zero runs, the output sink
 * and the gzip framing were changed or added for gem5. The changes are
 * ours: Mark Adler did not write, review or endorse this version, and
 * problems with it should be reported to gem5-opt, not to him. puff's
 * notice is reproduced unmodified below, as its license requires.
 *
 * puff.h
 * Copyright (C) 2002-2013 Mark Adler, all rights reserved
 * version 2.3, 21 Jan 2013
 *
 * This software is provided 'as-is', without any express or implied
 * warranty.  In no event will the author be held liable for any damages
 * arising from the use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 *
 * Mark Adler    madler@alumni.caltech.edu
 */

/**
 * A small DEFLATE decoder for the common case of a checkpoint image:
 * one gzip member whose data fills the destination exactly. It exists
 * because checkpointed memories are mostly zero, which deflate codes
 * as long runs of (length 258, distance 1) matches; zlib copies those
 * byte by byte, while here they become memset(). Any surprise (bad
 * data, multiple members, size mismatch, CRC mismatch) makes it return
 * false, and the caller redoes the whole image with zlib.
 */
class FastInflater
{
  public:
    FastInflater(const uint8_t *in, size_t in_len, RestoreSink &sink)
        : in(in), inLen(in_len), sink(sink)
    {}

    bool run();

  private:
    static constexpr int maxBits = 15;
    static constexpr int fastBits = 10;
    static constexpr size_t windowSize = 32768;
    static constexpr size_t chunk = 1 << 20;
    static constexpr size_t bufSize =
        windowSize + chunk + 2 * 258 + restorePageSize;

    struct Huffman
    {
        // fast[] entry: symbol << 4 | length, 0 if longer than fastBits
        uint16_t fast[1 << fastBits];
        uint16_t count[maxBits + 1];
        uint16_t symbol[288];
        // LSB-first code and length of each symbol (length 0: unused)
        uint16_t rcode[288];
        uint8_t clen[288];
    };

    const uint8_t *in;
    size_t inLen;
    size_t inPos = 0;
    uint64_t bitBuf = 0;
    int bitCnt = 0;
    bool overread = false;
    RestoreSink &sink;

    std::unique_ptr<uint8_t[]> buf;
    size_t pos = 0;     // write position in buf
    size_t emitted = 0; // buf[0, emitted) has been handed to the sink
    uint64_t total = 0; // bytes decoded so far

    Huffman lencode, distcode;

    void
    refill()
    {
        while (bitCnt <= 56) {
            uint64_t b = 0;
            if (inPos < inLen) {
                b = in[inPos];
            } else {
                overread = true;
            }
            inPos++;
            bitBuf |= b << bitCnt;
            bitCnt += 8;
        }
    }

    uint32_t
    bits(int n)
    {
        if (bitCnt < n) {
            refill();
        }
        uint32_t v = bitBuf & ((1ull << n) - 1);
        bitBuf >>= n;
        bitCnt -= n;
        return v;
    }

    bool build(Huffman &h, const uint8_t *lengths, int n);
    int decode(const Huffman &h);
    bool flush(bool final);
    bool zeroFill(uint64_t n);
    uint64_t skipZeroMatches();
    bool codes();
    bool stored();
    bool fixedBlock();
    bool dynamicBlock();
};

bool
FastInflater::build(Huffman &h, const uint8_t *lengths, int n)
{
    std::memset(h.count, 0, sizeof(h.count));
    for (int s = 0; s < n; s++) {
        h.count[lengths[s]]++;
    }
    if (h.count[0] == n) {
        return false;
    }
    int left = 1;
    for (int len = 1; len <= maxBits; len++) {
        left <<= 1;
        left -= h.count[len];
        if (left < 0) {
            return false; // over-subscribed
        }
    }
    uint16_t offs[maxBits + 2];
    offs[1] = 0;
    for (int len = 1; len < maxBits; len++) {
        offs[len + 1] = offs[len] + h.count[len];
    }
    for (int s = 0; s < n; s++) {
        if (lengths[s]) {
            h.symbol[offs[lengths[s]]++] = s;
        }
    }

    // Fast table: canonical codes, bit-reversed for LSB-first reading.
    std::memset(h.fast, 0, sizeof(h.fast));
    std::memset(h.clen, 0, sizeof(h.clen));
    uint32_t code = 0;
    int idx = 0;
    for (int len = 1; len <= maxBits; len++) {
        for (int k = 0; k < h.count[len]; k++, idx++, code++) {
            uint32_t rev = 0;
            for (int b = 0; b < len; b++) {
                rev |= ((code >> b) & 1) << (len - 1 - b);
            }
            h.rcode[h.symbol[idx]] = rev;
            h.clen[h.symbol[idx]] = len;
            if (len > fastBits) {
                continue;
            }
            for (uint32_t f = rev; f < (1u << fastBits); f += 1u << len) {
                h.fast[f] = h.symbol[idx] << 4 | len;
            }
        }
        code <<= 1;
    }
    return true;
}

/**
 * Called right after a (length 258, distance 1) match that wrote zeros.
 * A memory image is mostly zero, which deflate codes as a long series
 * of exactly that match, i.e. one fixed bit pattern repeated. Consume
 * as many further repeats as directly follow in the bit stream (but no
 * more than the destination can take) and return how many there were.
 * Each repeat is exactly what decode(lencode) + decode(distcode) would
 * have read as symbol 285 (length 258, no extra bits) and distance
 * symbol 0 (distance 1, no extra bits), so the decoded output is the
 * same: 258 more zero bytes each. The caller emits them via zeroFill().
 */
uint64_t
FastInflater::skipZeroMatches()
{
    int l1 = lencode.clen[285], l2 = distcode.clen[0];
    if (!l1 || !l2) {
        return 0;
    }
    int k = l1 + l2;
    uint64_t pat = lencode.rcode[285] | (uint64_t)distcode.rcode[0] << l1;
    // m repeats at once, m * k <= 56 (refill() leaves more than 56 bits)
    int m = 56 / k;
    uint64_t patm = 0;
    for (int i = 0; i < m; i++) {
        patm |= pat << (i * k);
    }
    uint64_t maskm = (1ull << (m * k)) - 1;
    uint64_t mask = (1ull << k) - 1;

    // Output that may still be produced: what the sink can take, minus
    // what is decoded but not yet handed over.
    uint64_t pend = pos - emitted;
    uint64_t room = sink.room() > pend ? sink.room() - pend : 0;
    uint64_t maxReps = room / 258;

    uint64_t r = 0;
    for (;;) {
        if (bitCnt <= 56) {
            refill();
        }
        // Never act on bits made up past the end of the input.
        if (overread) {
            break;
        }
        if (r + m <= maxReps && (bitBuf & maskm) == patm) {
            bitBuf >>= m * k;
            bitCnt -= m * k;
            r += m;
        } else if (r < maxReps && (bitBuf & mask) == pat) {
            bitBuf >>= k;
            bitCnt -= k;
            r++;
        } else {
            break;
        }
    }
    return r;
}

/**
 * Append @p n zero bytes to the output, with the same result as @p n
 * literal zeros. Long runs are not materialized: whole zero pages go
 * to the sink as a page count, and the window becomes all zeros.
 */
bool
FastInflater::zeroFill(uint64_t n)
{
    uint8_t *b = buf.get();
    if (n >= windowSize + 2 * restorePageSize) {
        // Pad (in the buffer) up to an output page boundary.
        uint64_t head =
            (restorePageSize - total % restorePageSize) % restorePageSize;
        std::memset(b + pos, 0, head);
        pos += head;
        total += head;
        n -= head;
        // The sink has only been handed whole pages so far (flush(false)
        // hands over page multiples), and total is page aligned now, so
        // the pending part is whole pages too.
        size_t pend = pos - emitted;
        if (pend + n > sink.room()) {
            return false; // image larger than the memory: let zlib do it
        }
        sink.put(b + emitted, pend);
        uint64_t pages = n / restorePageSize;
        sink.putZeroPages(pages);
        total += pages * restorePageSize;
        n -= pages * restorePageSize;
        // n >= windowSize before this, so the last windowSize bytes of
        // output (all a later match may refer to) are zero.
        std::memset(b, 0, windowSize);
        pos = emitted = windowSize;
    }
    while (n) {
        if (pos >= windowSize + chunk && !flush(false)) {
            return false;
        }
        size_t c = std::min<uint64_t>(n, windowSize + chunk + 258 - pos);
        std::memset(b + pos, 0, c);
        pos += c;
        total += c;
        n -= c;
    }
    return true;
}

int
FastInflater::decode(const Huffman &h)
{
    if (bitCnt < maxBits) {
        refill();
    }
    uint16_t e = h.fast[bitBuf & ((1u << fastBits) - 1)];
    if (e) {
        int len = e & 15;
        bitBuf >>= len;
        bitCnt -= len;
        return e >> 4;
    }
    // Slow path (codes longer than fastBits), canonical decoding.
    int code = 0, first = 0, index = 0;
    for (int len = 1; len <= maxBits; len++) {
        code |= (bitBuf >> (len - 1)) & 1;
        int count = h.count[len];
        if (code - count < first) {
            bitBuf >>= len;
            bitCnt -= len;
            return h.symbol[index + (code - first)];
        }
        index += count;
        first += count;
        first <<= 1;
        code <<= 1;
    }
    return -1;
}

bool
FastInflater::flush(bool final)
{
    // Hand whole pages to the sink, keep the last 32 KiB as window.
    size_t n = pos - emitted;
    if (!final) {
        n -= n % restorePageSize;
    }
    if (n > sink.room()) {
        return false; // image larger than the memory: let zlib do it
    }
    sink.put(buf.get() + emitted, n);
    emitted += n;
    if (!final) {
        size_t keep = std::min(emitted, pos - std::min(pos, windowSize));
        std::memmove(buf.get(), buf.get() + keep, pos - keep);
        pos -= keep;
        emitted -= keep;
    }
    return true;
}

bool
FastInflater::codes()
{
    static const uint16_t lbase[29] = {
        3,  4,  5,  6,  7,  8,  9,  10, 11,  13,  15,  17,  19,  23, 27,
        31, 35, 43, 51, 59, 67, 83, 99, 115, 131, 163, 195, 227, 258};
    static const uint8_t lext[29] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 1,
                                     1, 1, 2, 2, 2, 2, 3, 3, 3, 3,
                                     4, 4, 4, 4, 5, 5, 5, 5, 0};
    static const uint16_t dbase[30] = {
        1,    2,    3,    4,    5,    7,    9,    13,    17,    25,
        33,   49,   65,   97,   129,  193,  257,  385,   513,   769,
        1025, 1537, 2049, 3073, 4097, 6145, 8193, 12289, 16385, 24577};
    static const uint8_t dext[30] = {0, 0, 0,  0,  1,  1,  2,  2,  3,  3,
                                     4, 4, 5,  5,  6,  6,  7,  7,  8,  8,
                                     9, 9, 10, 10, 11, 11, 12, 12, 13, 13};

    uint8_t *b = buf.get();
    for (;;) {
        if (pos >= windowSize + chunk) {
            if (!flush(false)) {
                return false;
            }
        }
        if (overread) {
            return false;
        }
        int sym = decode(lencode);
        if (sym < 0) {
            return false;
        }
        if (sym < 256) {
            b[pos++] = sym;
            total++;
        } else if (sym == 256) {
            return true;
        } else {
            sym -= 257;
            if (sym >= 29) {
                return false;
            }
            size_t len = lbase[sym] + bits(lext[sym]);
            int dsym = decode(distcode);
            if (dsym < 0 || dsym >= 30) {
                return false;
            }
            size_t dist = dbase[dsym] + bits(dext[dsym]);
            if (dist > total || dist > pos) {
                return false;
            }
            uint8_t *o = b + pos;
            if (dist == 1) {
                std::memset(o, o[-1], len);
                if (len == 258 && o[-1] == 0) {
                    pos += len;
                    total += len;
                    uint64_t r = skipZeroMatches();
                    if (r && !zeroFill(r * 258)) {
                        return false;
                    }
                    continue;
                }
            } else {
                size_t left = len;
                while (left) {
                    size_t c = std::min(left, dist);
                    std::memcpy(o, o - dist, c);
                    o += c;
                    left -= c;
                }
            }
            pos += len;
            total += len;
        }
    }
}

bool
FastInflater::stored()
{
    // Drop to a byte boundary; whole bytes still in bitBuf come first.
    bits(bitCnt & 7);
    uint32_t len = bits(16);
    uint32_t nlen = bits(16);
    if ((len ^ 0xffff) != nlen) {
        return false;
    }
    while (len--) {
        if (pos >= windowSize + chunk && !flush(false)) {
            return false;
        }
        buf[pos++] = bits(8);
        total++;
    }
    return !overread;
}

bool
FastInflater::fixedBlock()
{
    uint8_t lengths[288];
    int s = 0;
    for (; s < 144; s++) {
        lengths[s] = 8;
    }
    for (; s < 256; s++) {
        lengths[s] = 9;
    }
    for (; s < 280; s++) {
        lengths[s] = 7;
    }
    for (; s < 288; s++) {
        lengths[s] = 8;
    }
    build(lencode, lengths, 288);
    for (s = 0; s < 30; s++) {
        lengths[s] = 5;
    }
    build(distcode, lengths, 30);
    return codes();
}

bool
FastInflater::dynamicBlock()
{
    static const uint8_t order[19] = {16, 17, 18, 0, 8,  7, 9,  6, 10, 5,
                                      11, 4,  12, 3, 13, 2, 14, 1, 15};
    int nlen = bits(5) + 257;
    int ndist = bits(5) + 1;
    int ncode = bits(4) + 4;
    if (nlen > 286 || ndist > 30) {
        return false;
    }
    uint8_t lengths[320] = {};
    for (int i = 0; i < ncode; i++) {
        lengths[order[i]] = bits(3);
    }
    Huffman lencl;
    if (!build(lencl, lengths, 19)) {
        return false;
    }
    int idx = 0;
    while (idx < nlen + ndist) {
        int sym = decode(lencl);
        if (sym < 0 || overread) {
            return false;
        }
        if (sym < 16) {
            lengths[idx++] = sym;
        } else {
            int len = 0, rep;
            if (sym == 16) {
                if (idx == 0) {
                    return false;
                }
                len = lengths[idx - 1];
                rep = 3 + bits(2);
            } else if (sym == 17) {
                rep = 3 + bits(3);
            } else {
                rep = 11 + bits(7);
            }
            if (idx + rep > nlen + ndist) {
                return false;
            }
            while (rep--) {
                lengths[idx++] = len;
            }
        }
    }
    if (lengths[256] == 0) {
        return false;
    }
    // build() rejects only over-subscribed codes. A bit pattern that an
    // incomplete code leaves unassigned decodes to -1, and the image
    // then goes to the zlib fallback.
    if (!build(lencode, lengths, nlen) ||
        !build(distcode, lengths + nlen, ndist)) {
        return false;
    }
    return codes();
}

bool
FastInflater::run()
{
    // gzip header (RFC 1952)
    if (inLen < 18 || in[0] != 0x1f || in[1] != 0x8b || in[2] != 8) {
        return false;
    }
    uint8_t flg = in[3];
    size_t p = 10;
    if (flg & 4) {
        if (p + 2 > inLen) {
            return false;
        }
        p += 2 + (in[p] | in[p + 1] << 8);
    }
    if (flg & 8) {
        while (p < inLen && in[p]) {
            p++;
        }
        p++;
    }
    if (flg & 16) {
        while (p < inLen && in[p]) {
            p++;
        }
        p++;
    }
    if (flg & 2) {
        p += 2;
    }
    if (p >= inLen) {
        return false;
    }
    inPos = p;

    buf.reset(new uint8_t[bufSize]);
    bool last;
    do {
        last = bits(1);
        int type = bits(2);
        bool ok;
        if (type == 0) {
            ok = stored();
        } else if (type == 1) {
            ok = fixedBlock();
        } else if (type == 2) {
            ok = dynamicBlock();
        } else {
            ok = false;
        }
        if (!ok || overread) {
            return false;
        }
    } while (!last);
    if (!flush(true)) {
        return false;
    }

    // Trailer starts at the next byte boundary after the last block.
    size_t used = inPos - bitCnt / 8;
    if (used + 8 > inLen) {
        return false;
    }
    // Exactly one member that fills the memory, as gem5 writes it.
    if (used + 8 != inLen || sink.room() != 0) {
        return false;
    }
    return le32(in + used) == (uint32_t)sink.crc() &&
           le32(in + used + 4) == (uint32_t)total;
}

/**
 * Inflate a gzip image (@p in, fully in memory) into @p dst, writing at
 * most @p dst_len bytes; see RestoreSink. Tries FastInflater first and
 * falls back to zlib. Returns bytes produced, or -1 with @p err set.
 */
int64_t
inflateGzipImage(const uint8_t *in, size_t in_len, uint8_t *dst,
                 uint64_t dst_len, bool dst_anon, std::string &err)
{
    {
        RestoreSink sink(dst, dst_len, dst_anon);
        FastInflater fast(in, in_len, sink);
        bool ok = fast.run();
        sink.finish();
        if (ok) {
            return sink.produced();
        }
    }
    RestoreSink sink(dst, dst_len, dst_anon);
    return zlibInflateImage(in, in_len, sink, err);
}

} // anonymous namespace

void
PhysicalMemory::unserializeStore(CheckpointIn &cp)
{
    const uint32_t chunk_size = 16384;

    unsigned int store_id;
    UNSERIALIZE_SCALAR(store_id);

    std::string filename;
    UNSERIALIZE_SCALAR(filename);
    std::string filepath = cp.getCptDir() + "/" + filename;

    // Map the compressed file; the fast path below needs it in memory.
    int fd = open(filepath.c_str(), O_RDONLY);
    if (fd < 0) {
        fatal("Can't open physical memory checkpoint file '%s'", filename);
    }
    struct stat st;
    if (fstat(fd, &st)) {
        fatal("Can't stat physical memory checkpoint file '%s'", filename);
    }
    size_t file_size = st.st_size;
    const uint8_t *file_data = nullptr;
    if (file_size >= 18) {
        void *m = mmap(NULL, file_size, PROT_READ, MAP_PRIVATE, fd, 0);
        if (m != MAP_FAILED) {
            file_data = static_cast<const uint8_t *>(m);
            madvise(m, file_size, MADV_SEQUENTIAL);
        }
    }
    close(fd);

    // we've already got the actual backing store mapped
    uint8_t* pmem = backingStore[store_id].pmem;
    AddrRange range = backingStore[store_id].range;

    Addr range_size;
    UNSERIALIZE_SCALAR(range_size);

    DPRINTF(Checkpoint, "Unserializing physical memory %s with size %d\n",
            filename, range_size);

    if (range_size != range.size())
        fatal("Memory range size has changed! Saw %lld, expected %lld\n",
              range_size, range.size());

    if (file_data && file_data[0] == 0x1f && file_data[1] == 0x8b) {
        // gzip: inflate ourselves, skipping writes of all-zero pages.
        // Untouched private anonymous memory already reads as zero.
        bool anon = backingStore[store_id].shmFd == -1;
        uint64_t size = range.size();
        auto job = [=]() -> std::string {
            std::string err;
            int64_t got =
                inflateGzipImage(file_data, file_size, pmem, size, anon, err);
            munmap(const_cast<uint8_t *>(file_data), file_size);
            if (got < 0) {
                return csprintf("Error reading physical memory checkpoint "
                                "file '%s': %s\n",
                                filename, err);
            }
            return std::string();
        };
        if (pendingRestores) {
            pendingRestores->push_back(job);
        } else {
            std::string err = job();
            if (!err.empty()) {
                fatal("%s", err);
            }
        }
        return;
    }
    if (file_data) {
        munmap(const_cast<uint8_t *>(file_data), file_size);
    }

    // Fallback (e.g. uncompressed file): let zlib handle it.
    gzFile compressed_mem = gzopen(filepath.c_str(), "rb");
    if (compressed_mem == NULL) {
        fatal("Can't open physical memory checkpoint file '%s'", filename);
    }

    uint64_t curr_size = 0;
    uint32_t bytes_read;
    while (curr_size < range.size()) {
        bytes_read = gzread(compressed_mem, pmem, chunk_size);
        if (bytes_read == 0)
            break;
        curr_size += bytes_read;
        pmem += bytes_read;
    }

    if (gzclose(compressed_mem))
        fatal("Close failed on physical memory checkpoint file '%s'\n",
              filename);
}

} // namespace memory
} // namespace gem5
