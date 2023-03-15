/**
 * Copyright (c) 2018-2020 Inria
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

/**
 * @file
 * Declaration of a Least Recently Used replacement policy.
 * The victim is chosen using the last touch timestamp.
 */

#ifndef __MEM_CACHE_REPLACEMENT_POLICIES_LOCALITY_RP_HH__
#define __MEM_CACHE_REPLACEMENT_POLICIES_LOCALITY_RP_HH__

#include <queue>
#include "mem/cache/replacement_policies/base.hh"
#include "base/sat_counter.hh"

namespace gem5
{

struct LOCALITYRPParams;

GEM5_DEPRECATED_NAMESPACE(ReplacementPolicy, replacement_policy);
namespace replacement_policy
{

class LOCALITY : public Base
{
  protected:
    /** LRU-specific implementation of replacement data. */
    struct LRUReplData : ReplacementData
    {
        /** Tick on which the entry was last touched. */
        Tick lastTouchTick;

        /**
         * Default constructor. Invalidate data.
         */
        LRUReplData() : lastTouchTick(0) {}
    };

    /** BRRIP-specific implementation of replacement data. */
    struct BRRIPReplData : ReplacementData
    {
        /**
         * Re-Reference Interval Prediction Value.
         * Some values have specific names (according to the paper):
         * 0 -> near-immediate re-rereference interval
         * max_RRPV-1 -> long re-rereference interval
         * max_RRPV -> distant re-rereference interval
         */
        SatCounter8 rrpv;

        /** Whether the entry is valid. */
        bool valid;

        /**
         * Default constructor. Invalidate data.
         */
        BRRIPReplData(const int num_bits)
            : rrpv(num_bits), valid(false)
        {
        }
    };

    /**
     * Number of RRPV bits. An entry that saturates its RRPV has the longest
     * possible re-reference interval, that is, it is likely not to be used
     * in the near future, and is among the best eviction candidates.
     * A maximum RRPV of 1 implies in a NRU.
     */
    const unsigned numRRPVBits;

    /**
     * The hit priority (HP) policy replaces entries that do not receive cache
     * hits over any cache entry that receives a hit, while the frequency
     * priority (FP) policy replaces infrequently re-referenced entries.
     */
    const bool hitPriority;

    /**
     * Bimodal throtle parameter. Value in the range [0,100] used to decide
     * if a new entry is inserted with long or distant re-reference.
     */
    const unsigned btp;

    // replacement policy decision: 1 = LRU, 2 = BRRIP
    int replacementPolicy;

  public:
    typedef LOCALITYRPParams Params;
    LOCALITY(const Params &p);
    ~LOCALITY() = default;

    /**
     * Invalidate replacement data to set it as the next probable victim.
     * Sets its last touch tick as the starting tick.
     *
     * @param replacement_data Replacement data to be invalidated.
     */
    void invalidate(const std::shared_ptr<ReplacementData>& replacement_data)
                                                                    override;

    /**
     * Touch an entry to update its replacement data.
     * Sets its last touch tick as the current tick.
     *
     * @param replacement_data Replacement data to be touched.
     */
    void touch(const std::shared_ptr<ReplacementData>& replacement_data) const
                                                                     override;

    void get_locality(std::queue<unsigned long>);
    void locality(const PacketPtr pkt) override;
    /**
     * Reset replacement data. Used when an entry is inserted.
     * Sets its last touch tick as the current tick.
     *
     * @param replacement_data Replacement data to be reset.
     */
    void reset(const std::shared_ptr<ReplacementData>& replacement_data) const
                                                                     override;

    /**
     * Find replacement victim using LRU timestamps.
     *
     * @param candidates Replacement candidates, selected by indexing policy.
     * @return Replacement entry to be replaced.
     */
    ReplaceableEntry* getVictim(const ReplacementCandidates& candidates) const
                                                                     override;

    /**
     * Instantiate a replacement data entry.
     *
     * @return A shared pointer to the new replacement data.
     */
    std::shared_ptr<ReplacementData> instantiateEntry() override;
};

} // namespace replacement_policy
} // namespace gem5

#endif // __MEM_CACHE_REPLACEMENT_POLICIES_LOCALITY_RP_HH__
