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

#include "mem/cache/replacement_policies/locality_rp.hh"

#include <cassert>
#include <memory>

#include "params/LOCALITYRP.hh"
#include "sim/cur_tick.hh"

#include "mem/packet.hh"
#include <queue>
#include <cmath>

// BRRIP
#include "base/logging.hh" // For fatal_if
#include "base/random.hh"

namespace gem5
{

GEM5_DEPRECATED_NAMESPACE(ReplacementPolicy, replacement_policy);
namespace replacement_policy
{

LOCALITY::LOCALITY(const Params &p)
  : Base(p), numRRPVBits(p.num_bits), hitPriority(p.hit_priority),
    btp(p.btp), replacementPolicy(1)
{
    fatal_if(numRRPVBits <= 0, "There should be at least one bit per RRPV.\n");
}

void
LOCALITY::invalidate(const std::shared_ptr<ReplacementData>& replacement_data)
{
    if (1 == replacementPolicy)
    {
        // Reset last touch timestamp
        std::static_pointer_cast<LRUReplData>(
            replacement_data)->lastTouchTick = Tick(0);
    }
    else if (2 == replacementPolicy)
    {
        std::shared_ptr<BRRIPReplData> casted_replacement_data =
        std::static_pointer_cast<BRRIPReplData>(replacement_data);

        // Invalidate entry
        casted_replacement_data->valid = false;
    }
}

void
LOCALITY::touch(const std::shared_ptr<ReplacementData>& replacement_data) const
{
    if (1 == replacementPolicy)
    {
        // Update last touch timestamp
        std::static_pointer_cast<LRUReplData>(
            replacement_data)->lastTouchTick = curTick();
    }
    else if (2 == replacementPolicy)
    {
        std::shared_ptr<BRRIPReplData> casted_replacement_data =
        std::static_pointer_cast<BRRIPReplData>(replacement_data);

        // Update RRPV if not 0 yet
        // Every hit in HP mode makes the entry the last to be evicted, while
        // in FP mode a hit makes the entry less likely to be evicted
        if (hitPriority) {
            casted_replacement_data->rrpv.reset();
        } else {
            casted_replacement_data->rrpv--;
        }
    }
}

void
LOCALITY::locality(const PacketPtr pkt)
{
    //printf("touch: 0x%lx\n", pkt->getAddr());
    std::queue<unsigned long> access_queue;
    access_queue.push(pkt->getAddr());
    if(access_queue.size() >= 1000){
	access_queue.pop();
    }
    get_locality(access_queue);
}

void
LOCALITY::get_locality(std::queue<unsigned long> q)
{
  int len = q.size();
  // Get mean
  std::queue<unsigned long> q_mean;
  unsigned long long mean = 0;
  while (!q_mean.empty())
  {
    mean += q_mean.front();
    q_mean.pop();
  }
  mean /= len;
  // Calculate variance
  unsigned long long variance = 0;
  while (!q.empty())
  {
    variance += std::pow(q.front() - mean, 2);
    q.pop();
  }
  variance /= len;
  variance = std::sqrt(variance);
  //printf("0x%llx\n", variance);
  //printf("%llu\n", variance);

  if (500000 > variance)
  {
    printf("LRU replacement policy chosen\n");
    replacementPolicy = 1;
  }
  else
  {
    printf("BRRIP replacement policy chosen\n");
    replacementPolicy = 2;
  }
}

void
LOCALITY::reset(const std::shared_ptr<ReplacementData>& replacement_data) const
{
    if (1 == replacementPolicy)
    {
        // Set last touch timestamp
        std::static_pointer_cast<LRUReplData>(
            replacement_data)->lastTouchTick = curTick();
    }
    else if (2 == replacementPolicy)
    {
        std::shared_ptr<BRRIPReplData> casted_replacement_data =
        std::static_pointer_cast<BRRIPReplData>(replacement_data);

        // Reset RRPV
        // Replacement data is inserted as "long re-reference" if lower than btp,
        // "distant re-reference" otherwise
        casted_replacement_data->rrpv.saturate();
        if (random_mt.random<unsigned>(1, 100) <= btp) {
            casted_replacement_data->rrpv--;
        }

        // Mark entry as ready to be used
        casted_replacement_data->valid = true;
    }
}


ReplaceableEntry*
LOCALITY::getVictim(const ReplacementCandidates& candidates) const
{
    // There must be at least one replacement candidate
    assert(candidates.size() > 0);

    ReplaceableEntry* victim = candidates[0];

    if (1 == replacementPolicy)
    {
        for (const auto& candidate : candidates) {
            // Update victim entry if necessary
            if (std::static_pointer_cast<LRUReplData>(
                        candidate->replacementData)->lastTouchTick <
                    std::static_pointer_cast<LRUReplData>(
                        victim->replacementData)->lastTouchTick) {
                victim = candidate;
            }
        }
    }
    else if (2 == replacementPolicy)
    {
        // Store victim->rrpv in a variable to improve code readability
        int victim_RRPV = std::static_pointer_cast<BRRIPReplData>(
                            victim->replacementData)->rrpv;

        // Visit all candidates to find victim
        for (const auto& candidate : candidates) {
            std::shared_ptr<BRRIPReplData> candidate_repl_data =
                std::static_pointer_cast<BRRIPReplData>(
                    candidate->replacementData);

            // Stop searching for victims if an invalid entry is found
            if (!candidate_repl_data->valid) {
                return candidate;
            }

            // Update victim entry if necessary
            int candidate_RRPV = candidate_repl_data->rrpv;
            if (candidate_RRPV > victim_RRPV) {
                victim = candidate;
                victim_RRPV = candidate_RRPV;
            }
        }

        // Get difference of victim's RRPV to the highest possible RRPV in
        // order to update the RRPV of all the other entries accordingly
        int diff = std::static_pointer_cast<BRRIPReplData>(
            victim->replacementData)->rrpv.saturate();

        // No need to update RRPV if there is no difference
        if (diff > 0){
            // Update RRPV of all candidates
            for (const auto& candidate : candidates) {
                std::static_pointer_cast<BRRIPReplData>(
                    candidate->replacementData)->rrpv += diff;
            }
        }
    }

    return victim;
}

std::shared_ptr<ReplacementData>
LOCALITY::instantiateEntry()
{
    std::shared_ptr<ReplacementData> data(nullptr);
    if (1 == replacementPolicy)
    {
        data = std::make_shared<ReplacementData>(LRUReplData());
    }
    else if (2 == replacementPolicy)
    {
	data = std::make_shared<ReplacementData>(BRRIPReplData(numRRPVBits));
    }
    return data;
}

} // namespace replacement_policy
} // namespace gem5
