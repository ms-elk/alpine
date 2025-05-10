#include "alias_table.h"

#include <utils/util.h>

#include <numeric>

namespace alpine {
AliasTable::AliasTable(std::vector<float> weights)
{
    mBins.resize(weights.size());
    std::vector<float> scaledProbs(weights.size());

    float sum = static_cast<float>(std::accumulate(weights.begin(), weights.end(), 0.0));
    for (uint32_t i = 0; i < weights.size(); ++i)
    {
        mBins[i].p = weights[i] / sum;
        scaledProbs[i] = mBins[i].p * weights.size();
    }

    std::vector<uint32_t> small, large;
    small.reserve(scaledProbs.size());
    large.reserve(scaledProbs.size());

    for (uint32_t i = 0; i < scaledProbs.size(); ++i)
    {
        if (scaledProbs[i] < 1.0f)
        {
            small.push_back(i);
        }
        else
        {
            large.push_back(i);
        }
    }

    while (!small.empty() && !large.empty())
    {
        uint32_t smallIdx = small.back();
        uint32_t largeIdx = large.back();
        small.pop_back();
        large.pop_back();

        mBins[smallIdx].q = scaledProbs[smallIdx];
        mBins[smallIdx].alias = largeIdx;

        scaledProbs[largeIdx] += scaledProbs[smallIdx] - 1.0f;
        if (scaledProbs[largeIdx] < 1.0f)
        {
            small.push_back(largeIdx);
        }
        else
        {
            large.push_back(largeIdx);
        }
    }

    const auto setOneToBin = [&](auto& work) {
        while (!work.empty())
        {
            auto& bin = mBins[work.back()];
            work.pop_back();
            bin.q = 1.0f;
            bin.alias = std::numeric_limits<uint32_t>::max();
        }
    };

    setOneToBin(small);
    setOneToBin(large);
}

AliasTable::Sample
AliasTable::sample(float u) const
{
    uint32_t binIdx = static_cast<uint32_t>(u * mBins.size());
    binIdx = std::min(binIdx, static_cast<uint32_t>(mBins.size()) - 1);

    float up = std::min(u * mBins.size() - binIdx, ONE_MINUS_EPSILON);
    uint32_t sampleIdx = up < mBins[binIdx].q ? binIdx : mBins[binIdx].alias;

    return { sampleIdx, mBins[sampleIdx].p };
}
}
