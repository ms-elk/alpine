#pragma once

#include <stdint.h>
#include <vector>

namespace alpine {
class AliasTable
{
public:
    AliasTable() = default;
    AliasTable(std::vector<float> weights);

    struct Sample
    {
        uint32_t idx;
        float pdf;
    };

    Sample sample(float u) const;

private:
    struct Bin
    {
        float q, p;
        uint32_t alias;
    };
    std::vector<Bin> mBins;
};
}
