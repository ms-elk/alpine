#pragma once

#include "utils/alias_table.h"
#include "light_sampler.h"

namespace alpine {
class PowerLightSampler : public LightSampler
{
public:
    PowerLightSampler(const std::vector<std::shared_ptr<Light>>& lights);

    Sample sample(float u) const override;

private:
    std::vector<std::shared_ptr<Light>> mLights;
    AliasTable mAliasTable;
};
}
