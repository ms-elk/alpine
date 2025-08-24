#include "animation.h"

#include <shapes/shape.h>

#include <algorithm>
#include <assert.h>
#include <cmath>

namespace alpine {
void
Animation::update(Accelerator* accelerator, float time)
{
    for (const auto& channel : mChannels)
    {
        float channelTime = std::clamp(
            time, channel.morphKeyframes.front().time, channel.morphKeyframes.back().time);

        auto index = [&]() {
            for (uint32_t i = 0; i < channel.morphKeyframes.size() - 1; ++i)
            {
                if (channelTime <= channel.morphKeyframes[i + 1].time)
                {
                    return i;
                }
            }

            return static_cast<uint32_t>(channel.morphKeyframes.size()) - 2;
        }();

        const auto& mkf0 = channel.morphKeyframes[index];
        const auto& mkf1 = channel.morphKeyframes[index+1];

        float t0 = mkf0.time;
        float t1 = mkf1.time;
        float t = (channelTime - t0) / (t1 - t0);

        channel.shape->update(accelerator, mkf0.weights, mkf1.weights, t);
    }
}
}
