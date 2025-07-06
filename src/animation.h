#pragma once

#include <memory>
#include <vector>

namespace alpine {
class Accelerator;
class Shape;

class Animation
{
public:
    struct Channel
    {
        struct MorphKeyframe
        {
            float time;
            std::vector<float> weights;
        };
        std::vector<MorphKeyframe> morphKeyframes;
        std::shared_ptr<Shape> shape;
    };

    Animation(std::vector<Channel>&& channels)
        : mChannels(std::move(channels)) {}

    void update(Accelerator* accelerator, float time);

private:
    std::vector<Channel> mChannels;
};
}
