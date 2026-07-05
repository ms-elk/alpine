#pragma once

namespace alpine {
class LightHandle
{
public:
    virtual void setPosition(const float position[3]) = 0;
    virtual void setScale(float scale) = 0;
};
}
