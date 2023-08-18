#pragma once

namespace alpine {
class ILight
{
public:
    virtual void enable(bool enabled) = 0;
    virtual bool isEnabled() const = 0;
    virtual void setPosition(const float position[3]) = 0;
    virtual void setScale(float scale) = 0;
};
}
