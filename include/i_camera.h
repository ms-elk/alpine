#pragma once

namespace alpine {
class ICamera
{
public:
    virtual void setLookAt(
        const float eye[3],
        const float target[3],
        const float up[3],
        float fovy,
        float aspect) = 0;
    virtual void orbit(float theta, float phi) = 0;
    virtual void zoom(float z) = 0;
    virtual void pan(float x, float y) = 0;
};
}
