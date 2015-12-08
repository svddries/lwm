#ifndef LWM_WORLD_MODEL_H_
#define LWM_WORLD_MODEL_H_

#include <geolib/datatypes.h>

namespace lwm
{

// ----------------------------------------------------------------------------------------------------

struct Line
{
    Line(unsigned int i1_, unsigned int i2_) : i1(i1_), i2(i2_) {}
    unsigned int i1;
    unsigned int i2;
};

// ----------------------------------------------------------------------------------------------------

class WorldModel
{

public:

    WorldModel();

    ~WorldModel();

    unsigned int addVertex(const geo::Vec2& v)
    {
        vertices_.push_back(v);
        return vertices_.size() - 1;
    }

    void addLine(unsigned int i1, unsigned int i2)
    {
        lines_.push_back(Line(i1, i2));
    }

    const std::vector<geo::Vec2>& vertices() const { return vertices_; }

    const std::vector<Line>& lines() const { return lines_; }

private:

    std::vector<geo::Vec2> vertices_;

    std::vector<Line> lines_;

};

} // end namespace lwm

#endif
