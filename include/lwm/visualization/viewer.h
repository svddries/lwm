#ifndef _VIEWER_H_
#define _VIEWER_H_

namespace lwm
{

class WorldModel;

class Viewer
{

public:

    Viewer();

    ~Viewer();

    void show(const WorldModel& world_model);

};

} // end namespace lwm

#endif
