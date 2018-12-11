#ifndef SONAR_MAPFRAME_H
#define SONAR_MAPFRAME_H

#include <Eigen/Core>

namespace sonar {

class MapFrame
{
public:
    MapFrame();



private:
    Eigen::Matrix3d m_R;
    Eigen::Vector3d m_t;
};

} // namespace sonar

#endif // SONAR_MAPFRAME_H
