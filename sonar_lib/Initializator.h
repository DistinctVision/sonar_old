#ifndef SONAR_INITIALIZATOR_H
#define SONAR_INITIALIZATOR_H

#include <memory>
#include <vector>

#include <Eigen/Core>

#include "General/Point2.h"

namespace sonar {

// forward declaration
class AbstractCamera;

class Initializator
{
public:
    Initializator();

    std::shared_ptr<const AbstractCamera> camera() const;
    void setCamera(const std::shared_ptr<const AbstractCamera> & camera);

    bool init(const std::vector<Point2d> & firstFrame,
              const std::vector<Point2d> & secondFrame,
              const std::vector<Point2d> & thirdFrame);

private:
    std::shared_ptr<const AbstractCamera> m_camera;
};

} // namespace sonar

#endif // SONAR_INITIALIZATOR_H
