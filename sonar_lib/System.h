#ifndef SONAR_SYSTEM_H
#define SONAR_SYSTEM_H

#include <memory>

namespace sonar {

enum class TrackingState: int
{
    NotStarted = 0,
    Initializing,
    Tracking,
    Lost
};

enum class TrackingQuality: int
{
    Good = 0,
    Bad,
    Ugly
};

enum class ProcessingMode: int
{
    CPU = 0,
    OPENGL,
    CPU_OPENCV
};

// forward declaration
class SourceFrame;
class AbstractInitTracker;
class Initializator;
class MapFrame;
class AbstractCamera;

class System
{
public:
    System();

    TrackingState trackingState() const;
    TrackingQuality trackingQuality() const;
    ProcessingMode processingMode() const;
    void setProcessingMode(ProcessingMode mode);

    std::shared_ptr<const AbstractInitTracker> initTracker() const;
    std::shared_ptr<AbstractInitTracker> initTracker();

    std::shared_ptr<const AbstractCamera> camera() const;
    void setCamera(const std::shared_ptr<const AbstractCamera> & camera);

    void start();
    void reset();

    std::shared_ptr<const MapFrame> process(const SourceFrame & sourceFrame);

private:
    TrackingState m_trackingState;
    TrackingQuality m_trackingQuality;
    ProcessingMode m_processingMode;

    std::shared_ptr<AbstractInitTracker> m_initTracker;
    std::shared_ptr<Initializator> m_initializator;

    std::shared_ptr<const AbstractCamera> m_camera;
};

} // namespace sonar

#endif // SONAR_SYSTEM_H
