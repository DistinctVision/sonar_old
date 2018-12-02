#ifndef SONAR_SYSTEM_H
#define SONAR_SYSTEM_H

namespace sonar {

enum class TrackingState
{
    Initializing,
    Tracking,
    Lost
};

enum class TrackingQuality
{
    Good,
    Bad,
    Ugly
};

class System
{
public:
    System();


private:
    TrackingState m_state;
    TrackingQuality m_quality;
};

} // namespace sonar

#endif // SONAR_SYSTEM_H
