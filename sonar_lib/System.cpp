#include "System.h"

namespace sonar {

System::System():
    m_state(TrackingState::Lost),
    m_quality(TrackingQuality::Ugly)
{

}

} // namespace sonar
