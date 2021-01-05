#ifndef TEST_SYNTETIC_INITIALIZATION_H
#define TEST_SYNTETIC_INITIALIZATION_H

namespace sonar {

class AbstractInitializator;

} // namespace sonar

bool test_synthetic_initialization(sonar::AbstractInitializator * initializator, bool use_plane_flag = false);
bool test_new_decomposition();

#endif // TEST_SYNTETIC_INITIALIZATION_H
