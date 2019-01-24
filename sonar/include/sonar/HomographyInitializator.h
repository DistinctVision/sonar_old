#ifndef SONAR_HOMOGRAPHYINITIALIZATOR_H
#define SONAR_HOMOGRAPHYINITIALIZATOR_H

#include "AbstractInitializator.h"

namespace sonar {

class HomographyInitializator:
        public AbstractInitializator
{
public:
    HomographyInitializator();

    int numberRansacIterations() const;
    void setNumberRansacIterations(int numberRansacIterations);

    std::tuple<bool, Info> compute(const std::shared_ptr<const AbstractCamera> & firstCamera,
                                   const std::vector<Point2d> & firstFrame,
                                   const std::shared_ptr<const AbstractCamera> & secondCamera,
                                   const std::vector<Point2d> & secondFrame,
                                   const std::shared_ptr<const AbstractCamera> & thirdCamera,
                                   const std::vector<Point2d> & thirdFrame,
                                   bool alignByPlaneFlag = true) override;

private:
    struct _Decomposition
    {
        Eigen::Matrix3d R;
        Eigen::Vector3d t;
        Eigen::Vector3d planeNormal;
        double planeD;
    };

    int m_numberRansacIterations;

    std::tuple<transformation_t, transformation_t,
               std::vector<int>, points_t,
               Eigen::Vector3d, Eigen::Vector3d>
    _compute(const bearingVectors_t & firstDirs,
             const bearingVectors_t & secondDirs,
             const bearingVectors_t & thirdDirs,
             double firstThreshold, double secondThreshold, double thirdThreshold) const;

    Eigen::Matrix3d _computeHomography(const bearingVectors_t & dirs_a,
                                       const bearingVectors_t & dirs_b,
                                       const std::vector<int> & samples) const;

    std::vector<_Decomposition> _decompose(const Eigen::Matrix3d & H) const;

    int _getNumberVisiblePoints(const _Decomposition & decomposition,
                                const bearingVectors_t & dirs_a,
                                const bearingVectors_t & dirs_b,
                                const std::vector<int> & indices,
                                const points_t & points) const;

    points_t _computePoints(const bearingVectors_t & dirs_a,
                            const bearingVectors_t & dirs_b,
                            const Eigen::Matrix3d & R, const Eigen::Vector3d & t) const;

    points_t _computePoints(const bearingVectors_t & dirs_a,
                            const bearingVectors_t & dirs_b,
                            const std::vector<int> & indices,
                            const Eigen::Matrix3d & R, const Eigen::Vector3d & t) const;
    bool _pickPlane(double & t,
                    const Eigen::Vector3d & planeNormal, const Eigen::Vector3d & planePoint,
                    const Eigen::Vector3d & rayDir, const Eigen::Vector3d & rayPoint) const;
};

} // namespace sonar

#endif // SONAR_HOMOGRAPHYINITIALIZATOR_H
