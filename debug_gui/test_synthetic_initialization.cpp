#include "test_synthetic_initialization.h"

#include <sonar/AbstractInitializator.h>
#include <sonar/CameraTools/PinholeCamera.h>
#include <sonar/MapFrame.h>
#include <sonar/System.h>

#include <sonar/General/MathUtils.h>

#include <memory>
#include <chrono>
#include <cmath>
#include <random>
#include <Eigen/Eigen>

#include <QDebug>

using namespace std;
using namespace std::chrono;
using namespace Eigen;
using namespace sonar;

Vector3d generateRandomVector3d(double scale = 1.0)
{
    return Vector3d((rand() % 2000 - 1000),
                    (rand() % 2000 - 1000),
                    (rand() % 2000 - 1000)).normalized() * scale;
}

Matrix3d generateRandomRotationMatrix(double delta_angle = M_PI * 0.25)
{
    return math_utils::exp_rotationMatrix(generateRandomVector3d(delta_angle));
}

bool compare(const Vector3d & a, const Vector3d & b)
{
    Vector3d d = (a - b).array().abs();
    for (int i = 0; i < 3; ++i)
    {
        if (d(i) > 3e-2)
            return false;
    }
    return true;
}

bool compare(const Matrix3d & a, const Matrix3d & b)
{
    Matrix3d d = (a - b).array().abs();
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            if (d(i, j) > 3e-2)
                return false;
        }
    }
    return true;
}

bool test_synthetic_initialization(AbstractInitializator * initializator, bool use_plane_flag)
{
    initializator->setMaxPixelError(1.0f);

    shared_ptr<AbstractCamera> camera(new PinholeCamera(Point2d(100.0, 100.0),
                                                        Point2d(50.0, 50.0),
                                                        Point2i(100, 100)));

    Vector3d scenePoint(0.0, 0.0, 10.0);

    Matrix3d real_firstWorldRotation = Matrix3d::Identity();
    Vector3d real_firstWorldPosition = Vector3d::Zero();

    Matrix3d real_secondWorldRotation = generateRandomRotationMatrix();
    //real_secondWorldRotation = math_utils::exp_rotationMatrix(Vector3d(M_PI * 0.5, 0.0, 0.0));
    Vector3d real_secondWorldPosition = scenePoint - real_secondWorldRotation *
            Vector3d(0.0, 0.0, ((rand() % 10) * 1e-1) + 10.0);

    Matrix3d real_thirdWorldRotation = generateRandomRotationMatrix();
    //real_thirdWorldRotation = math_utils::exp_rotationMatrix(Vector3d(-M_PI * 0.5, 0.0, 0.0));
    Vector3d real_thirdWorldPosition = scenePoint - real_thirdWorldRotation *
            Vector3d(0.0, 0.0, ((rand() % 10) * 1e-1) + 10.0);

    Matrix3d first_R = real_firstWorldRotation.inverse();
    Vector3d first_t = - first_R * real_firstWorldPosition;
    Matrix3d second_R = real_secondWorldRotation.inverse();
    Vector3d second_t = - second_R * real_secondWorldPosition;
    Matrix3d third_R = real_thirdWorldRotation.inverse();
    Vector3d third_t = - third_R * real_thirdWorldPosition;

    sonar::points_t real_points;
    real_points.reserve(100);
    vector<Point2d> first_image_points(100);
    vector<Point2d> second_image_points(100);
    vector<Point2d> third_image_points(100);
    while (real_points.size() < 100)
    {
        Vector3d point;
        if (use_plane_flag && (real_points.size() < 50))
        {
            point = Vector3d((rand() % 100) * 0.05 - 2.5,
                             (rand() % 100) * 0.05 - 2.5,
                             0.0) + scenePoint;
        }
        else
        {
            point = Vector3d((rand() % 100) * 0.05 - 2.5,
                             (rand() % 100) * 0.05 - 2.5,
                             (rand() % 100) * 0.05 - 2.5) + scenePoint;
        }

        Vector3d first_local = first_R * point + first_t;
        if (first_local.z() <= 0.0)
            continue;
        Vector3d second_local = second_R * point + second_t;
        if (second_local.z() <= 0.0)
            continue;
        Vector3d third_local = third_R * point + third_t;
        if (third_local.z() <= 0.0)
            continue;
        first_image_points[real_points.size()] = camera->toImagePoint(first_local);
        second_image_points[real_points.size()] = camera->toImagePoint(second_local);
        third_image_points[real_points.size()] = camera->toImagePoint(third_local);
        real_points.push_back(point);
    }

    AbstractInitializator::Info info;
    bool successFlag;
    tie(successFlag, info) = initializator->compute(camera, first_image_points,
                                                    camera, second_image_points,
                                                    camera, third_image_points, false);
    if (!successFlag)
        return false;

   {
        double nScale = third_t.norm() / info.thirdTransfrom.col(3).norm();

        info.secondTransfrom.col(3) *= nScale;
        info.thirdTransfrom.col(3) *= nScale;
        for (Vector3d & point : info.points)
            point *= nScale;
    }

    if (!compare(info.firstTransfrom.block<3, 3>(0, 0), first_R))
        return false;
    if (!compare(info.firstTransfrom.col(3), first_t))
        return false;
    if (!compare(info.secondTransfrom.block<3, 3>(0, 0), second_R))
        return false;
    if (!compare(info.secondTransfrom.col(3), second_t))
        return false;
    if (!compare(info.thirdTransfrom.block<3, 3>(0, 0), third_R))
        return false;
    if (!compare(info.thirdTransfrom.col(3), third_t))
        return false;

    for (size_t i = 0; i < real_points.size(); ++i)
    {
        if (!compare(real_points[i], info.points[i]))
            return false;
    }

    return true;
}

/*bool test_new_decomposition()
{
    Matrix3d real_R = generateRandomRotationMatrix();
    Vector3d real_t = generateRandomVector3d(3.0);
    Vector3d real_n = generateRandomVector3d(1.0);

    Matrix3d H = real_R + real_t * real_n.transpose();
    JacobiSVD<Matrix3d> svd(H, ComputeFullU | ComputeFullV | ComputeEigenvectors);

    Matrix3d W = svd.singularValues().asDiagonal();
    Matrix3d Wsq = W * W.transpose();
    Matrix3d E = W - Matrix3d::Identity();
    Matrix3d Esq = E * E.transpose();

    double z1 = - E(1, 1) / E(0, 0);
    double z2 = - E(2, 2) / E(0, 0);

    double a = Wsq(1, 1) * Wsq(2, 2) * z1 * z2 + Wsq(0, 0) * Wsq(2, 2) * z2 + Wsq(0, 0) * Wsq(1, 1) * z1;
    double b = 2.0 * E(1, 1) * Wsq(0, 0) * Wsq(2, 2) * z2 +
               2.0 * E(2, 2) * Wsq(0, 0) * Wsq(1, 1) * z1 +
               2.0 * E(0, 0) * Wsq(1, 1) * Wsq(2, 2) * z1 * z2 -
               4.0 * Wsq(0, 0) * Wsq(1, 1) * Wsq(2, 2) * z1 * z2;
    double c = Esq(0, 0) * Wsq(1, 1) * Wsq(2, 2) * z1 * z2 +
               Esq(1, 1) * Wsq(0, 0) * Wsq(2, 2) * z2 +
               Esq(2, 2) * Wsq(0, 0) * Wsq(1, 1) * z1;
    double sqrtD = sqrt(b * b - 4.0 * a * c);
    double x[2] = { (- b - sqrtD) / (2.0 * a), (- b + sqrtD) / (2.0 * a) };

    const double signs[2] = { -1.0, 1.0 };

    sonar::bearingVectors_t ts, ns;
    vector<Matrix3d, aligned_allocator<Matrix3d>> Rs;
    for (size_t i = 0; i < 2; ++i)
    {
        for (size_t j = 0; j < 2; ++j)
        {
            for (size_t k = 0; k < 2; ++k)
            {
                for (size_t ii = 0; ii < 2; ++ii)
                {
                    double tw_x = x[i] * signs[j];
                    Vector3d tw(tw_x, sqrt(tw_x * tw_x * z1) * signs[k], sqrt(tw_x * tw_x * z2) * signs[ii]);
                    Vector3d nw((E(0, 0) + tw.x() * tw.x()) / (2.0 * tw.x() * W(0, 0)),
                                (E(1, 1) + tw.y() * tw.y()) / (2.0 * tw.y() * W(1, 1)),
                                (E(2, 2) + tw.z() * tw.z()) / (2.0 * tw.z() * W(2, 2)));
                    Matrix3d Rw = W - tw * nw.transpose();
                    Vector3d t = svd.matrixU() * tw;
                    Vector3d n = svd.matrixV() * nw;
                    Matrix3d R = svd.matrixU() * Rw * svd.matrixV().transpose();
                    ts.push_back(t);
                    ns.push_back(n);
                    Rs.push_back(R);
                }
            }
        }
    }

    return true;
};*/

tuple<double, double> solveSqrEquation(double a, double b, double c)
{
    double D = b * b - 4.0 * a * c;
    assert(D > 0.0);
    double sqrtD = sqrt(D);
    double a2 = 2.0 * a;
    return make_tuple((- b - sqrtD) / a2, (- b + sqrtD) / a2);
}

bool test_new_decomposition()
{
    srand(system_clock::now().time_since_epoch().count());

    Matrix3d real_R = generateRandomRotationMatrix();
    Vector3d real_t = generateRandomVector3d(3.0);
    Vector3d real_n = generateRandomVector3d(1.0);

    Matrix3d H0 = real_R + real_t * real_n.transpose();
    Matrix3d rawH = H0 * (1.0 / 3.5);

    auto normalizedHomography = [] (const Matrix3d & H)
    {
        JacobiSVD<Matrix3d> svd(H, ComputeEigenvectors);
        double scale = 1.0 / svd.singularValues()[1];
        return H * scale;
    };
    Matrix3d H = normalizedHomography(rawH);
    Matrix3d Hinv = H.inverse();

    Matrix3d W = H * H.transpose() - Matrix3d::Identity();
    Matrix3d H_inv = H.inverse();
    Matrix3d S = H_inv.transpose() * H_inv;

    double roots_z1[2], roots_z2[2];
    tie(roots_z1[0], roots_z1[1]) = solveSqrEquation(W(0, 0), - 2.0 * W(0, 1), W(1, 1));
    tie(roots_z2[0], roots_z2[1]) = solveSqrEquation(W(0, 0), - 2.0 * W(0, 2), W(2, 2));

    const double signs[2] = { -1.0, 1.0 };

    sonar::bearingVectors_t ts, ns;
    vector<Matrix3d, aligned_allocator<Matrix3d>> Rs;
    for (size_t i = 0; i < 2; ++i)
    {
        const double & z1 = roots_z1[i];
        double z1_2 = z1 * z1;
        double z1_3 = z1_2 * z1;
        double z1_4 = z1_3 * z1;
        for (size_t j = 0; j < 2; ++j)
        {
            const double & z2 = roots_z2[j];
            double z2_2 = z2 * z2;
            double z2_3 = z2_2 * z2;
            double z2_4 = z2_3 * z2;
            Vector3d n_ = H * real_n;
            Matrix3d W_ = n_ * real_t.transpose() + real_t * n_.transpose() - real_t * real_t.transpose();
            double s1 = n_.transpose() * S * n_;
            double n_x = (W(0, 0) + pow(real_t.x(), 2.0)) / (2.0 * real_t.x());
            double n_y = (W(1, 1) + pow(real_t.x(), 2.0) * z1_2) / (2.0 * real_t.x() * z1);
            double n_z = (W(2, 2) + pow(real_t.x(), 2.0) * z2_2) / (2.0 * real_t.x() * z2);
            double s2 = n_x * n_x * S(0, 0) + n_x * n_y * S(0, 1) * 2.0 + n_x * n_z * S(0, 2) * 2.0 +
                    n_y * n_y * S(1, 1) + n_y * n_z * S(1, 2) * 2.0 + n_z * n_z * S(2, 2);

            double s = S(0, 0) * pow((W(0, 0) + pow(real_t.x(), 2.0)) / (2.0 * real_t.x()), 2.0) +
                    2.0 * S(0, 1) * ((W(0, 0) + pow(real_t.x(), 2.0)) / (2.0 * real_t.x())) *
                    ((W(1, 1) + pow(real_t.x(), 2.0) * z1_2) / (2.0 * real_t.x() * z1)) +
                    2.0 * S(0, 2) * ((W(0, 0) + pow(real_t.x(), 2.0)) / (2.0 * real_t.x())) *
                    ((W(2, 2) + pow(real_t.x(), 2.0) * z2_2) / (2.0 * real_t.x() * z2)) +
                    S(1, 1) * pow((W(1, 1) + pow(real_t.x(), 2.0) * z1_2) / (2.0 * real_t.x() * z1), 2.0) +
                    2.0 * S(1, 2) * ((W(1, 1) + pow(real_t.x(), 2.0) * z1_2) / (2.0 * real_t.x() * z1)) *
                    ((W(2, 2) + pow(real_t.x(), 2.0) * z2_2) / (2.0 * real_t.x() * z2)) +
                    S(2, 2) * pow((W(2, 2) + pow(real_t.x(), 2.0) * z2_2) / (2.0 * real_t.x() * z2), 2.0);
            double sA = 4.0 * z1_2 * z2_2 * pow(real_t.x(), 2.0);
            double sB = S(0, 0) * z1_2 * z2_2 * W(0, 0) * W(0, 0) +
                    S(0, 0) * z1_2 * z2_2 * 2.0 * W(0, 0) * pow(real_t.x(), 2.0) +
                    S(0, 0) * z1_2 * z2_2 * pow(real_t.x(), 4.0) +
                    2.0 * S(0, 1) * z1 * z2_2 * W(0, 0) * W(1, 1) +
                    2.0 * S(0, 1) * z1_3 * z2_2 * W(0, 0) * pow(real_t.x(), 2.0) +
                    2.0 * S(0, 1) * z1 * z2_2 * W(1, 1) * pow(real_t.x(), 2.0) +
                    2.0 * S(0, 1) * z1_3 * z2_2 * pow(real_t.x(), 4.0) +
                    2.0 * S(0, 2) * z1_2 * z2 * W(0, 0) * W(2, 2) +
                    2.0 * S(0, 2) * z1_2 * z2_3 * W(0, 0) * pow(real_t.x(), 2.0) +
                    2.0 * S(0, 2) * z1_2 * z2 * W(2, 2) * pow(real_t.x(), 2.0) +
                    2.0 * S(0, 2) * z1_2 * z2_3 * pow(real_t.x(), 4.0) +
                    S(1, 1) * z2_2 * W(1, 1) * W(1, 1) +
                    2.0 * S(1, 1) * z2_2 * W(1, 1) * z1_2 * pow(real_t.x(), 2.0) +
                    S(1, 1) * z1_4 * z2_2 * pow(real_t.x(), 4.0) +
                    2.0 * S(1, 2) * z1 * z2 * W(1, 1) * W(2, 2) +
                    2.0 * S(1, 2) * z1 * z2_3 * W(1, 1) * pow(real_t.x(), 2.0) +
                    2.0 * S(1, 2) * z1_3 * z2 * W(2, 2) * pow(real_t.x(), 2.0) +
                    2.0 * S(1, 2) * z1_3 * z2_3 * pow(real_t.x(), 4.0) +
                    S(2, 2) * z1_2 * W(2, 2) * W(2, 2) +
                    2.0 * S(2, 2) * z1_2 * z2_2 * W(2, 2) * pow(real_t.x(), 2.0) +
                    S(2, 2) * z1_2 * z2_4 * pow(real_t.x(), 4.0);

            double a = S(0, 0) * z1_2 * z2_2 +
                    2.0 * S(0, 1) * z1_3 * z2_2 +
                    2.0 * S(0, 2) * z1_2 * z2_3 +
                    S(1, 1) * z1_4 * z2_2 +
                    2.0 * S(1, 2) * z1_3 * z2_3 +
                    S(2, 2) * z1_2 * z2_4;
            double b = - 4.0 * z1_2 * z2_2 +
                    2.0 * S(0, 0) * z1_2 * z2_2 * W(0, 0) +
                    2.0 * S(0, 1) * z1_3 * z2_2 * W(0, 0) +
                    2.0 * S(0, 1) * z1 * z2_2 * W(1, 1) +
                    2.0 * S(0, 2) * z1_2 * z2_3 * W(0, 0) +
                    2.0 * S(0, 2) * z1_2 * z2 * W(2, 2) +
                    2.0 * S(1, 1) * z2_2 * z1_2 * W(1, 1) +
                    2.0 * S(1, 2) * z1 * z2_3 * W(1, 1) +
                    2.0 * S(1, 2) * z1_3 * z2 * W(2, 2) +
                    2.0 * S(2, 2) * z1_2 * z2_2 * W(2, 2);
            double c = S(0, 0) * z1_2 * z2_2 * W(0, 0) * W(0, 0) +
                    2.0 * S(0, 1) * z1 * z2_2 * W(0, 0) * W(1, 1) +
                    2.0 * S(0, 2) * z1_2 * z2 * W(0, 0) * W(2, 2) +
                    S(1, 1) * z2_2 * W(1, 1) * W(1, 1) +
                    2.0 * S(1, 2) * z1 * z2 * W(1, 1) * W(2, 2) +
                    S(2, 2) * z1_2 * W(2, 2) * W(2, 2);
            //double e = a * pow(real_t.x(), 4.0) + b * pow(real_t.x(), 2.0) + c;
            //qDebug() << e;
            double roots_sqr_t_x[2];
            tie(roots_sqr_t_x[0], roots_sqr_t_x[1]) = solveSqrEquation(a, b, c);
            for (size_t ii = 0; ii < 2; ++ii)
            {
                double root_t_x = sqrt(roots_sqr_t_x[ii]);
                for (size_t jj = 0; jj < 2; ++jj)
                {
                    double t_x = root_t_x * signs[jj];
                    Vector3d t(t_x, t_x * z1, t_x * z2);
                    Vector3d n = Hinv * Vector3d((W(0, 0) + t.x() * t.x()) / (2.0 * t.x()),
                                                 (W(1, 1) + t.y() * t.y()) / (2.0 * t.y()),
                                                 (W(2, 2) + t.z() * t.z()) / (2.0 * t.z()));
                    Matrix3d R = H - t * n.transpose();
                    double length_n = n.norm();
                    double dR = R.determinant();

                    qDebug() << length_n << dR;

                    ts.push_back(t);
                }
            }
        }
    }

    return true;
};
