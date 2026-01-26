// Copyright (C) 2024 Jan Michalczyk, Control of Networked Systems, University
// of Klagenfurt, Austria.
//
// All rights reserved.
//
// This software is licensed under the terms of the BSD-2-Clause-License with
// no commercial use allowed, the full terms of which are made available
// in the LICENSE file. No license in patents is granted.
//
// You can contact the author at <jan.michalczyk@aau.at>

#ifndef _UTIL_H_
#define _UTIL_H_

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <gtsam/base/types.h>
#include <gtsam/geometry/Quaternion.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <initializer_list>
#include <vector>

#include "aaucns_rio/debug.h"
#include "aaucns_rio/pcl_conversions_rio.h"
#include "aaucns_rio/trailpoint.h"

namespace aaucns_rio
{
namespace util
{
inline void fixEigenvalues(Eigen::MatrixXd& cov_mat)
{
    // TODO(jan m): optiimize this function.
    Eigen::MatrixXd partial_cov_mat = cov_mat;
    int zero_index = 0;
    for (int i = 0; i < partial_cov_mat.diagonal().size(); ++i)
    {
        if (partial_cov_mat.diagonal()(i) == 0)
        {
            zero_index = i;
            break;
        }
    }
    if (zero_index)
    {
        partial_cov_mat = cov_mat.block(0, 0, zero_index, zero_index);
    }

    Eigen::EigenSolver<Eigen::MatrixXd> vd(partial_cov_mat);

    Eigen::EigenSolver<Eigen::MatrixXd>::EigenvectorsType V(vd.eigenvectors());
    Eigen::EigenSolver<Eigen::MatrixXd>::EigenvalueType D(vd.eigenvalues());

    if (!(V.imag().isZero() && D.imag().isZero()))
    {
        std::cerr
            << "Warning: Eigenvalue decomposition has imaginary components"
            << std::endl;
    }
    Eigen::MatrixXd V_real(V.real());
    Eigen::VectorXd D_real(D.real());
    // determine if the matrix is already positive-semi-definite
    bool no_negative_eigenvalues = true;
    for (int k = 0; k < D_real.size(); k++)
    {
        if (D_real[k] < 0)
        {
            no_negative_eigenvalues = false;
        }
    }

    if (no_negative_eigenvalues)
    {
        return;
    }

    Eigen::VectorXd D_corrected(D_real);

    for (int k = 0; k < D_corrected.size(); k++)
    {
        if (D_corrected[k] < 0)
        {
            D_corrected[k] = std::abs(D_corrected[k]);
        }
    }

    partial_cov_mat = V_real * D_corrected.asDiagonal() * V_real.inverse();
    if (zero_index)
    {
        cov_mat.block(0, 0, zero_index, zero_index) = partial_cov_mat;
    }
    else
    {
        cov_mat = partial_cov_mat;
    }
}

inline pcl::PointCloud<RadarPointCloudType> applyNearRangeFiltering(
    const pcl::PointCloud<RadarPointCloudType>& current_pc2,
    const double threshold_in_meters_x, const double threshold_in_meters_y)
{
    pcl::PointCloud<RadarPointCloudType> filtered_current_pc2 = current_pc2;
    for (std::size_t i = 0; i < filtered_current_pc2.width; ++i)
    {
        if ((filtered_current_pc2.points[i].x < threshold_in_meters_x) &&
            (filtered_current_pc2.points[i].y < threshold_in_meters_y) &&
            (filtered_current_pc2.points[i].y > -threshold_in_meters_y))
        {
            filtered_current_pc2.points[i].intensity = 0;
        }
    }
    return filtered_current_pc2;
}
inline std::vector<TrailPoint> convertPC2ToTrailPoints(
    const pcl::PointCloud<RadarPointCloudType>& pc2)
{
    std::vector<TrailPoint> trailpoints;
    for (int i = 0; i < pc2.width; ++i)
    {
        TrailPoint trailpoint;
        trailpoint.most_recent_coordinates << pc2.points[i].x, pc2.points[i].y,
            pc2.points[i].z;
        trailpoint.intensity = pc2.points[i].intensity;
        trailpoint.is_active = false;
        trailpoint.is_persistent = false;
        trailpoints.push_back(trailpoint);
    }
    return trailpoints;
}

inline Eigen::MatrixXd convertPC2ToEigenMatrix(
    const pcl::PointCloud<RadarPointCloudType>& pc2)
{
    Eigen::MatrixXd pc2_as_matrix = Eigen::MatrixXd::Zero(pc2.width, 3);
    for (int i = 0; i < pc2_as_matrix.rows(); ++i)
    {
        pc2_as_matrix.row(i) << pc2.points[i].x, pc2.points[i].y,
            pc2.points[i].z;
    }
    return pc2_as_matrix;
}

// Get 3D cross product skew symmetric matrix of a given 3D vector.
template <class Derived>
Eigen::Matrix<typename Derived::Scalar, 3, 3> getSkewSymmetricMat(
    const Eigen::MatrixBase<Derived>& vec)
{
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, 3);
    return (Eigen::Matrix<typename Derived::Scalar, 3, 3>() << 0.0, -vec[2],
            vec[1], vec[2], 0.0, -vec[0], -vec[1], vec[0], 0.0)
        .finished();
}

template <typename t>
std::vector<std::vector<double>> convertEigenMatrixTo2DStdVector(
    const Eigen::MatrixBase<t>& matrix)
{
    std::vector<std::vector<double>> result;
    for (int i = 0; i < matrix.rows(); ++i)
    {
        std::vector<double> v;
        for (int j = 0; j < matrix.cols(); ++j)
        {
            v.push_back(matrix(i, j));
        }
        result.push_back(v);
    }
    return result;
}

template <typename t>
std::vector<double> convertEigenMatrixTo1DStdVector(
    const Eigen::MatrixBase<t>& matrix)
{
    std::vector<double> result;
    for (int i = 0; i < matrix.rows(); ++i)
    {
        std::vector<double> v;
        for (int j = 0; j < matrix.cols(); ++j)
        {
            result.push_back(matrix(i, j));
        }
    }
    return result;
}

// Copies gtsam quaternion to geometry_msgs/quaternion.
inline void gtsamQuaternionToMsg(const gtsam::Quaternion& q_in,
                                 geometry_msgs::Quaternion& q_out)
{
    q_out.w = q_in.w();
    q_out.x = q_in.x();
    q_out.y = q_in.y();
    q_out.z = q_in.z();
}

// Copies eigen quaternion to geometry_msgs/quaternion.
template <typename Scalar>
void quaternionToMsg(const Eigen::Quaternion<Scalar>& q_in,
                     geometry_msgs::Quaternion& q_out)
{
    q_out.w = q_in.w();
    q_out.x = q_in.x();
    q_out.y = q_in.y();
    q_out.z = q_in.z();
}

// Copies eigen quaternion to geometry_msgs/quaternion.
template <typename Scalar>
geometry_msgs::Quaternion quaternionToMsg(const Eigen::Quaternion<Scalar>& q_in)
{
    geometry_msgs::Quaternion q_out;
    quaternionToMsg(q_in, q_out);
    return q_out;
}

// Copies an eigen 3d vector to a 3d Point struct. point has to have members
// x,y,z.
template <typename Derived, typename TPointType>
void vector3dToPoint(const Eigen::MatrixBase<Derived>& vec, TPointType& point)
{
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, 3);
    point.x = vec[0];
    point.y = vec[1];
    point.z = vec[2];
}

// Copies an eigen 3d vector to a 3d Point struct. point has to have members
// x,y,z.
template <typename Derived, typename TPointType>
TPointType vector3dToPoint(const Eigen::MatrixBase<Derived>& vec)
{
    TPointType point;
    vector3dToPoint(vec, point);
    return point;
}

template <typename TPointType>
Eigen::Vector3d pointTo3dEigenVector(const TPointType& point)
{
    Eigen::Vector3d vector;
    vector << point.x, point.y, point.z;
    return vector;
}

inline Eigen::Matrix3d quaternionToRotationMat(
    const Eigen::Quaternion<double>& q)
{
    // Based on Sola's paper page 25, formula 115.
    Eigen::Matrix3d rotation;
    rotation << q.w() * q.w() + q.x() * q.x() - q.y() * q.y() - q.z() * q.z(),
        2 * (q.x() * q.y() - q.w() * q.z()),
        2 * (q.x() * q.z() + q.w() * q.y()),
        2 * (q.x() * q.y() + q.w() * q.z()),
        q.w() * q.w() - q.x() * q.x() + q.y() * q.y() - q.z() * q.z(),
        2 * (q.y() * q.z() - q.w() * q.x()),
        2 * (q.x() * q.z() - q.w() * q.y()),
        2 * (q.y() * q.z() + q.w() * q.x()),
        q.w() * q.w() - q.x() * q.x() - q.y() * q.y() + q.z() * q.z();
    return rotation;
}

// Computes a quaternion from the 3-element small angle approximation theta.
template <typename Derived>
Eigen::Quaternion<typename Derived::Scalar> quaternionFromSmallAngle(
    const Eigen::MatrixBase<Derived>& theta)
{
    typedef typename Derived::Scalar Scalar;
    EIGEN_STATIC_ASSERT_FIXED_SIZE(Derived);
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, 3);
    const Scalar q_squared = theta.squaredNorm() / 4.0;

    if (q_squared < 1)
    {
        return Eigen::Quaternion<Scalar>(sqrt(1 - q_squared), theta[0] * 0.5,
                                         theta[1] * 0.5, theta[2] * 0.5);
    }
    else
    {
        const Scalar w = 1.0 / sqrt(1 + q_squared);
        const Scalar f = w * 0.5;
        return Eigen::Quaternion<Scalar>(w, theta[0] * f, theta[1] * f,
                                         theta[2] * f);
    }
}

// Check for misbehavior of Eigen.
template <typename t>
bool checkForNumeric(const t& vec, int size, const std::string& info)
{
    for (int i = 0; i < size; i++)
    {
        if (isnan(vec[i]))
        {
            std::cerr << "=== ERROR ===  " << info << ": NAN at index " << i
                      << std::endl;
            return false;
        }
        if (isinf(vec[i]))
        {
            std::cerr << "=== ERROR ===  " << info << ": INF at index " << i
                      << std::endl;
            return false;
        }
    }
    return true;
}

inline pcl::PointCloud<RadarPointCloudType> applyPyramidFiltering(
    const pcl::PointCloud<RadarPointCloudType>& pc2)
{
    // 60 degrees
    const Eigen::Vector3d up_elevation_limit_n(-0.7660444, 0, 0.6427876);
    const Eigen::Vector3d down_elevation_limit_n(-0.7660444, 0, -0.6427876);
    // 60 degrees
    const Eigen::Vector3d left_azimuth_limit_n(-0.7660444, 0.6427876, 0);
    const Eigen::Vector3d right_azimuth_limit_n(-0.7660444, -0.6427876, 0);

    pcl::PointCloud<RadarPointCloudType> filtered_pc2 = pc2;

    for (int i = 0; i < filtered_pc2.width; ++i)
    {
        const Eigen::Vector3d point =
            util::pointTo3dEigenVector(filtered_pc2.points[i]);
        if ((left_azimuth_limit_n.transpose() * point > 0) ||
            (right_azimuth_limit_n.transpose() * point > 0) ||
            (up_elevation_limit_n.transpose() * point > 0) ||
            (down_elevation_limit_n.transpose() * point > 0))
        {
            filtered_pc2.points[i].intensity = 0;
        }
    }

    return filtered_pc2;
}

inline pcl::PointCloud<RadarPointCloudType> applyClusteringOfPoints(
    const pcl::PointCloud<RadarPointCloudType>& pc2,
    const double distance_threshold = 1.0, const int number_threshold = 4)
{
    pcl::PointCloud<RadarPointCloudType> filtered_pc2 = pc2;
    for (int i = 0; i < filtered_pc2.width; ++i)
    {
        const Eigen::Vector3d a =
            util::pointTo3dEigenVector(filtered_pc2.points[i]);
        int n_of_neighbours = 0;
        for (int j = 0; j < filtered_pc2.width; ++j)
        {
            if (i != j)
            {
                // Compute distance between the point and every other point.
                const Eigen::Vector3d b =
                    util::pointTo3dEigenVector(filtered_pc2.points[j]);
                const double distance = (a - b).norm();
                if (distance < distance_threshold)
                {
                    ++n_of_neighbours;
                }
            }
        }
        if (n_of_neighbours < number_threshold)
        {
            filtered_pc2.points[i].intensity = 0;
        }
    }
    return filtered_pc2;
}

inline pcl::PointCloud<RadarPointCloudType> applyStdDevFiltering(
    const pcl::PointCloud<RadarPointCloudType>& pc2,
    const double distance_threshold = 1.0, const double std_dev_threshold = 0.6)
{
    pcl::PointCloud<RadarPointCloudType> filtered_pc2 = pc2;
    for (int i = 0; i < filtered_pc2.width; ++i)
    {
        const Eigen::Vector3d a =
            util::pointTo3dEigenVector(filtered_pc2.points[i]);
        int n_of_neighbours = 0;
        std::vector<double> dist_to_neighbours_of_i;
        double mean_of_dist_to_neighbours = 0;
        int denominator = 1;
        for (int j = 0; j < filtered_pc2.width; ++j)
        {
            if (i != j)
            {
                // Compute distance between the point and every other point.
                const Eigen::Vector3d b =
                    util::pointTo3dEigenVector(filtered_pc2.points[j]);
                const double distance = (a - b).norm();
                if (distance < distance_threshold)
                {
                    // Keep the distance in the vector.
                    dist_to_neighbours_of_i.push_back(distance);
                    mean_of_dist_to_neighbours += distance;
                    ++denominator;
                }
            }
        }
        mean_of_dist_to_neighbours = mean_of_dist_to_neighbours / denominator;
        // Subtract the point from the mean and comapre to the threshold.
        for (const double dist : dist_to_neighbours_of_i)
        {
            const double std_dev = std::abs(mean_of_dist_to_neighbours - dist);
            if (std_dev > std_dev_threshold)
            {
                filtered_pc2.points[i].intensity = 0;
            }
        }
    }
    return filtered_pc2;
}

template <class t_DerivedOutput, typename t_Scalar, int t_rows, int t_cols,
          int t_flags>
void concatenateMatricesVertically(
    Eigen::PlainObjectBase<t_DerivedOutput>& result,
    const std::vector<Eigen::Matrix<t_Scalar, t_rows, t_cols, t_flags>>&
        matrices)
{
    std::vector<Eigen::Matrix<t_Scalar, t_rows, t_cols, t_flags>>
        nonempty_matrices = matrices;
    int number_of_matrices = nonempty_matrices.size();

    if (number_of_matrices > 0)
    {
        std::ptrdiff_t total_number_of_rows = 0;
        std::ptrdiff_t number_of_cols = 0;

        for (int i = 0; i < number_of_matrices; ++i)
        {
            if (!nonempty_matrices[i].size())
            {
                nonempty_matrices.erase(nonempty_matrices.begin() + i);
                --number_of_matrices;
                --i;
            }
        }
        if (!number_of_matrices)
        {
            result.resize(0, 0);
            return;
        }

        number_of_cols = nonempty_matrices[0].cols();
        for (int i = 0; i < number_of_matrices; ++i)
        {
            assert(number_of_cols == nonempty_matrices[i].cols());
            total_number_of_rows += nonempty_matrices[i].rows();
        }

        result.resize(total_number_of_rows, number_of_cols);

        std::ptrdiff_t row_index = 0;
        for (int i = 0; i < number_of_matrices; ++i)
        {
            result.block(row_index, 0, nonempty_matrices[i].rows(),
                         number_of_cols) = nonempty_matrices[i];
            row_index += nonempty_matrices[i].rows();
        }
    }
    else
    {
        result.resize(0, 0);
    }
}

template <typename t_Scalar, int t_rows, int t_cols, int t_flags>
Eigen::Matrix<t_Scalar, Eigen::Dynamic, Eigen::Dynamic> makeBlockDiagonal(
    const std::vector<Eigen::Matrix<t_Scalar, t_rows, t_cols, t_flags>>&
        input_matrices)
{
    switch (input_matrices.size())
    {
        case 0:
            return (Eigen::Matrix<t_Scalar, Eigen::Dynamic, Eigen::Dynamic>());

        case 1:
            return (input_matrices[0]);

        default:
            // Initialize the output diagonal matrix
            std::ptrdiff_t row_size = 0;
            std::ptrdiff_t col_size = 0;

            for (std::size_t i = 0; i < input_matrices.size(); ++i)
            {
                row_size += input_matrices[i].rows();
                col_size += input_matrices[i].cols();
            }
            Eigen::Matrix<t_Scalar, Eigen::Dynamic, Eigen::Dynamic> output;
            output.setZero(row_size, col_size);

            // Add in the input matrices
            std::ptrdiff_t cumulative_row = 0;
            std::ptrdiff_t cumulative_col = 0;

            for (std::size_t i = 0; i < input_matrices.size(); ++i)
            {
                int current_num_rows = input_matrices[i].rows();
                int current_num_cols = input_matrices[i].cols();

                output.block(cumulative_row, cumulative_col, current_num_rows,
                             current_num_cols) = input_matrices[i];

                cumulative_row += current_num_rows;
                cumulative_col += current_num_cols;
            }
            return output;
    }
}

template <class t_Derived>
void removeRow(Eigen::PlainObjectBase<t_Derived>& matrix,
               const std::ptrdiff_t row_to_remove)
{
    assert(row_to_remove <= matrix.rows());

    std::ptrdiff_t number_of_rows = matrix.rows() - 1;
    std::ptrdiff_t number_of_cols = matrix.cols();

    if (row_to_remove < number_of_rows)
    {
        matrix.block(row_to_remove, 0, number_of_rows - row_to_remove,
                     number_of_cols) =
            matrix.block(row_to_remove + 1, 0, number_of_rows - row_to_remove,
                         number_of_cols);
    }

    matrix.conservativeResize(number_of_rows, number_of_cols);
}

template <class t_Derived>
void removeColumn(Eigen::PlainObjectBase<t_Derived>& matrix,
                  const std::ptrdiff_t column_to_remove)
{
    assert(column_to_remove <= matrix.rows());

    std::ptrdiff_t number_of_rows = matrix.rows();
    std::ptrdiff_t number_of_cols = matrix.cols() - 1;

    if (column_to_remove < number_of_cols)
    {
        matrix.block(0, column_to_remove, number_of_rows,
                     number_of_cols - column_to_remove) =
            matrix.block(0, column_to_remove + 1, number_of_rows,
                         number_of_cols - column_to_remove);
    }

    matrix.conservativeResize(number_of_rows, number_of_cols);
}

template <typename GTSAM_KEY_VECTOR, typename GTSAM_KEY_TYPE>
bool areAllKeysIn(const GTSAM_KEY_VECTOR& keys,
                  const std::vector<GTSAM_KEY_TYPE>& keys_to_find)
{
    return std::all_of(
        keys_to_find.begin(), keys_to_find.end(), [&keys](const auto& key) {
            return std::find(keys.begin(), keys.end(), key) != keys.end();
        });
}

}  // namespace util
}  // namespace aaucns_rio

#endif /* _UTIL_H_ */
