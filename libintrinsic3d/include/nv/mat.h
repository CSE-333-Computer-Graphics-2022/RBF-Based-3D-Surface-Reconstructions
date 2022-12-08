

#pragma once


#ifndef WIN64
	#define EIGEN_DONT_ALIGN_STATICALLY
#endif
#include <Eigen/Core>
#include <Eigen/Dense>

#include <functional>


/**
 * @brief   Basic vector and matrix data types
 *          and some related helper functions
 * @author  Robert Maier <robert.maier@tum.de>
 */
namespace nv
{
	typedef Eigen::Vector2d Vec2;
	typedef Eigen::Vector3d Vec3;
	typedef Eigen::Vector4d Vec4;
	typedef Eigen::Matrix<double, 5, 1> Vec5;
	typedef Eigen::Matrix<double, 6, 1> Vec6;
	typedef Eigen::Matrix2d Mat2;
	typedef Eigen::Matrix3d Mat3;
	typedef Eigen::Matrix4d Mat4;
	typedef Eigen::Matrix<double, 6, 6> Mat6;

	typedef Eigen::Vector2f Vec2f;
	typedef Eigen::Vector3f Vec3f;
	typedef Eigen::Vector4f Vec4f;
	typedef Eigen::Matrix<float, 5, 1> Vec5f;
	typedef Eigen::Matrix<float, 6, 1> Vec6f;
	typedef Eigen::Matrix2f Mat2f;
	typedef Eigen::Matrix3f Mat3f;
	typedef Eigen::Matrix4f Mat4f;
	typedef Eigen::Matrix<float, 6, 6> Mat6f;

	typedef Eigen::Vector2i Vec2i;
	typedef Eigen::Vector3i Vec3i;
	typedef Eigen::Vector4i Vec4i;
	typedef Eigen::Matrix<int, 6, 1> Vec6i;
	typedef Eigen::Matrix2i Mat2i;
	typedef Eigen::Matrix3i Mat3i;
	typedef Eigen::Matrix4i Mat4i;
	typedef Eigen::Matrix<int, 6, 6> Mat6i;

	typedef Eigen::Matrix<unsigned char, 2, 1> Vec2b;
	typedef Eigen::Matrix<unsigned char, 3, 1> Vec3b;
	typedef Eigen::Matrix<unsigned char, 4, 1> Vec4b;
	typedef Eigen::Matrix<unsigned char, 2, 2> Mat2b;
	typedef Eigen::Matrix<unsigned char, 3, 3> Mat3b;
	typedef Eigen::Matrix<unsigned char, 4, 4> Mat4b;


	inline Vec2i round(const Vec2f &v) { return (v + Vec2f::Constant(0.5f)).cast<int>(); }
	inline Vec2i round(const Vec2 &v) { return (v + Vec2::Constant(0.5)).cast<int>(); }
	inline Vec3i round(const Vec3f &v) { return (v + Vec3f::Constant(0.5f)).cast<int>(); }
	inline Vec3i round(const Vec3 &v) { return (v + Vec3::Constant(0.5)).cast<int>(); }
	inline Vec4i round(const Vec4f &v) { return (v + Vec4f::Constant(0.5f)).cast<int>(); }
	inline Vec4i round(const Vec4 &v) { return (v + Vec4::Constant(0.5)).cast<int>(); }

    inline Vec2i floor(const Vec2f &v) { return Vec2i((int)std::floor(v[0]), (int)std::floor(v[1])); }
	inline Vec2i floor(const Vec2 &v) { return Vec2i((int)std::floor(v[0]), (int)std::floor(v[1])); }
    inline Vec3i floor(const Vec3f &v) { return Vec3i((int)std::floor(v[0]), (int)std::floor(v[1]), (int)std::floor(v[2])); }
	inline Vec3i floor(const Vec3 &v) { return Vec3i((int)std::floor(v[0]), (int)std::floor(v[1]), (int)std::floor(v[2])); }
    inline Vec4i floor(const Vec4f &v) { return Vec4i((int)std::floor(v[0]), (int)std::floor(v[1]), (int)std::floor(v[2]), (int)std::floor(v[3])); }
	inline Vec4i floor(const Vec4 &v) { return Vec4i((int)std::floor(v[0]), (int)std::floor(v[1]), (int)std::floor(v[2]), (int)std::floor(v[3])); }

    inline Vec2i ceil(const Vec2f &v) { return Vec2i((int)std::ceil(v[0]), (int)std::ceil(v[1])); }
	inline Vec2i ceil(const Vec2 &v) { return Vec2i((int)std::ceil(v[0]), (int)std::ceil(v[1])); }
    inline Vec3i ceil(const Vec3f &v) { return Vec3i((int)std::ceil(v[0]), (int)std::ceil(v[1]), (int)std::ceil(v[2])); }
	inline Vec3i ceil(const Vec3 &v) { return Vec3i((int)std::ceil(v[0]), (int)std::ceil(v[1]), (int)std::ceil(v[2])); }
    inline Vec4i ceil(const Vec4f &v) { return Vec4i((int)std::ceil(v[0]), (int)std::ceil(v[1]), (int)std::ceil(v[2]), (int)std::ceil(v[3])); }
	inline Vec4i ceil(const Vec4 &v) { return Vec4i((int)std::ceil(v[0]), (int)std::ceil(v[1]), (int)std::ceil(v[2]), (int)std::ceil(v[3])); }

} 


namespace std
{
	template <>
	struct hash<nv::Vec3i> : private std::unary_function<nv::Vec3i, size_t>
	{
		size_t operator()(const nv::Vec3i& v) const
		{
			const size_t p0 = 73856093;
			const size_t p1 = 19349669;
			const size_t p2 = 83492791;
			const size_t res = ((size_t)v[0] * p0) ^ ((size_t)v[1] * p1) ^ ((size_t)v[2] * p2);
			return res;
		}
	};

} // namespace std
