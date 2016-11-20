///////////////////////////////////////////////////////////////////////////////////
/// OpenGL Mathematics (glm.g-truc.net)
///
/// Copyright (c) 2005 - 2012 G-Truc Creation (www.g-truc.net)
/// Permission is hereby granted, free of charge, to any person obtaining a copy
/// of this software and associated documentation files (the "Software"), to deal
/// in the Software without restriction, including without limitation the rights
/// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
/// copies of the Software, and to permit persons to whom the Software is
/// furnished to do so, subject to the following conditions:
/// 
/// The above copyright notice and this permission notice shall be included in
/// all copies or substantial portions of the Software.
/// 
/// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
/// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
/// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
/// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
/// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
/// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
/// THE SOFTWARE.
///
/// @ref gtx_simd_vec4
/// @file glm/gtx/simd_vec4.hpp
/// @date 2009-05-07 / 2011-06-07
/// @author Christophe Riccio
///
/// @see core (dependence)
///
/// @defgroup gtx_simd_vec4 GLM_GTX_simd_vec4: SIMD vec4 type and functions
/// @ingroup gtx
/// 
/// @brief SIMD implementation of vec4 type.
/// 
/// <glm/gtx/simd_vec4.hpp> need to be included to use these functionalities.
///////////////////////////////////////////////////////////////////////////////////

#ifndef GLM_GTX_simd_vec4
#define GLM_GTX_simd_vec4 GLM_VERSION

// Dependency:
#include "../glm.hpp"

#if(GLM_ARCH != GLM_ARCH_PURE)

#if(GLM_ARCH & GLM_ARCH_SSE2)
#	include "../core/intrinsic_common.hpp"
#	include "../core/intrinsic_geometric.hpp"
#else
#	error "GLM: GLM_GTX_simd_vec4 requires compiler support of SSE2 through intrinsics"
#endif

#if(defined(GLM_MESSAGES) && !defined(glm_ext))
#	pragma message("GLM: GLM_GTX_simd_vec4 extension included")
#endif

namespace glm{
namespace detail
{
	/// 4-dimensional vector implemented using SIMD SEE intrinsics.
	/// \ingroup gtx_simd_vec4
	GLM_ALIGNED_STRUCT(16) fvec4SIMD
	{
		enum ctor{null};
		typedef __m128 value_type;
		typedef std::size_t size_type;
		static size_type value_size();

		typedef fvec4SIMD type;
		typedef tvec4<bool> bool_type;

		__m128 Data;

		//////////////////////////////////////
		// Implicit basic constructors

		fvec4SIMD();
		fvec4SIMD(__m128 const & Data);
		fvec4SIMD(fvec4SIMD const & v);

		//////////////////////////////////////
		// Explicit basic constructors

		explicit fvec4SIMD(
			ctor);
		explicit fvec4SIMD(
			float const & s);
		explicit fvec4SIMD(
			float const & x, 
			float const & y, 
			float const & z, 
			float const & w);
		explicit fvec4SIMD(
			tvec4<float> const & v);

		////////////////////////////////////////
		//// Convertion vector constructors

		fvec4SIMD(vec2 const & v, float const & s1, float const & s2);
		fvec4SIMD(float const & s1, vec2 const & v, float const & s2);
		fvec4SIMD(float const & s1, float const & s2, vec2 const & v);
		fvec4SIMD(vec3 const & v, float const & s);
		fvec4SIMD(float const & s, vec3 const & v);
		fvec4SIMD(vec2 const & v1, vec2 const & v2);
		//fvec4SIMD(ivec4SIMD const & v);

		//////////////////////////////////////
		// Unary arithmetic operators

		fvec4SIMD& operator= (fvec4SIMD const & v);
		fvec4SIMD& operator+=(fvec4SIMD const & v);
		fvec4SIMD& operator-=(fvec4SIMD const & v);
		fvec4SIMD& operator*=(fvec4SIMD const & v);
		fvec4SIMD& operator/=(fvec4SIMD const & v);

		fvec4SIMD& operator+=(float const & s);
		fvec4SIMD& operator-=(float const & s);
		fvec4SIMD& operator*=(float const & s);
		fvec4SIMD& operator/=(float const & s);

		fvec4SIMD& operator++();
		fvec4SIMD& operator--();

		//////////////////////////////////////
		// Swizzle operators

		template <comp X, comp Y, comp Z, comp W>
		fvec4SIMD& swizzle();
		template <comp X, comp Y, comp Z, comp W>
		fvec4SIMD swizzle() const;
		template <comp X, comp Y, comp Z>
		fvec4SIMD swizzle() const;
		template <comp X, comp Y>
		fvec4SIMD swizzle() const;
		template <comp X>
		fvec4SIMD swizzle() const;
	};
}//namespace detail

	typedef glm::detail::fvec4SIMD simdVec4;

	/// @addtogroup gtx_simd_vec4
	/// @{

	//! Convert a simdVec4 to a vec4.
	//! (From GLM_GTX_simd_vec4 extension)
	detail::tvec4<float> vec4_cast(
		detail::fvec4SIMD const & x);

	//! Returns x if x >= 0; otherwise, it returns -x. 
	//! (From GLM_GTX_simd_vec4 extension, common function)
	detail::fvec4SIMD abs(detail::fvec4SIMD const & x);

	//! Returns 1.0 if x > 0, 0.0 if x = 0, or -1.0 if x < 0. 
	//! (From GLM_GTX_simd_vec4 extension, common function)
	detail::fvec4SIMD sign(detail::fvec4SIMD const & x);

	//! Returns a value equal to the nearest integer that is less then or equal to x. 
	//! (From GLM_GTX_simd_vec4 extension, common function)
	detail::fvec4SIMD floor(detail::fvec4SIMD const & x);

	//! Returns a value equal to the nearest integer to x 
	//! whose absolute value is not larger than the absolute value of x. 
	//! (From GLM_GTX_simd_vec4 extension, common function)
	detail::fvec4SIMD trunc(detail::fvec4SIMD const & x);

	//! Returns a value equal to the nearest integer to x. 
	//! The fraction 0.5 will round in a direction chosen by the 
	//! implementation, presumably the direction that is fastest. 
	//! This includes the possibility that round(x) returns the 
	//! same value as roundEven(x) for all values of x. 
	//! (From GLM_GTX_simd_vec4 extension, common function)
	detail::fvec4SIMD round(detail::fvec4SIMD const & x);

	//! Returns a value equal to the nearest integer to x.
	//! A fractional part of 0.5 will round toward the nearest even
	//! integer. (Both 3.5 and 4.5 for x will return 4.0.) 
	//! (From GLM_GTX_simd_vec4 extension, common function)
	//detail::fvec4SIMD roundEven(detail::fvec4SIMD const & x);

	//! Returns a value equal to the nearest integer 
	//! that is greater than or equal to x. 
	//! (From GLM_GTX_simd_vec4 extension, common function)
	detail::fvec4SIMD ceil(detail::fvec4SIMD const & x);

	//! Return x - floor(x).
	//! (From GLM_GTX_simd_vec4 extension, common function)
	detail::fvec4SIMD fract(detail::fvec4SIMD const & x);

	//! Modulus. Returns x - y * floor(x / y) 
	//! for each component in x using the floating point value y.
	//! (From GLM_GTX_simd_vec4 extension, common function)
	detail::fvec4SIMD mod(
		detail::fvec4SIMD const & x, 
		detail::fvec4SIMD const & y);

	//! Modulus. Returns x - y * floor(x / y) 
	//! for each component in x using the floating point value y.
	//! (From GLM_GTX_simd_vec4 extension, common function)
	detail::fvec4SIMD mod(
		detail::fvec4SIMD const & x, 
		float const & y);

	//! Returns the fractional part of x and sets i to the integer
	//! part (as a whole number floating point value). Both the
	//! return value and the output parameter will have the same
	//! sign as x.
	//! (From GLM_GTX_simd_vec4 extension, common function)
	//detail::fvec4SIMD modf(
	//	detail::fvec4SIMD const & x, 
	//	detail::fvec4SIMD & i);

	//! Returns y if y < x; otherwise, it returns x.
	//! (From GLM_GTX_simd_vec4 extension, common function)
	detail::fvec4SIMD min(
		detail::fvec4SIMD const & x, 
		detail::fvec4SIMD const & y);

	detail::fvec4SIMD min(
		detail::fvec4SIMD const & x, 
		float const & y);

	//! Returns y if x < y; otherwise, it returns x.
	//! (From GLM_GTX_simd_vec4 extension, common function)
	detail::fvec4SIMD max(
		detail::fvec4SIMD const & x, 
		detail::fvec4SIMD const & y);

	detail::fvec4SIMD max(
		detail::fvec4SIMD const & x, 
		float const & y);

	//! Returns min(max(x, minVal), maxVal) for each component in x 
	//! using the floating-point values minVal and maxVal.
	//! (From GLM_GTX_simd_vec4 extension, common function)
	detail::fvec4SIMD clamp(
		detail::fvec4SIMD const & x, 
		detail::fvec4SIMD const & minVal, 
		detail::fvec4SIMD const & maxVal); 

	detail::fvec4SIMD clamp(
		detail::fvec4SIMD const & x, 
		float const & minVal, 
		float const & maxVal); 

	//! \return If genTypeU is a floating scalar or vector: 
	//! Returns x * (1.0 - a) + y * a, i.e., the linear blend of 
	//! x and y using the floating-point value a. 
	//! The value for a is not restricted to the range [0, 1].
	//!
	//! \return If genTypeU is a boolean scalar or vector: 
	//! Selects which vector each returned component comes
	//! from. For a component of a that is false, the
	//! corresponding component of x is returned. For a
	//! component of a that is true, the corresponding
	//! component of y is returned. Components of x and y that
	//! are not selected are allowed to be invalid floating point
	//! values and will have no effect on the results. Thus, this
	//! provides different functionality than
	//! genType mix(genType x, genType y, genType(a))
	//! where a is a Boolean vector.
	//! 
	//! From GLSL 1.30.08 specification, section 8.3
	//! 
	//! \param[in]  x Floating point scalar or vector.
	//! \param[in]  y Floating point scalar or vector.
	//! \param[in]  a Floating point or boolean scalar or vector.
	//!
	// \todo Test when 'a' is a boolean.
	//! (From GLM_GTX_simd_vec4 extension, common function)
	detail::fvec4SIMD mix(
		detail::fvec4SIMD const & x, 
		detail::fvec4SIMD const & y, 
		detail::fvec4SIMD const & a);

	//! Returns 0.0 if x < edge, otherwise it returns 1.0.
	//! (From GLM_GTX_simd_vec4 extension, common function)
	detail::fvec4SIMD step(
		detail::fvec4SIMD const & edge, 
		detail::fvec4SIMD const & x);

	detail::fvec4SIMD step(
		float const & edge, 
		detail::fvec4SIMD const & x);

	//! Returns 0.0 if x <= edge0 and 1.0 if x >= edge1 and
	//! performs smooth Hermite interpolation between 0 and 1
	//! when edge0 < x < edge1. This is useful in cases where
	//! you would want a threshold function with a smooth
	//! transition. This is equivalent to:
	//! genType t;
Binary file ./Stereo/lib/glm/gtx/simd_vec4.hpp matches
