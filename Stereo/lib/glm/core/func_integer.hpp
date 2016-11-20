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
/// @ref core
/// @file glm/core/func_integer.hpp
/// @date 2010-03-17 / 2011-06-18
/// @author Christophe Riccio
///
/// @see <a href="http://www.opengl.org/registry/doc/GLSLangSpec.4.20.8.pdf">GLSL 4.20.8 specification, section 8.8 Integer Functions</a>
/// 
/// @defgroup core_func_integer Integer functions
/// @ingroup core
/// 
/// These all operate component-wise. The description is per component. 
/// The notation [a, b] means the set of bits from bit-number a through bit-number 
/// b, inclusive. The lowest-order bit is bit 0.
///////////////////////////////////////////////////////////////////////////////////

#ifndef glm_core_func_integer
#define glm_core_func_integer GLM_VERSION

namespace glm
{
	/// @addtogroup core_func_integer
	/// @{

	/// Adds 32-bit unsigned integer x and y, returning the sum
	/// modulo pow(2, 32). The value carry is set to 0 if the sum was
	/// less than pow(2, 32), or to 1 otherwise.
	///
	/// @tparam genUType Unsigned integer scalar or vector types.
	/// 
    /// @see <a href="http://www.opengl.org/sdk/docs/manglsl/xhtml/uaddCarry.xml">GLSL uaddCarry man page</a>
    /// @see <a href="http://www.opengl.org/registry/doc/GLSLangSpec.4.20.8.pdf">GLSL 4.20.8 specification, section 8.8 Integer Functions</a>
	template <typename genUType>
	genUType uaddCarry(
		genUType const & x, 
		genUType const & y, 
		genUType & carry);

	/// Subtracts the 32-bit unsigned integer y from x, returning
	/// the difference if non-negative, or pow(2, 32) plus the difference
	/// otherwise. The value borrow is set to 0 if x >= y, or to 1 otherwise.
	///
	/// @tparam genUType Unsigned integer scalar or vector types.
	/// 
    /// @see <a href="http://www.opengl.org/sdk/docs/manglsl/xhtml/usubBorrow.xml">GLSL usubBorrow man page</a>
    /// @see <a href="http://www.opengl.org/registry/doc/GLSLangSpec.4.20.8.pdf">GLSL 4.20.8 specification, section 8.8 Integer Functions</a>
	template <typename genUType>
	genUType usubBorrow(
		genUType const & x, 
		genUType const & y, 
		genUType & borrow);
		
	/// Multiplies 32-bit integers x and y, producing a 64-bit
	/// result. The 32 least-significant bits are returned in lsb.
	/// The 32 most-significant bits are returned in msb.
	///
	/// @tparam genUType Unsigned integer scalar or vector types.
	/// 
    /// @see <a href="http://www.opengl.org/sdk/docs/manglsl/xhtml/umulExtended.xml">GLSL umulExtended man page</a>
    /// @see <a href="http://www.opengl.org/registry/doc/GLSLangSpec.4.20.8.pdf">GLSL 4.20.8 specification, section 8.8 Integer Functions</a>
	template <typename genUType>
	void umulExtended(
		genUType const & x, 
		genUType const & y, 
		genUType & msb, 
		genUType & lsb);
		
	/// Multiplies 32-bit integers x and y, producing a 64-bit
	/// result. The 32 least-significant bits are returned in lsb.
	/// The 32 most-significant bits are returned in msb.
	/// 
	/// @tparam genIType Signed integer scalar or vector types.
	///
    /// @see <a href="http://www.opengl.org/sdk/docs/manglsl/xhtml/imulExtended.xml">GLSL imulExtended man page</a>
    /// @see <a href="http://www.opengl.org/registry/doc/GLSLangSpec.4.20.8.pdf">GLSL 4.20.8 specification, section 8.8 Integer Functions</a>
	template <typename genIType>
	void imulExtended(
		genIType const & x, 
		genIType const & y, 
		genIType & msb, 
		genIType & lsb);

	/// Extracts bits [offset, offset + bits - 1] from value,
	/// returning them in the least significant bits of the result.
	/// For unsigned data types, the most significant bits of the
	/// result will be set to zero. For signed data types, the
Binary file ./Stereo/lib/glm/core/func_integer.hpp matches
