/*
 * Copyright (C) 1989-2007 Ogochan.
 * 
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 * 
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 * 
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the
 * Free Software Foundation, Inc., 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/*
*	Global type define
*/

#ifndef	_INC_TYPES_H
#define	_INC_TYPES_H

#include	<sys/types.h>
#include	<stdint.h>

#ifndef	byte
#define	byte		unsigned char
#endif

#ifndef	word
#define	word		unsigned short
#endif

#ifndef	dword
#define	dword		unsigned long
#endif

#ifndef	ulong
#define	ulong		unsigned long
#endif

#ifndef	Bool
#define	Bool		int
#endif

#ifndef	FALSE
#define	FALSE		0
#endif

#ifndef	TRUE
#define	TRUE		(!FALSE)
#endif

#define	IntToBool(v)	((v)?TRUE:FALSE)
#define	PRINT_BOOL(b)	((b) ? "T" : "F")

#define	TO_INT(x)	((x) - '0')
#define	TO_CHAR(x)	((x) + '0')

#define	ALIGN_BYTES		sizeof(int)

#define	ROUND_ALIGN(p)	\
	((((p)%ALIGN_BYTES) == 0) ? (p) : (((p)/ALIGN_BYTES)+1)*ALIGN_BYTES)

#endif

