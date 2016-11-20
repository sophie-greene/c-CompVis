/*
 * a simple UVC video viewer
 * Copyright (C) 2007 Ogochan.
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

#ifndef	__INC_CONFIG_FILE_H__
#define	__INC_CONFIG_FILE_H__

#ifndef	ARG_TABLE
typedef enum ARG_TYPE {
    BOOLEAN, INTEGER, LONGINT, STRING, PROCEDURE
} ARG_TYPE;

typedef struct {
	char		*option;
	ARG_TYPE	type;
	Bool		defval;
	void		*var;
	char		*message;
}	_ARG_TABLE;
#define	ARG_TABLE	_ARG_TABLE
#endif

extern	Bool	GetConfigFile(char *file, ARG_TABLE *args);
extern	Bool	PutConfigFile(char *file, ARG_TABLE *args);

#endif
