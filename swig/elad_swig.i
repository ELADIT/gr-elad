/* -*- c++ -*- */
/* 
 * Copyright 2015 ELAD . 
 *  
 * This is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.
 *
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */


#define ELAD_API

%include "gnuradio/swig/gnuradio.i"			// the common stuff

//load generated python docstrings
%include "elad_swig_doc.i"

%{
#include "elad/fdm_source_c.h"
%}


%include "elad/fdm_source_c.h"
GR_SWIG_BLOCK_MAGIC2(elad, fdm_source_c);
