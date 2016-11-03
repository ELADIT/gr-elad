/* -*- c++ -*- */
/* 
 * Copyright 2015 ELAD
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


#ifndef INCLUDED_ELAD_FDM_SOURCE_C_H
#define INCLUDED_ELAD_FDM_SOURCE_C_H

#include <elad/api.h>
#include <gnuradio/sync_block.h>

namespace gr {
  namespace elad {

    /*!
     * \brief <+description of block+>
     * \ingroup elad
     *
     */
    class ELAD_API fdm_source_c : virtual public gr::sync_block
    {
     public:
      typedef boost::shared_ptr<fdm_source_c> sptr;

      /*!
       * \brief Return a shared_ptr to a new instance of elad::fdm_source_c.
       *
       * To avoid accidental use of raw pointers, elad::fdm_source_c's
       * constructor is in a private implementation
       * class. elad::fdm_source_c::make is the public interface for
       * creating new instances.
       */
//      void set_FREQ( long LOfreq );
//      void set_filter( long LOfreq, int filter, int atten );
//      void set_atten( long LOfreq, int filter, int atten );

      static sptr make(long LOfreq, int filter, int atten, char *serial, int resampling);
    };

  } // namespace elad
} // namespace gr

#endif /* INCLUDED_ELAD_FDM_SOURCE_C_H */

