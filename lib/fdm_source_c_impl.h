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

#ifndef INCLUDED_ELAD_FDM_SOURCE_I_IMPL_H
#define INCLUDED_ELAD_FDM_SOURCE_I_IMPL_H

#define DEFAULT_READLEN		12288
#define DEFAULT_BUFFER_MUL	8	
#define DEFAULT_BUFFER_LEVEL	0.5f
#define WAIT_FUDGE		(1.2+0.3)

#include <cstdlib>

#include <elad/fdm_source_c.h>

#include <boost/thread.hpp>
#include <boost/thread/condition.hpp>
#include <boost/thread/recursive_mutex.hpp>

#include <dlfcn.h>


namespace gr {
  namespace elad {

    class fdm_source_c_impl : public fdm_source_c
    {
    private:

		size_t m_recv_samples_per_packet;
		uint64_t m_nSamplesReceived;
		uint32_t m_nOverflows;
		bool m_bRunning;
		boost::recursive_mutex d_mutex;
		boost::thread m_pCaptureThread;
		uint32_t m_nBufferSize;
		uint32_t m_nBufferStart;
		uint32_t m_nBufferItems;
		boost::condition m_hPacketEvent;
		uint8_t* m_pUSBBuffer;
		bool m_bBuffering;
		uint32_t m_nReadLength;
		uint32_t m_nBufferMultiplier;
		bool m_bUseBuffer;
		float m_fBufferLevel;
		uint32_t m_nReadPacketCount;
		uint32_t m_nBufferOverflowCount;
		uint32_t m_nBufferUnderrunCount;
		uint32_t m_bytes_per_sample;
		boost::xtime m_wait_delay, m_wait_next;
		libusb_device_handle *m_dev_handle;
		int m_isDuo;
		int m_isS2;
		int m_isS1;
		float m_globalOffset;
		float m_lpOffset;
		float m_attOffset;
		float m_recalc;
		int m_resampling;
		float m_rescale;


		libusb_context *ctx; 		
		struct devInfo
		{
			libusb_device_handle *deviceHandle;
			unsigned char deviceSpeed;
			unsigned short vid;
			unsigned short pid;
			char	*sn;
		}myDevInfo;

		void reset();
		static void _capture_thread(fdm_source_c_impl *p);
		void capture_thread();

	public:
	  	fdm_source_c_impl(long LOfreq, int filter, int atten, char *serial, int resampling);
	  	~fdm_source_c_impl();
                void fdm_cb_in( struct libusb_transfer * transfer );
		bool start();
		bool stop();
		void calc_delay(long dSampleRate);
		int work(int noutput_items, gr_vector_const_void_star &input_items, gr_vector_void_star &output_items);
		void set_filter(long freq, int filter, int atten);
		void set_atten(long freq, int filter, int atten);
		void set_FREQ( long LOfreq );


                long d_LOfreq;
                int d_filter;
                int d_atten;
	};

  } // namespace elad
} // namespace gr

#endif /* INCLUDED_ELAD_FDM_SOURCE_I_IMPL_H */

