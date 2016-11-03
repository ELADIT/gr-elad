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

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <unistd.h>
#include <libusb-1.0/libusb.h>

#define S_RATE 122880000
#define S_RATE_S1 61440000

#include <gnuradio/io_signature.h>
#include "fdm_source_c_impl.h"

#define ZeroMemory(p,l)			memset(p, 0x00, l)
#define ZERO_MEMORY(p)			ZeroMemory(&p, sizeof(p))
#define SAFE_DELETE_ARRAY(p)	{ if (p) { delete [] p; p = NULL; } }
#define SAFE_DELETE(p)			{ if (p) { delete p; p = NULL; } }

namespace gr {
	namespace elad {

		typedef struct _cbdata_t {
			fdm_source_c_impl *obj;
			struct libusb_transfer *transfer;
		} cbdata_t, *cbdata_p;

		fdm_source_c::sptr fdm_source_c::make(long LOfreq, int filter, int atten, char *serial, int resampling) {
			return gnuradio::get_initial_sptr (new fdm_source_c_impl(LOfreq, filter, atten, serial, resampling));
		}

                fdm_source_c_impl::fdm_source_c_impl(long LOfreq, int filter, int atten, char *serial, int resampling)
                        :gr::sync_block("fdm_source_c", gr::io_signature::make(0, 0, 0), gr::io_signature::make(1, 1, 2*sizeof(float))),
			d_LOfreq(LOfreq),
			d_filter(filter),
			d_atten(atten),
			m_nBufferItems(0),
			m_pUSBBuffer(NULL),
			m_nBufferStart(0),
			m_nBufferSize(512*24*8),
			m_bBuffering(false),
			m_nReadLength(DEFAULT_READLEN),
			m_nBufferMultiplier(DEFAULT_BUFFER_MUL),
			m_fBufferLevel(DEFAULT_BUFFER_LEVEL),
			m_nReadPacketCount(0),
			m_nBufferOverflowCount(0),
			m_nBufferUnderrunCount(0),
			m_bytes_per_sample(4),
			m_dev_handle(NULL),
			m_isDuo(0),
			m_isS2(0),
			m_isS1(0),
			m_globalOffset(0.0),
			m_lpOffset(0.0),
			m_attOffset(0.0),
			m_recalc(0.0),
			m_resampling(resampling),
			m_rescale(0.0)
			{	

			int n=0;
			libusb_device **devs;
			struct libusb_device_descriptor desc;
			int cnt, vid, pid, k;

			m_nReadLength = 512*24;
			m_nBufferMultiplier = 8;
			m_bytes_per_sample = m_resampling==6144000?2:4;
			if( m_resampling==384000 || m_resampling==768000 ) {
				m_rescale=6.0;
			} else if( m_resampling==1536000 || m_resampling==1536000 || m_resampling == 3072000 ) { 
				m_rescale=5.4;
			} else if( m_resampling==6144000 ) { 
				m_rescale=-0.7;
			}

			m_bRunning = false;

			int res;
			unsigned char buffer[1024];
			int j;
			long freq;
			double tuningFreq;
			int sampleRateCorr;
			unsigned int tuningWordHex, twLS, twMS;

                        // Must be the same since rate is determined by the libusb reads!
                        m_recv_samples_per_packet = m_nReadLength;

                        m_nBufferSize = m_recv_samples_per_packet * m_nBufferMultiplier;
                        m_pUSBBuffer = new uint8_t[m_nBufferSize];
                        assert(m_pUSBBuffer);
                        memset(m_pUSBBuffer,0, m_nBufferSize);
                        calc_delay(19200);

			fprintf( stderr, "Operations init elad-comms version 1.0\n" );

			// Initialize libusb-1.0
			res = libusb_init(&ctx);
			if( res < 0 ){
				fprintf( stderr, "Init Error %d\n", res );
				return;
			}

			// set verbosity level to 3, as suggested in the documentation
			libusb_set_debug( ctx, 3 );

			cnt = libusb_get_device_list(ctx, &devs);
			if(cnt < 0) {
				fprintf( stderr, "Get Device Error\n" ); 
				return;
			}
			for( j=0,k=0; j<cnt;j++ ) {
				res = libusb_get_device_descriptor( devs[j], &desc );
				if( res != 0 ){
					fprintf( stderr, "Get Device Descriptor %d %d\n", j, res ); 
					return;
				}
				vid=desc.idVendor;
				pid=desc.idProduct;
				fprintf( stderr, "%d vid %04x pid %04x %d\n", j, vid, pid, k );
				if( vid==0x1721 ) {
					res = libusb_open( devs[j], &m_dev_handle );
					if( res ) {
						fprintf( stderr, "Cannot open FDM Device\n" ); 
						return;
					} else {
						fprintf( stderr, "FDM Device Opened\n" ); 
					}

					// detach kernel drivers
					if( libusb_kernel_driver_active( m_dev_handle, 0 ) == 1 ) {
						fprintf( stderr, "Kernel Driver Active\n" ); 
						if( libusb_detach_kernel_driver( m_dev_handle, 0 ) == 0 ) {
							fprintf( stderr, "Kernel Driver Detached\n" ); 
						}
					}

					// claim interface
					if( libusb_claim_interface( m_dev_handle, 0 ) < 0 ) {
						fprintf( stderr, "Cannot claim Interface\n" ); 
						continue;
					}
					fprintf( stderr, "Interface claimed\n" ); 

					// read USB driver version
					memset( buffer, 0, sizeof( buffer ) );
					res = libusb_control_transfer( m_dev_handle, 0xc0, 0xFF, 0x0000, 0x0000, buffer, 2, 0 );
					if( res  ){
						fprintf( stderr, "USB Driver Version: %d.%d\n", buffer[0], buffer[1] );
					} else {
						fprintf( stderr, "USB Driver Version read failed\n" );
					}

					// read HW version
					memset( buffer, 0, sizeof( buffer ) );
					res = libusb_control_transfer( m_dev_handle, 0xc0, 0xA2, 0x404C, 0x0151, buffer, 2, 0 );
					if( res==2  ){
						fprintf( stderr, "HW Version: %d.%d\n", buffer[0], buffer[1] );
					} else {
						fprintf( stderr, "HW Version read failed\n" );
					}

					// read serial number
					memset( buffer, 0, sizeof( buffer ) );
					res = libusb_control_transfer( m_dev_handle, 0xc0, 0xA2, 0x4000, 0x0151, buffer, 32, 0 );
					if( res==32  ){
						if( pid==0x061a ) {
							fprintf( stderr, "FDM DUO " ); 
							m_isDuo=1;
							m_isS2=0;
							m_isS1=0;
						}
						if( pid==0x061c ) {
							fprintf( stderr, "FDM S2 " ); 
							m_isDuo=0;
							m_isS2=1;
							m_isS1=0;
						}
						if( pid==0x0610 ) {
							fprintf( stderr, "FDM S1 " ); 
							m_isDuo=0;
							m_isS2=0;
							m_isS1=1;
						}
						fprintf( stderr, "Serial: %-s\n", buffer ); 
					} else {
						fprintf( stderr, "Serial read failed (%d)\n", res ); 
						return;
					}
					if( !serial || !strlen(serial) || !strcmp(serial,"+") || !strcmp( (const char *)buffer, (const char *)serial ) ) {
						fprintf( stderr, "Device caught\n" ); 
						libusb_free_device_list( devs, 1 );
						fprintf( stderr, "libusb devices list freed\n" ); 
						break;
					} else {
						fprintf( stderr, "Get >%s<  Need >%s<\n", buffer, serial==NULL?"":serial );
						libusb_close( m_dev_handle );
						m_dev_handle = NULL;
					}			
				}
				k++;
			}
			if( j==cnt ) {
				fprintf( stderr, "Cannot find FDM Device\n" ); 
				return;
			}

			if( !m_dev_handle ) {
				res = libusb_open( devs[j], &m_dev_handle );
				if( res ) {
					fprintf( stderr, "Cannot open FDM Device\n" ); 
					return;
				} else {
					fprintf( stderr, "FDM Device Opened\n" ); 
				}

				libusb_free_device_list( devs, 1 );
				fprintf( stderr, "libusb devices list freed\n" ); 

				// detach kernel drivers
				if( libusb_kernel_driver_active( m_dev_handle, 0 ) == 1 ) {
					fprintf( stderr, "Kernel Driver Active\n" ); 
					if( libusb_detach_kernel_driver( m_dev_handle, 0 ) == 0 ) {
						fprintf( stderr, "Kernel Driver Detached\n" ); 
					}
				}

				// claim interface
				if( libusb_claim_interface( m_dev_handle, 0 ) < 0 ) {
					fprintf( stderr, "Cannot claim Interface\n" ); 
					return;
				}
				fprintf( stderr, "Interface claimed\n" ); 
			}

			if( m_isDuo ) {
				// stop FIFO
				memset( buffer, 0, sizeof( buffer ) );
				res = libusb_control_transfer( m_dev_handle, 0xc0, 0xE1, 0x0000, 0xE9<<8, buffer, 1, 0 );
				// res = control_transfer( localDeviceInfo.winUsbHandle, 0xc0, 0xE1, 0x0000, 0x00E9<<8, buffer, 1, 0 );
				if( res==1  ){
					fprintf( stderr, "Stop FIFO\n" );
				} else {
					fprintf( stderr, "Stop FIFO failed (%d)\n", res );
				}

				// imposta la FIFO di EP6 del CY-USB in modalità slave...
				memset( buffer, 0, sizeof( buffer ) );
				res = libusb_control_transfer( m_dev_handle, 0xc0, 0xE1, 0x0000, 0xE8<<8, buffer, 1, 0 );
				if( res==1  ){
					fprintf( stderr, "Init FIFO\n" );
				} else {
					fprintf( stderr, "Init FIFO failed (%d)\n", res );
				}

				set_FREQ( LOfreq );
				set_atten( LOfreq, filter, atten );
				set_filter( LOfreq, filter, atten );

				// start FIFO
				res = libusb_control_transfer( m_dev_handle, 0xc0, 0xE1, 0x0001, 0x0E9<<8, buffer, 1, 0 );
				if( res != 1 || buffer[0] != 0xE9 ) {
					fprintf( stderr, "Enable SYN failed (%d %02X)\n", res, buffer[0] );
				}
				fprintf( stderr, "SYN Enabled\n" );

			} else {

				// set tuning frequency into FPGA
				set_FREQ( LOfreq );
				set_atten( LOfreq, filter, atten );
				set_filter( LOfreq, filter, atten );

				// start FIFO
				res = libusb_control_transfer( m_dev_handle, 0xc0, 0xE9, 0x0001, 0x0000, buffer, 1, 0 );

				if( res != 1 || buffer[0] != 0xE9 ) {
					fprintf( stderr, "Enable SYN failed (%d %02X)\n", res, buffer[0] ); 
					return;
				}
				fprintf( stderr, "SYN Enabled\n" );
			}
			myDevInfo.deviceHandle = m_dev_handle;				

		}

		fdm_source_c_impl::~fdm_source_c_impl() {
			stop();
		}

                void fdm_source_c_impl::set_FREQ( long LOfreq ) {
			int res;
			unsigned char buffer[1024];
			int j;
			long freq;
			double tuningFreq;
			int sampleRateCorr;
			unsigned int tuningWordHex, twLS, twMS;

			// read global offset
			memset( buffer, 0, sizeof( buffer ) );
			res = libusb_control_transfer( m_dev_handle, 0xc0, 0xA2, 0x4028, 0x0151, buffer, 4, 0 );
			if( res==4  ){
				memcpy( &m_globalOffset, buffer, 4 );
				fprintf( stderr, "Global Offset: %f\n", m_globalOffset ); 
			} else {
				sampleRateCorr = 0;
				fprintf( stderr, "Global Offset failed (%d)\n", res ); 
				return;
			}

			// read lp offset
			memset( buffer, 0, sizeof( buffer ) );
			res = libusb_control_transfer( m_dev_handle, 0xc0, 0xA2, 0x402c, 0x0151, buffer, 4, 0 );
			if( res==4  ){
				memcpy( &m_lpOffset, buffer, 4 );
				fprintf( stderr, "LP Offset: %f\n", m_lpOffset ); 
			} else {
				sampleRateCorr = 0;
				fprintf( stderr, "LP Offset failed (%d)\n", res ); 
				return;
			}

			// read att offset
			memset( buffer, 0, sizeof( buffer ) );
			res = libusb_control_transfer( m_dev_handle, 0xc0, 0xA2, 0x4030, 0x0151, buffer, 4, 0 );
			if( res==4  ){
				memcpy( &m_attOffset, buffer, 4 );
				fprintf( stderr, "ATT Offset: %f\n", m_attOffset ); 
			} else {
				sampleRateCorr = 0;
				fprintf( stderr, "ATT Offset failed (%d)\n", res ); 
				return;
			}

			// read sample rate correction
			memset( buffer, 0, sizeof( buffer ) );
			res = libusb_control_transfer( m_dev_handle, 0xc0, 0xA2, 0x4024, 0x0151, buffer, 4, 0 );
			if( res==4  ){
				memcpy( &sampleRateCorr, buffer, 4 );
				fprintf( stderr, "Sample Rate Correction: %d\n", sampleRateCorr ); 
			} else {
				sampleRateCorr = 0;
				fprintf( stderr, "Sample rare correction failed (%d)\n", res ); 
				return;
			}

			if( m_isDuo ) {
				// set tuning frequency into FPGA
				memset( buffer, 0, sizeof( buffer ) );
				tuningFreq = LOfreq - (floor(LOfreq/(S_RATE+sampleRateCorr)) * (S_RATE+sampleRateCorr));
				tuningWordHex = (unsigned int)((4294967296.0*tuningFreq)/(S_RATE+sampleRateCorr));
				fprintf( stderr, "Tuning TuningHEX=%08X ", tuningWordHex );
				twLS = tuningWordHex&0x0000FFFF;
				tuningWordHex>>=16;
				twMS = 0xF2;
				twMS <<= 8;
				twMS |= (tuningWordHex&0x000000FF);
				tuningWordHex>>=8;
				buffer[0] = tuningWordHex&0x000000FF;
				buffer[1] = 0;
				// fprintf( stderr, "twMS=%04X twLS=%04X Buffer[0]=%02X Buffer[1]=%02X\n", twMS, twLS, buffer[0], buffer[1] );
				res = libusb_control_transfer( m_dev_handle, 0x40, 0xE1, twLS, twMS, buffer, 2, 0 );
				if( res == 2 ) {
					// printf( "Xilinx frequency set\n" );
				} else {
					fprintf( stderr, "Xilinx Frequency set failed\n" ); 
					return;
				}

				// verify that CAT buffer is free from previous commands
				memset( buffer, 0, sizeof( buffer ) );
				for( j=0; ; j++ ) {
					res = libusb_control_transfer( m_dev_handle, 0xc0, 0xE1, 0x00, 0x0FC<<8, buffer, 3, 0 );
					if( res != 3 || ((buffer[2]&0x04)!=0x04) || j==200 ) {
						break;
					}
					usleep( 10000 );
				} 
				if( res != 3 ) {
					fprintf( stderr, "CAT buffer waiting error\n" ); 
					return;
				}
				if ((j==200)&&(buffer[2]&0x04)==0x04) {
					fprintf( stderr, "CAT buffer waiting timeout\n" ); 
					return;
				}
				// fprintf( stderr, "CAT buffer waiting ended (%d)\n", j ); 

				// set tuning frequency to ibe delivered to CAT
				memset( buffer, 0, sizeof( buffer ) );
				sprintf( (char *)buffer, "CF%11ld;", LOfreq );
				res = libusb_control_transfer( m_dev_handle, 0x40, 0xE1, 16, 0xF1<<8, buffer, 16, 0 );
				// printf( "End setting cat frequency\n" );
				printf( "Duo set freq %ld\n", LOfreq );
			} else {
				// set tuning frequency into FPGA
				memset( buffer, 0, sizeof( buffer ) );
				tuningFreq = LOfreq - (floor(LOfreq/((m_isS1?S_RATE_S1:S_RATE)+sampleRateCorr)) * ((m_isS1?S_RATE_S1:S_RATE)+sampleRateCorr));
				tuningWordHex = (unsigned int)((4294967296.0*tuningFreq)/((m_isS1?S_RATE_S1:S_RATE)+sampleRateCorr));
				// fprintf( stderr, "Tuning TuningHEX=%08X ", tuningWordHex );
				twLS = tuningWordHex&0x0000FFFF;
				tuningWordHex>>=16;
				twMS = tuningWordHex&0x0000FFFF;
				buffer[0] = tuningWordHex&0x000000FF;
				buffer[1] = 0;
				res = libusb_control_transfer( m_dev_handle, 0x40, 0xF2, twLS, twMS, buffer, 2, 0 );
				if( res == 2 ) {
					// printf( "Xilinx frequency set\n" );
				} else {
					fprintf( stderr, "Xilinx Frequency set failed\n" ); 
				}
				printf( "S1/S2 set freq %ld\n", LOfreq );
			}
                }

                void fdm_source_c_impl::set_filter(long freq, int filter, int atten) {
			int j;
			int res;
			unsigned char buffer[1024];
			unsigned int fl;
			if( m_isS1 ) {
				res = libusb_control_transfer( m_dev_handle, 0xc0, 0xF7, (unsigned int)filter, 0x0002, buffer, 1, 0 );
				if( res != 1 || buffer[0] != 0xF7 ) {
					fprintf( stderr, "Set filters failed (%d %02X)\n", res, buffer[0] ); 
				}
				fprintf( stderr, "S1 Filters set (%d)\n", filter );
			}
			if( m_isS2 ) {
				if( freq < 61440000 ) {
					fl=0x03;
				} else if( freq < 122880000 ) {
					fl=0x02;
				} else {
					fl=0x04;
				}
				if( !filter ) {
					fl = 0x01;
				}
				if( atten ) {
					fl |=0x08;
				}
				res = libusb_control_transfer( m_dev_handle, 0xc0, 0xF7, fl, 0x0002, buffer, 1, 0 );
				if( res != 1 || buffer[0] != 0xF7 ) {
					fprintf( stderr, "Set filters failed (%d %02X)\n", res, buffer[0] ); 
				}
				fprintf( stderr, "S2 Filters set (%d)\n", filter );
			}
			if( m_isDuo ) {
				// verify that CAT buffer is free from previous commands
				memset( buffer, 0, sizeof( buffer ) );
				for( j=0; ; j++ ) {
					res = libusb_control_transfer( m_dev_handle, 0xc0, 0xE1, 0x00, 0x0FC<<8, buffer, 3, 0 );
					if( res != 3 || ((buffer[2]&0x04)!=0x04) || j==200 ) {
						break;
					}
					usleep( 10000 );
				} 
				if( res != 3 ) {
					fprintf( stderr, "CAT buffer waiting error\n" ); 
					return;
				}
				if ((j==200)&&(buffer[2]&0x04)==0x04) {
					fprintf( stderr, "CAT buffer waiting timeout\n" ); 
					return;
				}
				// fprintf( stderr, "CAT buffer waiting ended (%d)\n", j ); 

				// set tuning frequency to ibe delivered to CAT
				memset( buffer, 0, sizeof( buffer ) );
				sprintf( (char *)buffer, "LP%1d;", filter );
				res = libusb_control_transfer( m_dev_handle, 0x40, 0xE1, 16, 0xF1<<8, buffer, 16, 0 );
				fprintf( stderr, "DUO LPF set (%d - %s -> %d)\n", filter, buffer, res );
			}
			m_recalc = pow( 10.0,((m_isDuo?21.4:0)+m_rescale+m_globalOffset+(filter?m_lpOffset:0)+(atten?m_attOffset:0))/20.0);
                }

                void fdm_source_c_impl::set_atten(long freq, int filter, int atten) {
			int j;
			int res;
			unsigned char buffer[1024];

			if( m_isS2 ) {
				set_filter( freq, filter, atten );
			}
			if( m_isS1 ) {
				res = libusb_control_transfer( m_dev_handle, 0xc0, 0xF7, (unsigned int)atten, 0x0003, buffer, 1, 0 );
				if( res != 1 || buffer[0] != 0xF7 ) {
					fprintf( stderr, "Set attenuator failed (%d %02X)\n", res, buffer[0] ); 
				}
				fprintf( stderr, "S1 Attenuation set (%d)\n", atten );
			}
			if( m_isDuo ) {
				// verify that CAT buffer is free from previous commands
				memset( buffer, 0, sizeof( buffer ) );
				for( j=0; ; j++ ) {
					res = libusb_control_transfer( m_dev_handle, 0xc0, 0xE1, 0x00, 0x0FC<<8, buffer, 3, 0 );
					if( res != 3 || ((buffer[2]&0x04)!=0x04) || j==200 ) {
						break;
					}
					usleep( 10000 );
				} 
				if( res != 3 ) {
					fprintf( stderr, "CAT buffer waiting error\n" ); 
					return;
				}
				if ((j==200)&&(buffer[2]&0x04)==0x04) {
					fprintf( stderr, "CAT buffer waiting timeout\n" ); 
					return;
				}
				// fprintf( stderr, "CAT buffer waiting ended (%d)\n", j ); 

				// set tuning frequency to ibe delivered to CAT
				memset( buffer, 0, sizeof( buffer ) );
				sprintf( (char *)buffer, "AT%1d;", atten );
				res = libusb_control_transfer( m_dev_handle, 0x40, 0xE1, 16, 0xF1<<8, buffer, 16, 0 );
				fprintf( stderr, "DUO Attenuation set (%d - %s -> %d)\n", atten, buffer, res );
			}
			m_recalc = pow( 10.0,((m_isDuo?21.4:0)+m_rescale+m_globalOffset+(filter?m_lpOffset:0)+(atten?m_attOffset:0))/20.0);
                }

		bool fdm_source_c_impl::start() {
			boost::recursive_mutex::scoped_lock lock(d_mutex);

			struct sched_param param;
			param.sched_priority=90;

			if (m_bRunning) {
				return true;
			}

			// Need to set this BEFORE starting thread (otherwise it will exit)
			m_bRunning = true;				

			m_bBuffering = true;
			m_pCaptureThread = boost::thread(_capture_thread, this);

			pthread_t thread_id=(pthread_t)m_pCaptureThread.native_handle();
			// if you need priority, decomment the following instruction
			// pthread_setschedparam( thread_id, SCHED_RR, &param );

			return true;
		}

		bool fdm_source_c_impl::stop() {

			boost::recursive_mutex::scoped_lock lock(d_mutex);

			if (m_bRunning == false) {
				return true;
			}

			// Must set before 'join' as this will signal capture thread to exit
			m_bRunning = false;			

			// In case general_work is waiting
			m_hPacketEvent.notify_one();	
			// Release lock AFTER notification, so waiting thread resumes in lockstep
			lock.unlock();					
			// Wait for capture thread to finish
			m_pCaptureThread.join();		

			return true;
		}

		void fdm_source_c_impl::calc_delay(long dSampleRate) {

			boost::recursive_mutex::scoped_lock lock(d_mutex);

			double dDelay = 1000000000ULL * WAIT_FUDGE / (double)((dSampleRate) / (double)m_nReadLength);
			uint64_t delay = (uint64_t)ceil(dDelay);

			m_wait_delay.sec = delay / 1000000000ULL;
			m_wait_delay.nsec = delay % 1000000000ULL;
		}

		void fdm_source_c_impl::_capture_thread(fdm_source_c_impl *p) {
			return p->capture_thread();
		}

		void fdm_source_c_impl::fdm_cb_in( struct libusb_transfer * transfer ) {

			boost::recursive_mutex::scoped_lock lock(d_mutex, boost::defer_lock);

			// printf( "Callback\n!" );

			cbdata_p cbd = (cbdata_p)transfer->user_data;
			struct libusb_transfer * transfer_in = cbd->transfer;

			libusb_submit_transfer( transfer_in );

			uint8_t* pBuffer = transfer->buffer;

			uint32_t nRemaining = std::min(m_nBufferSize - m_nBufferItems, (uint32_t)m_nReadLength);
			bool bSignal = true;
		
			lock.lock();

			if (nRemaining > 0) {
				uint8_t* p = m_pUSBBuffer + ((m_nBufferStart + m_nBufferItems) % m_nBufferSize);
				uint32_t nPart1 = (m_nBufferStart + m_nBufferItems) % m_nBufferSize;
				uint32_t nSize1 = std::min(nRemaining, m_nBufferSize - nPart1);
				memcpy(p, pBuffer, nSize1);

				uint32_t nResidual = nRemaining - nSize1;

				if (nResidual > 0) {
					memcpy(m_pUSBBuffer, pBuffer + nSize1, nResidual);
				}

				m_nBufferItems += nRemaining;

				if ((m_bBuffering) && (m_nBufferItems >= (uint32_t)(m_recv_samples_per_packet + (float)m_nBufferSize * m_fBufferLevel))) {
					fprintf( stderr, "Finished buffering: m_nBufferItems:  %d, m_nBufferSize: %d, m_nReadPacketCount:  %d, m_nReadPacketCount: %d\n", m_nBufferItems, m_nBufferSize, m_nReadPacketCount );

					m_bBuffering = false;
				}
				bSignal = !m_bBuffering;
			} else {
				fprintf( stderr, "rB\n" );
				++m_nBufferOverflowCount;
			}
	
			lock.unlock();
	
			if (bSignal) {
				m_hPacketEvent.notify_one();
			}
		}

		void cb_in( struct libusb_transfer * transfer ) {

			cbdata_p cbd = (cbdata_p)transfer->user_data;
			fdm_source_c_impl *obj = cbd->obj;
			obj->fdm_cb_in( transfer );
		}

		void fdm_source_c_impl::capture_thread() {

			boost::recursive_mutex::scoped_lock lock(d_mutex, boost::defer_lock);

			if ((m_nReadLength == 0) || (m_pUSBBuffer == NULL) || (m_nBufferSize == 0) || (m_fBufferLevel < 0)) {
				lock.lock();
				// This will signal EOF
				m_bRunning = false;				
				m_hPacketEvent.notify_one();
				lock.unlock();

				fprintf( stderr, "Capture threading NOT starting due to state error\n" );
				fprintf( stderr, "m_nReadLength %d m_pUSBBuffer %d m_nBufferSize %d m_fBufferLevel %d\n", m_nReadLength, m_pUSBBuffer, m_nBufferSize, m_fBufferLevel );
				return;
			}
	  
			fprintf( stderr, "Capture threading starting\n" );
	  
			uint8_t* pBuffer1 = new uint8_t[m_nReadLength];
			uint8_t* pBuffer2 = new uint8_t[m_nReadLength];
	  
			struct libusb_transfer * transfer_in1 = libusb_alloc_transfer(0);
			struct libusb_transfer * transfer_in2 = libusb_alloc_transfer(0);
			
			cbdata_t cbdata1;
			cbdata_t cbdata2;

			cbdata1.obj=this;
			cbdata1.transfer=transfer_in1;
			cbdata2.obj=this;
			cbdata2.transfer=transfer_in2;

			libusb_fill_bulk_transfer( transfer_in1, myDevInfo.deviceHandle, 0x86, pBuffer1, m_nReadLength, cb_in, &cbdata1, 5000);
			libusb_fill_bulk_transfer( transfer_in2, myDevInfo.deviceHandle, 0x86, pBuffer2, m_nReadLength, cb_in, &cbdata2, 5000);
			libusb_submit_transfer( transfer_in1 );
			libusb_submit_transfer( transfer_in2 );

			fprintf( stderr, "Capture threading started\n" );
			
			for( ;; ) {
				lock.lock();
				if (m_bRunning == false) {
					break;
				}
				lock.unlock();
				libusb_handle_events_completed( ctx, NULL );
			}
	  
			fprintf( stderr, "Capture threading exiting\n" );
capture_thread_exit:
			fprintf( stderr, "Clear pBuffer ..." );
			SAFE_DELETE_ARRAY(pBuffer1);
			SAFE_DELETE_ARRAY(pBuffer2);
			fprintf( stderr, "Done\n" );
		}



		int fdm_source_c_impl::work(int noutput_items_c, gr_vector_const_void_star &input_items, gr_vector_void_star &output_items) {
			float *out = (float *) output_items[0];
			int noutput_items=noutput_items_c*2;

			boost::recursive_mutex::scoped_lock lock(d_mutex);

			if (m_bRunning == false) {
				fprintf( stderr, "Work called while not running!\n" );
				return -1;	// EOF
			}
	  
			if (noutput_items*m_bytes_per_sample > m_nBufferSize) {
				fprintf( stderr, "Work wants (%d * %d) more than the buffer size (%d)!\n", noutput_items, m_bytes_per_sample, m_nBufferSize );
				noutput_items = int(m_nBufferSize/m_bytes_per_sample);
			}

retry_notify:

			while ((m_bBuffering) || (m_nBufferItems <= ((uint32_t)(m_fBufferLevel * (float)m_nBufferSize) + m_recv_samples_per_packet))) {
				// If getting too full, send them all through
				bool notified = true;
				if (m_bBuffering) {
					// Always wait for new samples to arrive while buffering
					m_hPacketEvent.wait(lock);			
				} else {
					xtime_get(&m_wait_next, CLOCK_MONOTONIC);
					m_wait_next.nsec += m_wait_delay.nsec;
					if (m_wait_next.nsec >= 1000000000) {
						m_wait_next.sec += 1;
						m_wait_next.nsec -= 1000000000;
					}
					// Wait for more samples to arrive, 
					// or wait just longer than it would have actually taken and use buffer samples
					notified = m_hPacketEvent.timed_wait(lock, m_wait_next);	
				}

				// Timeout
				if (notified == false) {
					fprintf( stderr, "rT\n" );
					// Running late, use up some of the buffer
					break;								
				}

				if (m_bRunning == false) {
					fprintf( stderr, "No longer running after packet notification - signalling EOF\n" );
					return -1;
				}

				// No longer filling buffer, so send samples back to runtime
				if (m_bBuffering == false) {
					break;
				}

				fprintf( stderr, "Caught packet signal while buffering!\n" );
			}

			if (m_nBufferItems < m_recv_samples_per_packet) {
				fprintf( stderr, "rU\n" );
				m_bBuffering = true;
				++m_nBufferUnderrunCount;

				// Keep waiting for buffer to fill back up sufficiently
				goto retry_notify;						
			} else if (m_nBufferItems < (noutput_items*m_bytes_per_sample))	{
				// Double check
				noutput_items = int(m_nBufferItems/m_bytes_per_sample);
			}


			++m_nReadPacketCount;
			uint32_t nPart1 = std::min(uint32_t(noutput_items*m_bytes_per_sample), m_nBufferSize - m_nBufferStart);
			uint8_t* p = m_pUSBBuffer + m_nBufferStart;
			uint32_t nResidual = (noutput_items*m_bytes_per_sample) - nPart1;
			//printf( "nPart1=%d\n", nPart1/m_bytes_per_sample);
			int q=nPart1/m_bytes_per_sample;
			if(m_bytes_per_sample==2) {
				float c=m_recalc/32.0/1024.0;
				for( int j=0; j<q; j++ ) {
					short r=(short)((uint16_t *)p)[j];
					out[j]=r*c;
				}
			} else {
				float c=m_recalc/2.0/1024.0/1024.0/1024.0;
				for( int j=0; j<q; j++ ) {
					int r=(int)((uint32_t *)p)[j];
					out[j]=r*c;
				}
			}
//			memcpy(out, p, nPart1);

			if (nResidual) {
				int k=nPart1/m_bytes_per_sample;
				int q=nResidual/m_bytes_per_sample;
				if(m_bytes_per_sample==2) {
					float c=m_recalc/32.0/1024.0;
					for( int j=0; j<q; j++ ) {
						short r=(short)((uint16_t *)m_pUSBBuffer)[j];
						out[k+j]=r*c;
					}
				} else {
					float c=m_recalc/2.0/1024.0/1024.0/1024.0;
					for( int j=0; j<q; j++ ) {
						int r=(int)((uint32_t *)m_pUSBBuffer)[j];
						out[k+j]=r*c;
					}
				}
//				memcpy(out + nPart1, m_pUSBBuffer, nResidual);
			}
		  
			m_nBufferItems -= noutput_items*m_bytes_per_sample;

			if (m_nBufferItems > 0) {
				m_nBufferStart = (m_nBufferStart + noutput_items*m_bytes_per_sample) % m_nBufferSize;
			}
		  

			// Tell runtime system how many output items we produced.
			return noutput_items/2;				
		}	

	} /* namespace elad */
} /* namespace gr */
