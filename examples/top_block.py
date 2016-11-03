#!/usr/bin/env python
##################################################
# Gnuradio Python Flow Graph
# Title: Top Block
# Generated: Mon Mar 21 22:11:40 2016
##################################################

from gnuradio import eng_notation
from gnuradio import gr
from gnuradio import wxgui
from gnuradio.eng_option import eng_option
from gnuradio.fft import window
from gnuradio.filter import firdes
from gnuradio.wxgui import fftsink2
from grc_gnuradio import wxgui as grc_wxgui
from optparse import OptionParser
import elad
import wx

class top_block(grc_wxgui.top_block_gui):

    def __init__(self):
        grc_wxgui.top_block_gui.__init__(self, title="Top Block")
        _icon_path = "/usr/share/icons/hicolor/32x32/apps/gnuradio-grc.png"
        self.SetIcon(wx.Icon(_icon_path, wx.BITMAP_TYPE_ANY))

        ##################################################
        # Variables
        ##################################################
        self.samplesXsimbol = samplesXsimbol = 16
        self.samp_rate = samp_rate = 192000

        ##################################################
        # Blocks
        ##################################################
        self.wxgui_fftsink2_3 = fftsink2.fft_sink_c(
        	self.GetWin(),
        	baseband_freq=0,
        	y_per_div=10,
        	y_divs=10,
        	ref_level=0,
        	ref_scale=2.0,
        	sample_rate=192000,
        	fft_size=1024,
        	fft_rate=15,
        	average=False,
        	avg_alpha=None,
        	title="FFT Plot",
        	peak_hold=False,
        )
        self.Add(self.wxgui_fftsink2_3.win)
        self.wxgui_fftsink2_2 = fftsink2.fft_sink_c(
        	self.GetWin(),
        	baseband_freq=0,
        	y_per_div=10,
        	y_divs=10,
        	ref_level=0,
        	ref_scale=2.0,
        	sample_rate=192000,
        	fft_size=1024,
        	fft_rate=15,
        	average=False,
        	avg_alpha=None,
        	title="FFT Plot",
        	peak_hold=False,
        )
        self.Add(self.wxgui_fftsink2_2.win)
        self.wxgui_fftsink2_0 = fftsink2.fft_sink_c(
        	self.GetWin(),
        	baseband_freq=0,
        	y_per_div=10,
        	y_divs=10,
        	ref_level=-30,
        	ref_scale=2.0,
        	sample_rate=samp_rate,
        	fft_size=1024,
        	fft_rate=15,
        	average=False,
        	avg_alpha=None,
        	title="Bandwidth",
        	peak_hold=False,
        	win=window.blackmanharris,
        	size=(300,100),
        )
        self.GridAdd(self.wxgui_fftsink2_0.win, 0, 1, 1, 1)
        self.elad_fdm_source_c_3 = elad.fdm_source_c(LOfreq=14180000,
        	filter=1,
        	atten=1,
        	serial="SF02QF",
        	resampling=192000)
          
        self.elad_fdm_source_c_2 = elad.fdm_source_c(LOfreq=14200000,
        	filter=0,
        	atten=1,
        	serial="SC0EOU_0003T",
        	resampling=192000)
          
        self.elad_fdm_source_c_0 = elad.fdm_source_c(LOfreq=14180000,
        	filter=0,
        	atten=0,
        	serial="SE0COD  ",
        	resampling=192000)
          

        ##################################################
        # Connections
        ##################################################
        self.connect((self.elad_fdm_source_c_0, 0), (self.wxgui_fftsink2_0, 0))
        self.connect((self.elad_fdm_source_c_2, 0), (self.wxgui_fftsink2_2, 0))
        self.connect((self.elad_fdm_source_c_3, 0), (self.wxgui_fftsink2_3, 0))



    def get_samplesXsimbol(self):
        return self.samplesXsimbol

    def set_samplesXsimbol(self, samplesXsimbol):
        self.samplesXsimbol = samplesXsimbol

    def get_samp_rate(self):
        return self.samp_rate

    def set_samp_rate(self, samp_rate):
        self.samp_rate = samp_rate
        self.wxgui_fftsink2_0.set_sample_rate(self.samp_rate)

if __name__ == '__main__':
    import ctypes
    import sys
    if sys.platform.startswith('linux'):
        try:
            x11 = ctypes.cdll.LoadLibrary('libX11.so')
            x11.XInitThreads()
        except:
            print "Warning: failed to XInitThreads()"
    parser = OptionParser(option_class=eng_option, usage="%prog: [options]")
    (options, args) = parser.parse_args()
    tb = top_block()
    tb.Start(True)
    tb.Wait()
