# -*- octave -*-
# $Id$
#
# arguments:
#   filenames... cell array of input filenames
#   qrgs ...     2-component vector with frequency limits in Hz
#
#  examples 1) plot_spec({"file1", "file2"}), or
#           2) plot_spec(glob("objdir/Data/_VLF2_/y2012-m03-d29*"), [20e3 25e3])
#

function plot_spec(filenames, qrgs)
  if nargin < 1
    filenames=glob("objdir/Data/_VLF2_/y2012-m03-d29*")
  endif
  d = ps(filenames);

  b = d.freq_Hz >= 0;
  if nargin == 2
    b = d.freq_Hz>qrgs(1) & d.freq_Hz<qrgs(2);
  endif

  imagesc(d.freq_Hz(b), d.time, d.spec(:,b));
  datetick('y', 'HH:MM');
  ylabel("time");
  xlabel("frequency (Hz)");
endfunction