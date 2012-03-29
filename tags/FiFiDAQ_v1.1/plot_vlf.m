# -*- octave -*-
# $Id$
#
# filenames: cell array of input filenames
#
#  examples 1) plot_vlf({"file1", "file2"}), or
#           2) plot_vlf(glob("objdir/Data/_VLF2_/y2012-m03-d29*"))
#

function plot_vlf(filenames)
  if nargin != 1
    filenames=glob("objdir/Data/_VLF2_/y2012-m03-d29*")
  endif

  ## qrgs [kHz]
  qrgs   = [20.9           22.1                 24            23.4];
  labels = {"g;20.9 HWU;", "r;22.1 Skeleton;", "m;24.0 NAA;", "b;23.4 DHO;"};

  d=ps(filenames);
  ##
  ## df .. Frequenzmittelungsbreite
  ##
  df=0.05;  ## kHz

  figure; hold on;
  for i=1:length(qrgs)
    idx=find(d.freq_Hz>1e3*(qrgs(i)-df) & d.freq_Hz<1e3*(qrgs(i)+df));
    plot(d.time,mean(d.spec(:,idx)'), labels{i});
  endfor
  datetick('x', 'HH:MM');
  xlabel("time");
  ylabel("relative signal strength (dB)");
endfunction