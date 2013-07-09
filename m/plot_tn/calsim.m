# -*- octave -*-

function calsim
  fs = [60, 75, 77.5, 162]*1e3;
  slope_in = 6*1e-6;  

  df = 1;
  dt = 1;

  fs_measured = fs + fs*slope_in;

  subplot(2,1,1);
  plot(fs, fs_measured-fs, '*');

  dt = 2;
  df = 1;
  
  subplot(2,1,2);
  fs_measured = fs + mod(fs*slope_in * 2, 1)/2;
  plot(fs, fs_measured-fs, '*');
  

endfunction