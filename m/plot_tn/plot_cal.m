# -*- octave -*-

function plot_cal(date)
  system(["awk '!/^#/ {printf(\"%.6f\\n\", 3600*substr($2,0,2)+60*substr($2,4,2)+substr($2,7,10))}' " ...
	  "Data/L2.077500_TS_DCF77/" date ".txt > ttt"]);
  
  system(["awk '!/^#/ {printf(\"%.6f %e\\n\", substr($2,0,2)+substr($2,4,2)/60.+substr($2,7,10)/3600., 1e-6*$4)}' " ...
	  "Data/L1.CAL/" date ".txt > cal"]);

  t=load("ttt");
  x = 0.5*(t(1:end-1)+t(2:end))/3600; # in h 
  y = abs(diff(t)/2-1);
  max(abs(y))
#  [x,y]=xyz(t);

  plotFileName="cal.ps";
  cmd = "psxy -JX6i/3il -R0/24/1e-7/1e-1 -B3g3:'UTC / h':/a1pf3g1:'abs(@~D@~t/@~D@~t@-0@--1)':WSne -Sc.06c -Wblue -Gblue";
  fid = popen([cmd " -K > " plotFileName], "w")
  fprintf(fid, "%f %e\n", [x y]');
  fclose(fid);

  tc=load("cal");
  cmd = "psxy -J -R -Wfat,red ";
  fid = popen([cmd " -O >> " plotFileName], "w")
  fprintf(fid, "%f %e\n", tc');
  fclose(fid);

  system(["ps2raster -A -Tg -E250 -P " plotFileName]);
endfunction

function [x,y]=xyz(t)
  x=[];
  y=[];
  dt=0.5;
  for hh=0:23
    for mm=0:dt:60-dt
      xe=hh+(mm+dt/2)/60;
      tt=t(t/3600 > xe-dt/2/60 & t/3600 <= xe+dt/2/60);
      if length(tt) > 0
	x(end+1,1) = xe;
	y(end+1,1) = abs(median(diff(tt)/2-1));
      endif
    endfor
  endfor
  semilogy(x,y)
endfunction