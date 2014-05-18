# -*- octave -*-

function fs=plot_fsk_freq(plotFileName, dataFileName, v0, df)
  system(["gzip -dc " dataFileName ...
	  " | awk '!/^#/ {printf(\"%.6f %f %f\\n\", substr($2,0,2)+substr($2,4,2)/60+substr($2,7,10)/3600, $3,$5)}' > fs"]);
  
  fs=load("fs");

  r = sprintf("%d/%d", v0-250, v0+250);
  
  cmd = ["psxy -X2.5i -JX6i/3i -R0/24/" r " -B3g3:'UTC / h':/100g50:'frequency / Hz':WSne -Sc.06c -Wblue -Gblue"];
  fid = popen([cmd " -K > " plotFileName], "w");
  fprintf(fid, "%f %e\n", fs(:,1:2)');
  fclose(fid);

  cmd = "psxy -J -R -Wfat,darkgray,-";
  fid = popen([cmd " -O -K >> " plotFileName], "w");
  fprintf(fid, " 0 %d\n", v0);
  fprintf(fid, "24 %d\n", v0);
  fclose(fid);

  cmd = "psxy -J -R -Wfat,darkgreen";
  fid = popen([cmd " -O -K >> " plotFileName], "w");
  fprintf(fid, "6 %d\n", v0-df);
  fprintf(fid, "6 %d\n", v0+df);
  fclose(fid);

  cmd = "pstext -J -R -Gdarkgreen";
  fid = popen([cmd " -O -K >> " plotFileName], "w");
  fprintf(fid, "3   %d 14 0 0 CB %d Hz\n", v0-0.9*df, v0-df);
  fprintf(fid, "3   %d 14 0 0 CT %d Hz\n", v0+0.9*df, v0+df);
  fprintf(fid, "6.5 %d 20 0 0 LM @~D@~f = -%d Hz\n", v0-df/2, df);
  fprintf(fid, "6.5 %d 20 0 0 LM @~D@~f = +%d Hz\n", v0+df/2, df);
  fclose(fid);

  plot_title(plotFileName, dataFileName);
  ps2png(plotFileName);

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