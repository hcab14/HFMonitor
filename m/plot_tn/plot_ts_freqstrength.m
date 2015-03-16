# -*- octave -*-

function [fs,x,n]=plot_ts_freqstrength(plotFileName, dataFileName, v0, threshold)
  system(["gzip -dc " dataFileName ...
	  "| awk '!/^#/ {printf(\"%.6f %f %f %f\\n\", substr($2,0,2)+substr($2,4,2)/60+substr($2,7,10)/3600, $3,$5, $7)}' > fs"]);
  
  fs=load("fs");

  r = sprintf("%d/%d/0/40", v0-200, v0+200);
  cmd = ["psxy -JX6i/3i -R" r " -B100g50:' freq / Hz':/10g5:'S/N / dB':WSne -Sc.05c -Gblue -Wblue "];
  fid = popen([cmd " -K > " plotFileName], "w");
  fprintf(fid, "%f %f\n", fs(:,[2 4])');
  fclose(fid);

  cmd = ["psxy -J -R -Wthicker,red,- "];
  fid = popen([cmd " -O -K >> " plotFileName], "w");
  fprintf(fid, "%f %f\n", v0-200, threshold)
  fprintf(fid, "%f %f\n", v0+200, threshold)
  fclose(fid);

  plot_title(plotFileName, dataFileName);

  ps2png(plotFileName);
endfunction