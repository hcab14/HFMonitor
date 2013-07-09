# -*- octave -*-

function fs=plot_fsk_strength(plotFileName, dataFileName, v0, df, ddf, smooth, max_dBm)
  system(["gzip -dc " dataFileName ...
	  " | awk '!/^#/ {printf(\"%.6f %f %f\\n\", substr($2,0,2)+substr($2,4,2)/60+substr($2,7,10)/3600, $3,$5)}' > fs"]);
  
  fs=load("fs");

  b = abs(fs(:,2)-(v0-df+ddf)) < 0.5;

  r = sprintf("%d/%d/%d/%d", 0, 24, -120, max_dBm);

  if smooth == 0
    cmd = ["psxy -X2.5i -JX6i/3i -R" r " -B3g3:'UTC / h':/20g20:'strength / dBm':WSne -Sc.04c -Wblue -Gblue"];
    fid = popen([cmd " -K > " plotFileName], "w");
    fprintf(fid, "%f %f\n", [fs(b,1), fs(b,3)]')
    fclose(fid);
  else
    cmd = ["psxy -X2.5i -JX6i/3i -R" r " -B3g3:'UTC / h':/20g20:'strength / dBm':WSne -Sc.04c -Wlightblue -Glightblue"];
    fid = popen([cmd " -K > " plotFileName], "w");
    fprintf(fid, "%f %f\n", [fs(b,1), fs(b,3)]')
    fclose(fid);
    
    filter_lp = @(x, a) filter([0 a], [1 a-1], x, x(1));
    
    cmd = ["psxy -J -R -Sc.04c -Wblue -Gblue"];
    fid = popen([cmd " -O -K >> " plotFileName], "w");
    fprintf(fid, "%f %f\n", [fs(:,1), filter_lp(fs(:,3), 2/60)]');
    fclose(fid);
  endif

  plot_title(plotFileName, dataFileName);
  ps2png(plotFileName);
endfunction
