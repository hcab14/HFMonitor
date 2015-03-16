# -*- octave -*-

function fs=plot_ts_strength(plotFileName, dataFileName, threshold)
  system(["gzip -dc " dataFileName ...
	  "| awk '!/^#/ {printf(\"%.6f %f %f %f\\n\", substr($2,0,2)+substr($2,4,2)/60+substr($2,7,10)/3600, $3,$5, $7)}' > fs"]);
  
  fs=load("fs");

  b = fs(:,4) > threshold;
  
  cmd = ["psxy -X2.5i -JX6i/3i -R0/24/-120/0 -B3g3:'UTC / h':/20g20:'strength / dBm':WSne -Sc.04c -Wred -Gred"];
  fid = popen([cmd " -K > " plotFileName], "w");
  fprintf(fid, "%f %f\n", [fs(b,1), fs(b,3)]')
  fclose(fid);

  cmd = ["psxy -J -R -Sc.04c -Wblue -Gblue"];
  fid = popen([cmd " -O -K >> " plotFileName], "w");
  fprintf(fid, "%f %f\n", [fs(~b,1), fs(~b,3)]')
  fclose(fid);

  plot_title(plotFileName, dataFileName);

  ps2png(plotFileName);
  system(["ps2raster -A -Tg -E250 -P  " plotFileName]);
endfunction
