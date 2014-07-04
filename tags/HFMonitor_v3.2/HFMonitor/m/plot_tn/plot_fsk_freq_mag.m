# -*- octave -*-

function fs=plot_fsk_freq_mag(plotFileName, date, v0, df, ddf)
  system(["awk '!/^#/ {printf(\"%.6f %f %f\\n\", substr($2,0,2)+substr($2,4,2)/60+substr($2,7,10)/3600, $3,$5)}' " ...
	  date " > fs"]);
  
  fs=load("fs");

  b = abs(fs(:,2)-(v0-df+ddf)) < 4;
  r = sprintf("%d/%d", v0-df-4, v0-df+4);
  
  cmd = ["psxy -X2.5i -JX6i/3i -R0/24/" r " -B3g3:'UTC / h':/2g1:'frequency / Hz':WSne -Sc.06c -Wblue -Gblue"];
  fid = popen([cmd " > " plotFileName], "w");
  fprintf(fid, "%f %f\n", [fs(b,1) fs(b,2)]');
  fclose(fid);

  system(["ps2raster -A -Tg -E250 -P  " plotFileName]);
endfunction
