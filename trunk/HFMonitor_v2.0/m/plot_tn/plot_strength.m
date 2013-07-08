# -*- octave -*-

function fs=plot_strength(plotFileName, dataFileNames)
  N=length(dataFileNames);
  system(["psbasemap -X2.5i -JX6i/3i -R0/24/-120/-60 -B3g3:'UTC / h':/20g10:'strength / dBm':WSne -K >" ...
	  plotFileName]);

  for i=1:N
    plot_single(plotFileName, dataFileNames{i});
  endfor  
  plot_title(plotFileName, dataFileNames{1});

  ps2png(plotFileName);
endfunction

function plot_single(plotFileName, dataFileName)
  system(["gzip -dc " dataFileName ...
	  "| awk '!/^#/ {printf(\"%.6f %f %f\\n\", substr($2,0,2)+substr($2,4,2)/60+substr($2,7,10)/3600, $3,$4)}' > fs"]);  
  fs=load("fs");

  filter_lp = @(x, a) filter([0 a], [1 a-1], x, x(1));

  cmd = ["psxy -J -R -Sc.04c -Wlightblue -Glightblue"];
  fid = popen([cmd " -O -K >> " plotFileName], "w");
  fprintf(fid, "%f %f\n", [fs(:,1), fs(:,3)]');
  fclose(fid);

  cmd = ["psxy -J -R -Sc.04c -Wblue -Gblue"];
  fid = popen([cmd " -O -K >> " plotFileName], "w");
  fprintf(fid, "%f %f\n", [fs(:,1), filter_lp(fs(:,3), 2/60)]');
  fclose(fid);
endfunction
