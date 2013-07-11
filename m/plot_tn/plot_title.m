# -*- octave -*-

function plot_title(plotFileName, dataFileName)
  cmd = ["pstext -JX6i/3i -R0/1/0/1 -N -D0/.1i "];
  fid = popen([cmd " -O >> " plotFileName], "w");
  dd=dataFileName(end-19:end-7);
  qrg_kHz= sscanf(dataFileName(9:end-28), "%06d", [1 1])/1000;
  fprintf(fid, "0.5 1.0 18 0 0 CB %.1f kHz %s\n", 
	  qrg_kHz, 
	  dataFileName(16:end-21));
  fprintf(fid, "1.0 1.0 16 0 0 RB %s/%s/%s", dd(8:9), dd(12:13), dd(2:5));
  fclose(fid);
endfunction