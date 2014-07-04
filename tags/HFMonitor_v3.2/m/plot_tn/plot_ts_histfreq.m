# -*- octave -*-

function [fs,x,n]=plot_ts_histfreq(plotFileName, dataFileName, v0, threshold)
  system(["gzip -dc " dataFileName ...
	  "| awk '!/^#/ {printf(\"%.6f %f %f %f\\n\", substr($2,0,2)+substr($2,4,2)/60+substr($2,7,10)/3600, $3,$5, $7)}' > fs"]);
  
  fs=load("fs");

  b = fs(:,4) > threshold;
  
  [x,n] = make_hist([-.4:.02:.4], fs(b,2)-v0);

  n(n<1e-2)=5e-3;

  cmd = ["psxy -JX6i/3il -R-.4/.4/5e-3/20 -B.1g.1:' @~D@~f / Hz':/a1pf3g1:'pdf':WSne -Glightblue -Wthick,blue "];
  fid = popen([cmd " -K > " plotFileName], "w");
  fprintf(fid, "%f %e\n", [x;n]);
  fclose(fid);

  cmd = ["psxy -J -R -Wthicker,darkgreen,- "];
  fid = popen([cmd " -O -K >> " plotFileName], "w");
  fprintf(fid, "%f %e\n", [x;normpdf(x,0, 0.11)]);
  fclose(fid);

  xx=fs(b,2)-v0;
  xx(abs(xx)>.4)=[];

  cmd = ["pstext -JX6i/3i -R0/1/0/1 "];
  fid = popen([cmd " -O -K >> " plotFileName], "w");
  fprintf(fid, "0.03 0.90 14 0 0 LT @;blue;(@~m@~,@~s@~)=(%.3f,%.3f) Hz@;;\n", mean(xx), std(xx));
  fprintf(fid, "0.03 0.82 14 0 0 LT @;darkgreen;@~s@~@-overbound@-=%.2f Hz@;;\n", 0.11);
  fclose(fid);

  plot_title(plotFileName, dataFileName);

  ps2png(plotFileName);
endfunction

function [x,n] = make_hist(x0,y)
  for i=1:length(x0)-1
    n(2*i-1) = sum(y>x0(i) & y<=x0(i+1));
    n(2*i) = n(2*i-1);
    x(2*i-1) = x0(i);
    x(2*i  ) = x0(i+1);
  endfor
  norm = length(y) * diff(x0)(1)
  n ./= norm;
endfunction