# -*- octave -*-

function plot_dt
  system("awk '{printf(\"%.6f %.6f\\n\", 3600*substr($3,0,2)+60*substr($3,4,2)+substr($3,7,10), 3600*substr($5,0,2)+60*substr($5,4,2)+substr($5,7,10))}' xx.txt > xx");
  load xx
  x=diff(xx(:,1));
  [y,t]=hist(1e6*(x(abs(x/0.00544-1)<.05)-0.00544),100); # t in 10-6 sec
  [m,ip]=max(y(t>0))
  [m,im]=max(y(t<0))
  pp = t(t>0)(ip) 
  pm = t(t<0)(im)

  dt=diff(t)(1)
  N=length(t);
  tt(1:2:2*N-1)=t-dt/2;
  tt(2:2:2*N  )=t+dt/2;
  yy(1:2:2*N-1)=y;
  yy(2:2:2*N)  =y;

  plotFileName="dt";
  cmd = "psxy -JX6i/3i -R-200/200/0/4000 -B50g50:'dt / @~m@~sec':/1000g1000:N:WSen -Wthick,black -Gblue ";
  fid = popen([cmd " -K > " plotFileName ".ps"], "w");
  fprintf(fid, "%f %f\n", [tt;yy])
  fprintf(fid, "%f %f\n", [tt;yy])
  fclose(fid);

  cmd = "psxy -J -R -Wthick,black -Wthicker,darkgreen ";
  fid = popen([cmd " -O -K >> " plotFileName ".ps"], "w");
  fprintf(fid, "%f 2500\n%f 2500", -128/2, 128/2);
  fclose(fid);

  cmd = "pstext -J -R  ";
  fid = popen([cmd " -O >> " plotFileName ".ps"], "w");
  fprintf(fid, "0 2600 18 0 0 CB @;darkgreen;128 @~m@~sec@;;");
  fclose(fid);

  system(["ps2raster -E250 -Tg -A -P " plotFileName ".ps"]);

endfunction