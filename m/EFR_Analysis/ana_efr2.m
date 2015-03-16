# -*- octave -*-
# $Id$


#[rs,t,d,z,a,r]=suncycle(51.174, 15.918, [datenum(2012,1,1):1:(datenum(2013,1,1)-1)]');

#idx=find(abs(z(:,:)+ 7)<0.1);plot(24*60*mod(t(idx),1), floor(t(idx)), '.b');
#idx=find(abs(z(:,:)- 1)<0.1);plot(24*60*mod(t(idx),1), floor(t(idx)), '.r');
#idx=find(abs(z(:,:)-13)<0.1);plot(24*60*mod(t(idx),1), floor(t(idx)), '.g');
#idx=find(abs(z(:,:)-23)<0.1);plot(24*60*mod(t(idx),1), floor(t(idx)), '.r');

function [x,dd]=ana_efr2
  KRK = [50.061389 19.938333];
  MFL = [50.01556   9.01083 ];
  LKH = [47.373056 19.004444];
  BRG = [52.285833 11.898056];
  
  stns = {"L2.129100_EFR_Mainflingen", ...
          "L2.135600_EFR_Lakihegy", ...
          "L2.139000_EFR_Burg"};  
  limits = {[-70 -40],
	    [-50 -35],
	    [-60 -40]};
  coords = [(KRK+MFL)/2
	    (KRK+LKH)/2
	    (KRK+BRG)/2];
  for year=2012:2014
    for i=1:length(stns)
      d{i} = load(sprintf("%d_%s.mat", year, stns{i}));
      x=plot_year(d{i}.a.median, year, stns{i}, limits{i}, coords(i,:));
      dd=d{i}.a.median;
    endfor
  endfor
endfunction

function x=plot_year(a, year, stn, lim, coords)
  [rs,t,dec,z,a_,r]=suncycle(coords(1), coords(2), [datenum(year,1,1):1:(datenum(year+1,1,1)-1)]');

  colordata = colormap;
  bbb = colordata(end,:) == [1 1 1];
  if sum(bbb) != 3
    colordata_new=[colordata; 1 1 1];
    colormap(colordata_new);
  endif

  imagesc([1:1440]/60, [1:size(a,1)], a, lim+0.3*[0 1]);

  set(gca, 'xtick', [0: 4: 24]);
  set(gca, 'ytick', [0:45:360]);
  grid on;
  title(strrep(stn, "_", "\\_"));
  xlabel("UPC (h)");
  ylabel(sprintf("day of year %d", year));
  colorbar;
  hold on;
  idx=find(abs(z(:,:)+ 7)<0.25); plot(24*mod(t(idx),1), floor(t(idx)), '.b');
  idx=find(abs(z(:,:)- 1)<0.25); plot(24*mod(t(idx),1), floor(t(idx)), '.r');
  idx=find(abs(z(:,:)-13)<0.25); plot(24*mod(t(idx),1), floor(t(idx)), '.g');
  idx=find(abs(z(:,:)-23)<0.25); plot(24*mod(t(idx),1), floor(t(idx)), '.m');  
  print(sprintf("%d_%s.eps", year, stn),"-deps");
  print(sprintf("%d_%s.pdf", year, stn),"-dpdf");
  hold off
endfunction

function x=plot_year_hist(a, year, stn, lim, coords)
  [rs,t,dec,z,a_,r]=suncycle(coords(1), coords(2), [datenum(year,1,1):1:(datenum(year+1,1,1)-1)]');

  ## compute night times for each day
  b_night = zeros(size(a));
  size(b_night)
  N=size(a,1);
  for day=1:size(a,1)
    ts = round(1440*[mean(t(day,     find(abs(z(day,   1:1440)+7)<0.1))-day+1)
		     mean(t(day,1440+find(abs(z(day,1441:2880)+7)<0.1))-day+1)]);
    b_night(day,1:ts(1)) = 1;
    b_night(day,ts(2):N) = 1;
  endfor
  b_night = b_night==1;
  b_day   = ~b_night;

  days = [1:N]';
  b_winter = (days<=180-90 | days>180+90)*ones(1,1440) == 1;
  b_summer = ~-b_winter;

  x=a(b_winter & b_night);   
endfunction
