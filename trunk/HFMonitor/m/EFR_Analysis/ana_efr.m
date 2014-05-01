# -*- octave -*-
# $Id$

function [a,dns]=ana_efr
  stns = {"L2.129100_EFR_Mainflingen", ...
	  "L2.135600_EFR_Lakihegy", ...
	  "L2.139000_EFR_Burg"};  
  for year=2012:2013
    for i=3:length(stns)
      [a,dns] = proc_year(stns{i}, year);
      save(sprintf("%d_%s.mat", year, stns{i}), "a", "dns");
    endfor
  endfor
endfunction

function [a,dns]=proc_year(stn, year)
  t0 = datenum(year,1,1);
  t1 = datenum(year+1,1,1)-1;

  a_median = zeros(t1-t0, 60*24);
  a_mean   = zeros(t1-t0, 60*24);
  a_std    = zeros(t1-t0, 60*24);
#  t1=t0+1;
  
  dns=t0:1:t1;
  for dn=dns
    tic;
    dv=datevec(dn);    
    d=read_data(stn, dv(1), dv(2), dv(3));
    toc;

    tic;
    if isempty(d)
      continue
    endif
    sf = [];
    tt = [];    
    _b_df = d.df<0;
    if (sum(_b_df) == 0)
      continue;
    endif
    _x  = d.s(_b_df);
    _hh = d.hh(_b_df);
    _mm = d.mm(_b_df);
    for hh=0:23
      _b_hh = _hh==hh;
      if (sum(_b_hh) == 0)
	continue;
      endif
      _mmh  = _mm(_b_hh);
      _x2   = _x(_b_hh);
      for mm=0:59
	_b = _mmh==mm;
        if (sum(_b) < 10)
	  continue
	endif
	_x3=_x2(_b);
  	a_median(1+dn-t0,1+hh*60+mm) =median(_x3); 
  	a_std   (1+dn-t0,1+hh*60+mm) = std   (_x3);
  	a_mean  (1+dn-t0,1+hh*60+mm) = mean  (_x3);
      endfor
    endfor
    toc
  endfor
  a=struct("median", a_median, ...
	   "mean",   a_mean, ...
	   "std",    a_std);
endfunction

function d=read_data(stn, year, month, day)
  d=[];
  fn = sprintf("../../DataKRK/%s/y%d-m%02d-d%02d.txt",stn, year, month, day)
  fflush(stdout);
  fid = fopen(fn);
  do_fclose=1;
  if (fid < 0)
    fid = popen(["bzip2 -dc " fn ".bz2"], "r");
    do_fclose=0;
  endif
  if fgetl(fid) > 0
    fgetl(fid);
    fgetl(fid);
    fgetl(fid);
    [v,n] = fscanf(fid, "%d-%d-%d %d:%d:%f %f %f %f %f %f %f", [12 Inf]);
    if do_fclose
      fclose(fid);
    else
      pclose(fid);
    endif
    if (n>12)
      d = struct("year",  v( 1,:), ...		 
		 "month", v( 2,:), ...
		 "day",   v( 3,:), ...
		 "hh",    v( 4,:), ...
		 "mm",    v( 5,:), ...
		 "ss",    v( 6,:), ...
		 "f",     v( 7,:), ...
		 "s",     v( 9,:), ...
		 "df",    v(12,:));
    endif
  endif
endfunction