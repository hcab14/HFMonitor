# -*- octave-mode -*-
# $Id$

function d=ana_efr2
  stns = {"L2.129100_EFR_Mainflingen", ...
          "L2.135600_EFR_Lakihegy", ...
          "L2.139000_EFR_Burg"};  
  limits = {[-70 -40],
	    [-70 -40],
	    [-70 -40]};
  year=2014;

  for i=1:length(stns)
    d{i} = load(sprintf("%d_%s.mat", year, stns{i}));
    plot_year(d{i}.a.median, year, stns{i}, limits{i});
  endfor
endfunction

function plot_year(a, year, stn, lim)
  imagesc([1:1440]/60, [1:size(a,1)], a, lim);
  set(gca, 'xtick', [0:4:24]);
  title(strrep(stn, "_", "\\_"));
  xlabel("UPC (h)");
  ylabel(sprintf("day of year %d", year));
  colorbar();
  print(sprintf("%d_%s.eps", year, stn),"-deps");
endfunction