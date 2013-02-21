# -*- octave -*-

function d=filter_data(d, dt)
  if nargin == 1
    dt=60;  ## default time step
  endif

  ## time intervals [tt(i) tt(i+1)]
  tt = floor(d.t(1))+[0:dt:3600*24]/24/3600;

  N   = length(d.t);
  idx = find(d.t>=tt(1),1);
  for i=1:length(tt)-1;    
    ## we consider only a minimal range of the data
    idx2 = find(d.t>tt(i+1), 1);
    if isempty(idx2) idx2=N; endif
    range = idx:idx2;
    t_in_range = d.t(range);
    s_in_range = d.s(range);
    b = t_in_range>=tt(i) & t_in_range<tt(i+1);
    t_in_bin = t_in_range(b);
    s_in_bin = s_in_range(b);
    n_in_bin = length(s_in_bin);
    if n_in_bin > 0
      d.filter_xx_t(i)   = mean(t_in_bin);
      d.filter_xx_s(i)   = median(s_in_bin);
      d.filter_xx_rms(i) = std(s_in_bin);
    else
      d.filter_xx_t(i)   = tt(i)+0.5*dt/24/3600;
      d.filter_xx_s(i)   = -1000;
      d.filter_xx_rms(i) = 0;
    endif
    idx += n_in_bin;
  endfor
endfunction