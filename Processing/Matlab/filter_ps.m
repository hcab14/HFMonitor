# -*- octave -*-
# $Id$


function ps_f=filter_ps(d)
  ps_f = struct("time", [], "spec", [], "freq_Hz", d.freq_Hz);
  v=datevec(d.time);
  idx=0;
  while idx != length(d.time)
    d0=v(idx+1,1:5);
    b=v(:,1)==d0(1) & v(:,2)==d0(2) & v(:,3)==d0(3) & v(:,4)==d0(4) & v(:,5)==d0(5);    
    ps_f.time(end+1)   = mean(d.time(b));
    ps_f.spec(end+1,:) = mean(d.spec(b,:));    
    idx= find(b)(end);
  endwhile
endfunction 
function ps_f=filter_ps_(d)
  ps_f = struct("time", [], "spec", [], "freq_Hz", d.freq_Hz);
  while length(d.time) > 0
    v=datevec(d.time);
    d0=v(1,1:5);
    b=v(:,1)==d0(1) & v(:,2)==d0(2) & v(:,3)==d0(3) & v(:,4)==d0(4) & v(:,5)==d0(5);    
    ps_f.time(end+1)   = mean(d.time(b));
    ps_f.spec(end+1,:) = mean(d.spec(b,:));    
    d.time= d.time(~b);
    d.spec= d.spec(~b,:);
  endwhile
endfunction 
