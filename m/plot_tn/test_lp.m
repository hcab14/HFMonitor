# -*- octave -*-
function test_lp
  N=100000;

  x=randn(N,1);
  filter_lp = @(x,lambda) filter([0 lambda], [1 lambda-1], x, x(1));

  alpha=[0.001 0.005 0.01 0.05 0.1 0.5];
  for i=1:length(alpha)
   std(filter_lp(x,alpha(i))(N/2:end))/sqrt(alpha(i)/(2-alpha(i))) 
  endfor
endfunction