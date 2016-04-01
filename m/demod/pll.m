# -*- octave -*-

function [theta,ud,uf,f1]=pll(s, xi, dwl, fc, fs)
  ## lock range dwl Hz
  wl= dwl*2*pi;
  wn= wl/2/xi;

  k0=kd=1;

  tau1= k0*kd/wn**2;
  tau2= xi*2/wn;

  ts= 1/fs;

  ## loop filter coefficients from s -> z transform with prewarping
  a= [1 -1];
  b= ts/2/tau1*(1+ [1 -1]/tan(ts/2/tau2));

  ud(1)    = 0;
  uf(1)    = 0;
  theta(1) = 0;
  f1(1)    = 2*pi*fc;
  for n=2:length(s)
    ## phase update
    theta(n) = theta(n-1) + f1(n-1)*ts;

    ## keep phase bounded
#    if (theta(n)>pi) theta(n) -= 2*pi; endif

    ## phase detector
    ud(n) = imag(log(s(n)*exp(-i*theta(n))));

#    ud(n) -= round(ud(n)/pi/2*8)/8*2*pi;

    ## loop filter (1 + s*tau1) / (s*tau2)
    uf(n) = -a(2)*uf(n-1) + b(1)*ud(n) + b(2)*ud(n-1);

    ## nco frequency update
    f1(n) = 2*pi*fc + uf(n);
  endfor
endfunction
