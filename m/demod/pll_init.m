# -*- octave -*-

function s=pll_init(xi, dwl, fc, fs)
  wl= dwl*2*pi;
  wn= wl/2/xi;
  
  k0=kd=1;
  
  tau1= k0*kd/wn**2;
  tau2= xi*2/wn;
  
  s.ts= 1/fs;
  s.fc= fc;

  ## loop filter coefficients from s -> z transform with prewarping
  s.a= [1 -1];
  s.b= s.ts/2/tau1*(1+ [1 -1]/tan(s.ts/2/tau2));
  s.ud    = 0;
  s.uf    = 0;
  s.f1    = 2*pi * s.fc;
  s.theta = -s.f1 * s.ts;
endfunction
