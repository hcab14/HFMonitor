# -*- octave -*-

function [s,theta,ud,uf,f1,theta2,ud2,uf2,f12]=pll2
  N=16000;
  fs=10000;

  fc=2000;
  
  p0 = pi/4
  s = exp(i*2*pi*(fc-0)/fs*[0:N-1] + i*p0);
  m =cos(2*pi*100/fs*[0:N-1]) + 0.2*sin(2*pi*300/fs*[0:N-1]) + 0.7* sin(2*pi*150/fs*[0:N-1]);
  m=1;
  s .*= m;
  s=awgn(s,15);

  ## critical damping
  xi=1/sqrt(2);

  ## lock range 100 Hz
  dwl=100;

  [theta,ud,uf,f1]=pll(s, xi, dwl, fc, fs);

  ps=pll_init(xi, dwl, fc, fs);
  
  for n=1:length(s)
    ps = pll_1(s(n), ps);
    theta2(n) = ps.theta;
    ud2(n) = ps.ud;
    uf2(n) = ps.uf;
    f12(n) = ps.f1;
  endfor

endfunction


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
  s.f1    = 2*pi*s.fc;
  s.theta = -s.f1*s.ts;
endfunction

function s=pll_1(z, s)
  ## phase update
  s.theta += s.f1*s.ts;
  
  ## phase detector
  ud_old = s.ud;
  s.ud = imag(log(z*exp(-i*s.theta)));
  
  ## loop filter (1 + s*tau1) / (s*tau2)
  s.uf = -s.a(2)*s.uf + s.b(1)*s.ud + s.b(2)*ud_old;
  
  ## nco frequency update
  s.f1 = 2*pi*s.fc + s.uf;
endfunction
