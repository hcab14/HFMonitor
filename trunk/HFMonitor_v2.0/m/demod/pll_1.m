# -*- octave -*-

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
