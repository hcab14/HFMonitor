# -*- octave -*-

function [z,zc,c]=na2(N,f0,a, fc)

  [y,fs,bits] = wavread("../../DataKRKIQ/DC_298000_DGPS/y2013-m11-d08_H12M50.wav");
  z = y(:,1)+1i*y(:,2);

  osc = @(f,fs, N) exp(2*pi*i*f/fs *[0:N-1]');

  z .*= osc(fc, fs, length(z));

  d = fs/f0

  z=real(z(1:5000));

  c=zeros(2*N*d+1,1);
  for n=1:N;
    c(1+round(d*N)+round(n*d)) = c(1+round(d*N)-round(n*d)) = exp(-n*a);
  endfor
  c /= sum(c);

  zc = conv(z,c);

  
endfunction