# -*- octave -*-

function [f,H,b]=raised_cosine(N,beta,T,fs)
  f=[0:N/2-1 -N/2:-1]*fs/N;

  f1=(1-beta)/2/T;
  f2=(1+beta)/2/T;

  H=f*0;
  H(abs(f) <= f1) = T;
  b=abs(f) >= f1 & abs(f) <= f2;
  H(b) = T/2*(1+cos(pi*T/beta*(abs(f(b))-f1)));

  b=ifft(H);
  b=real(b);
  b/=sum(b);
endfunction