# -*- octave -*-

# FFT FIR filter design

function [b,y]=fd(N,w)
  y=zeros(1,N);  
  wN=round(w*N);
  y(1:wN)        =1;
  y(end-wN+2:end)=1;

  hamming=@(N) 0.54-0.46*cos(2*pi*[0:N-1]/(N-1));

  b= ifft(y);
  N2=floor(N/2);
  b= [ b(N2:end) b(1:N2-1)];
  b.*= hamming(N);
  b/= sum(b);
endfunction