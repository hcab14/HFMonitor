# -*- octave -*-

function [y,b,yf,x]=hilb
  [y,fs,bits] = wavread("mode-sta4285.wav");
  size(y)
  y=y(1:10000);
  [b,yf]=hilb_filt(1025);

  x=filter(b,1,y);
#  plot(abs(fft(y+i*x)));
endfunction

function [b,y]=hilb_filt(N)
  y=zeros(1,N);  

  N2=floor(N/2);
  y(1) =0;
  y(2:N2+1)=1;
  y(N2+2:end)=-1;
  [N2-2+1 length(y)-N2-2]


  hamming=@(N) 0.54-0.46*cos(2*pi*[0:N-1]/(N-1));

  b= ifft(y);
  b= [ b(N2+1:end) b(1:N2)];
  b.*= hamming(N);
#  b/= sum(hamming(N));
endfunction