# -*- octave -*-

function [bits,bitsE,bitsO,btE,btO,xE,xO,ct,st,theta0,theta1,theta2,f10,z,fs]=test_msk
  addpath("../demod");
  [y,fs,bits] = wavread ("../../DataKRKIQ/DC_298000_DGPS/y2013-m11-d06_H18M10.wav");

  z=y(:,1)+1i*y(:,2);

  baud=100;
  bw= 3*baud;
  
  osc = @(f,fs, N) exp(2*pi*i*f/fs *[0:N-1]');
  fc = -11/4-(0.45245+0.0219+0.0065511-0.016809-0.0052643)/4;
  f0 = 0;
  z .*= osc(f0-fc, fs, length(z));
  
  b= fir2(5000,[0 bw/fs 1.1*bw/fs 1], [1 1 0 0 ], 2, 20);
  b=conv(b,b);
  z=fftfilt(b,z);

  if 0
    f=[0:2500 -2499:-1];
    for i=1:300
      s(:,i)=abs(ifft(z(1+(i-1)*5000:i*5000)));
    endfor;
    plot(f,log10(mean(s')));
  endif

  z=z(1:5000*30);

  decim=5;
  z=z(1:decim:end);
  fs /= decim;

  z=z';

  [bits,bitsE,bitsO,btE,btO,xE,xO,ct,st,theta0,theta1,theta2,f10]=demod_msk(z, baud, fs);

#  bbits=bitxor(bitsE, bitsO)

#  decode_rtcm(bbits);

endfunction