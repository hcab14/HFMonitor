# -*- octave -*-

function [ipBit,bits]=test_msk
  [z,yt,ipBit,bHat,baud,fs]=gen_msk(0,0.01);

  zf=fftfilt(fir1(1000, 2*baud/fs), z);
  zf=zf(500:end);

  [bits,xE,xO,ct,st,theta0,theta1,theta2]=demod_msk(zf, baud, fs);

  N=min(length(bits),4800);
  length(find([bits(end-N+1:end)!=ipBit(end-N-125:end-1-125)]))
endfunction