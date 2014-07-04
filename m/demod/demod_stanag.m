# -*- octave -*-

function [z,theta,ud,uf,fs]=demod_stanag(df)
  filename="mode-sta4285.wav";
#  filename="mode-nato-stanag-4285-sending-crypto-msg.wav";
#  filename="s4285_081020_2200_8r6268_600lita2_fnav_fum.wav";
 [y,fs,bits] = wavread(filename);
 fs
 y=y(2000:30000);
 z=hilbert(y);

 if (0)
   N = 1025; 
   n = 0:N-1; 
   b = (4./( 2*pi*(n-(N-1 )/2))) .* (rem(n,2)~=0);
   b(round(N/2))=0;
   
   y2=filter(b,1,y);
   
   z=y(1:end-512) + i*y2(513:end);
 endif
   
 osc = @(f, N) exp(2*pi*i*f/fs *[0:N-1]');
 z .*= osc(-1800,length(z));

 b=fir1(2000,2800/fs);
 z=fftfilt(b,z);

 z .*= osc(df,length(z));

 z /= mean(abs(z));

 if (0)
   idx=1+floor((0:floor(80/2400*fs)-1)/fs*2400);
   z_ref=gen_80()(idx);
   
   for j=1:100
     p(j)=2*pi/100*j;
     [a1(j),a2(j)]=max(imag(corr_(z_ref*exp(i*p(j)),z)));
   endfor
 endif


 ## critical damping
 xi=1/sqrt(2);

 ## lock range dwl Hz
 dwl=30;

 [theta,ud,uf]=pll(z, xi, dwl, df, fs);

  ## alpha=0.1;
  ## beta=alpha**2/4;
  ## [theta,f,e,zf,s]= costas(z,df*.45594,fs,alpha,beta,"QPSK",8);
endfunction

function c=corr_(x,y)
  N=length(x);
  for i=1:2000
    c(i) = x*y(1+i-1:N+i-1);
  endfor
endfunction

function z=gen_80
  x=uint32(sum(bitshift(1,[1 3 4])));
  for i=1:80
    z(i)=2*(double(bitand(x,1))-0.5);
    x=_lfsr(x);
  endfor
endfunction

function z=gen_xx

endfunction

function x=_lfsr(x)
  bit=mod(sum(bitget(x,[1 3])),2);
  x=bitshift(x,-1);
  x=bitset(x,5,bit);
endfunction

function x=_lfsr2(x)
  bit=mod(sum(bitget(x,[1 5])),2);
  x=bitshift(x,-1);
  x=bitset(x,9,bit);
endfunction