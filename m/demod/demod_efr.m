# -*- octave -*-

function [z,zf,fs]=demod_efr
  fn = "wav/iq_135kHz.wav";
  [y,fs,bits]=wavread(fn);
  fs
  z = y(:,1)+1i*y(:,2);

  f0 = 1e3*sscanf(fn(end-9:end-7), "%f");

  fc = 139e3;

  osc = @(f,fs, N) exp(2*pi*i*f/fs *[0:N-1]');
  z .*= osc(f0-fc, fs, length(z));

  bw= 1.8e3;
  b= fir2(5000,[0 bw/fs bw/fs 1], [1 1 0 0 ], 2, 20);
  b=conv(b,b);
  z=fftfilt(b,z,10**5);

  decim=20;
  z=z(1:decim:end);
  fs /= decim;

  b1 = fir2(32, [0 100/fs 100/fs 1], [1 1 0 0]);
  zf1=filter(b1,1,z .* osc( 170, fs, length(z)));
  zf2=filter(b1,1,z .* osc(-170, fs, length(z)));

  n=8204:10200;
  n=1:120000;
#  n=21702:23000;
#  n=22202:22600;
#  zf1=conv(zf1,ones(1,n_period));
#  zf2=conv(zf2,ones(1,n_period));
  s=2*(abs(zf1(n))>abs(zf2(n)))-1;

  clear early late err t
  n_period = fs/200
  err(1)=0;
  t(1)=2*n_period;
  n=2;
  alpha=0.5;
  while floor(t(n-1)) < length(s)
    m=floor(t(n-1));
    early(n) = sum(s(m-2*n_period+1:m-n_period));
    late(n)  = sum(s(m-  n_period+1:m));
    err(n)   = (1-alpha)*err(n-1)- alpha*(abs(early(n))-abs(late(n)));
    t(n)     = t(n-1) + n_period + 0.5*err(n);
    n += 1;
  endwhile

  t = floor(t)-n_period/2;
  bits=s(t)==1;

  [bytes,parity,bbits,ns]=decode_efr(bits);
endfunction