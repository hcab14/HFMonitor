# -*- octave -*-

function [xt,yt,ipBit,bHat,baud,fsHz]=gen_msk(Eb_N0_dB,fc)
  N = 5*10^3; % number of bits or symbols
  
  fsHz = 1; % sampling period
  T    = 4; % symbol duration

  baud=1/T;

  ct = cos(pi*[-T:N*T-1]/(2*T));
  st = sin(pi*[-T:N*T-1]/(2*T));

  %% MSK Transmitter
  ipBit = rand(1,N)>0.5; % generating 0,1 with equal probability
  ipMod =  2*ipBit - 1; % BPSK modulation 0 -> -1, 1 -> 0
  
  ai = kron(ipMod(1:2:end),ones(1,2*T));  % even bits
  aq = kron(ipMod(2:2:end),ones(1,2*T));  % odd bits
  
  ai = [ai zeros(1,T)  ]; % padding with zero to make the matrix dimension match 
  aq = [zeros(1,T) aq ];  % adding delay of T for Q-arm
  
  %% MSK transmit waveform
  xt = 1/sqrt(T)*[ai.*ct + j*aq.*st];

  xt .*= exp(2*pi*i*fc*[-T:N*T-1]);

  %% Additive White Gaussian Noise
  nt = 1/sqrt(2)*[randn(1,N*T+T) + j*randn(1,N*T+T)]; % white gaussian noise, 0dB variance 
  
  %% Noise addition
  yt = xt + 10^(-Eb_N0_dB/20)*nt; % additive white gaussian noise
  
  %% MSK receiver 
  %% multiplying with cosine and sine waveforms
  xE = conv(real(yt).*ct,ones(1,2*T));
  xO = conv(imag(yt).*st,ones(1,2*T));
  
  bHat = zeros(1,N);
  bHat(1:2:end) = xE(2*T+1:2*T:end-2*T) > 0 ; % even bits
  bHat(2:2:end) = xO(3*T+1:2*T:end-T) > 0 ;  % odd bits
  
  %% counting the errors
  nErr = size(find([ipBit - bHat]),2)
endfunction