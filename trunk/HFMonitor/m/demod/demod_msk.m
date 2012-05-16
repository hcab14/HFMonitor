# -*- octave -*-

function [bits,bitsE,bitsO,btE,btO,xE,xO,ct,st,theta0,theta1,theta2]=demod_msk(z, baud, fs)
  xi=1/sqrt(2);

  ## timing
  ## z.**2 has components at +- 2*fm, fm=1/(4*T)=baud/4
  pll_plus  = pll_init(xi, 0.05*baud/2,  baud/2, fs);
  pll_zero  = pll_init(xi, 0.05*baud/2,  0,      fs);
  pll_minus = pll_init(xi, 0.05*baud/2, -baud/2, fs);

 for n=1:length(z)
    zn = z(n); #*exp(-i*(pll_zero.theta + pll_zero.f1*pll_zero.ts)/2*0);
    pll_plus  = pll_1(zn**2, pll_plus);
    theta1(n) = pll_plus.theta;
    ud1(n) = pll_plus.ud;
    uf1(n) = pll_plus.uf;
    f11(n) = pll_plus.f1;

    pll_minus = pll_1(zn**2, pll_minus);
    theta2(n) = pll_minus.theta;
    ud2(n) = pll_minus.ud;
    uf2(n) = pll_minus.uf;
    f12(n) = pll_minus.f1;

    pll_zero = pll_1(exp(i*theta1(n)+i*theta2(n)), pll_zero);
    theta0(n) = pll_zero.theta;
    ud0(n) = pll_zero.ud;
    uf0(n) = pll_zero.uf;
    f10(n) = pll_zero.f1;
  endfor

#  theta0=theta1+theta2;

  ct=cos((theta1-theta0/2)/2);
  st=sin((theta1-theta0/2)/2);

  xE=real(z .* exp(-i*theta0/4)) .* ct;
  xO=imag(z .* exp(-i*theta0/4)) .* st;

  _sign=@(x) 1-2*(x<0);

  bits=bitsE=bitsO=[];
  bits_t=[];
  sE=sign(ct(1)); 
  sO=sign(st(1));
  sumE=xE(1); 
  sumO=xO(1);
  for n=2:length(z)-1
    ## ct <-> xE
    if _sign(ct(n)) == sE
      sumE += xE(n);
    else
      bits(end+1) = bitsE(end+1:end+2) = sumE>0;
      if isempty(bitsO) bitsO=true; endif
      bits_t(end+1)=n;
      sumE=0; 
      sE=_sign(ct(n));
    endif
    ## st <-> xO
    if _sign(st(n)) == sO
      sumO += xO(n);
    else
      bits(end+1) = bitsO(end+1:end+2) = sumO>0;
      if isempty(bitsE) bitsE=true; endif
      bits_t(end+1)=n;
      sumO=0; 
      sO=_sign(st(n));
    endif
  endfor
  _N=min(length(bitsE), length(bitsO));
  bitsE=bitsE(2:_N-1);
  bitsO=bitsO(2:_N-1);
  bt =zeros(1,length(z)*40);
  btE=zeros(1,length(z)*40);
  btO=zeros(1,length(z)*40);
  for n=3:length(bits)
    bt(40*bits_t(n-1)+1:40*bits_t(n)) = 2*bits(n)-1;
    if mod(n,2) == 1
      btE(40*bits_t(n-2)+1:40*bits_t(n)) = 2*bits(n)-1;
    else
      btO(40*bits_t(n-2)+1:40*bits_t(n)) = 2*bits(n)-1;
    endif
  endfor
endfunction