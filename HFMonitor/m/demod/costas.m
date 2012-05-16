# -*- octave -*-

function [theta,f,e,in,s]=costas(in,fc,fs,alpha,beta, mode,n)  
  ##Initialization 
  theta(1)= .0;
  f(1) = fc/2;
  e(1)=0;
  x=abs(in(1));

  y0=y1=0;
  w0=w1=0;

  phase_diff=0;
# [f,H,b]=raised_cosine(50,0.5,1/2400/2,fs);
  [fb,fa]=cheby1(15,0.1,.4);
  SF=zeros(15,1);
  sig=0;

  for I=2:length(in)
    [c(I-1),SF] = filter(fb,fa, in(I-1) * exp(-i*theta(I-1)), SF);
    c(I-1) = in(I-1);

    if (strcmp(mode,"AM"))
      x=0.99*x + 0.01*abs(in(I-1));
      in(I-1) /= x;
      phase_diff = imag(c(I-1));
      e(I) = sign(real(c(I-1))) * phase_diff;
    else
      mu=1.0;
      R=0.999; # dc block
      
#      y1=c(I-1);
#      w1 = R*w0 + y1 - y0;
#      w0=w1; y0=y1;
      
#      phase_diff = imag(log(filter(b,1,c)(end)**n))/n;
#      phase_diff = (1-mu)*phase_diff+ mu*imag(log(c(I-1)**n))/n;
      
      phase_diff = (1-mu)*phase_diff+ mu * imag(log(c(I-1)));
      
#      phase_diff-= round((phase_diff-e(I-1)-2*pi*f(I-1)/fs)/2/pi*n)*2*pi/n;
#      phase_diff += 4*pi*f(I-1)/fs;

      ## demodulated signal 
      s(I) = round(phase_diff/pi/2*n);
      e(I) = phase_diff - s(I)*2*pi/n;

    endif

    f(I)     = f(I-1) + alpha*e(I-1);

    theta(I) = theta(I-1) + alpha*e(I-1) + beta*f(I-1);

#    if abs(theta(I))>pi
#      theta(I) = theta(I)-2*pi*sign(theta(I));
#    endif
  endfor

endfunction

