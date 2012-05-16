# -*- octave -*-

function plot_msk_check(zs, btE, btO, ct,st, xE,xO)
  counter=1;
  N=6;
  for n1=2.23e6:50000:length(zs)-50000
    n2=n1+50000-1;
    if counter > N
      counter=1; pause;
    endif
    subplot(N,1,counter);
    n=n1:n2;
    plot(n1:40:n2, diff(unwrap(imag(log(zs(n1:n2+1)))))(1:40:end), ...
	 n1:40:n2, -(btE(n).*btO(n))(1:40:end)*0.005,'r');
    ylim(0.01*[-1 1]);

    m=n1/40:n2/40; 
    subplot(N,1,counter+1);
    plot(n1:40:n2, ct(m)*0.0001,'g', n1:40:n2, xE(m))
    ylim(0.0003*[-1 1]);
    
    subplot(N,1,counter+2);
    plot(n1:40:n2, st(m)*0.0001,'g', n1:40:n2, xO(m))
    ylim(0.0003*[-1 1]);
    counter += 3;
  endfor

  m=n1/40:n2/40; 
  

endfunction