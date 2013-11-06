# -*- octave -*-

function [as,qrgs,qrg,slope,slope_rms,c,chi2]=cal
  qrgs= { 60000 + [-1 0 1],    ...
	  75000 + [-2 -1 0 1], ...
	  77500 + [-1 0 1],    ...
	 162000 + [-2 -1 0 1]  ...
	 };
  as={};
  for i=1:length(qrgs)
    qs=qrgs{i};
    a=[];
    for j=1:length(qs)
      a(end+1,:)=read_arg(qs(j));
    endfor
    as{i}=a;
  endfor
  for i=1:length(qrgs)
    xxx = mod(diff(as{i}'), 1)/0.5;
##    xxx= mod(diff(as{i}')/0.5,1);
    plot(xxx, '*'); pause
    srms = std(xxx)
    [slope_rms(i), is] =  min(srms);
    slope(i)  = mean(xxx)(is);
    qrg(i)    = qrgs{i}(is);
  endfor

  c={}  
  chi2=[];
  for dq1=-2:2
    for dq2=-2:2
      for dq3=-2:2
	for dq4=-2:2
	  dq=[dq1 dq2 dq3 dq4];	
	  y=(qrg+dq-slope)';
	  x= [60000    75000    77500   162000]';
	  q=diag(slope_rms);
	  format long
	  [x1,x2]=gls(y,x,q);
	  fprintf("%3d %3d %3d %3d %f %e\n", dq(1), dq(2), dq(3), dq(4), 1e6*(1-x1), x2);
				#	plot(x,y-x1*x, '*');
	  c{end+1} = struct("dq", dq, ...
			    "y", x, ...
			    "dy", slope_rms, ...
			    "x", y, ...
			    "beta", x1, ...
			    "chi2", x2);
	  chi2(end+1)=x2;
	endfor
      endfor
    endfor
  endfor
endfunction

function a=read_arg(qrg)
  fid= popen(sprintf("awk '/^FP_ %.0f/{print $4}' test.dat", qrg), "r");
  [a,c]=fscanf(fid, "%f", [1 Inf]);
  fclose(fid);
endfunction

function [beta,beta_RMS]=lsqr(x,y,w)
  A=x;
  cov=inv(A'*diag(w)*A);
  beta=cov*A'*diag(w)*y;
endfunction