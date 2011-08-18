# -*- octave -*-
# $Id$

function d=proc_vlf(year, month, day)
  d=ps(glob(sprintf("Data/_VLF_/y%d-m%02d-d%02d_H??M??.mat", year, month, day)));

  qrgs=[23400 24000];

  mkdir Data/_VLFdata_;
  mkdir Data/_VLFplots_;

  v = datevec(d.time);
  
  for hh=0:23
    b = (v(:,4)==hh);
    if sum(b) == 0 
      continue 
    endif
    ds = datestr(d.time(b), "HH:MM:SS");
    for j=1:length(qrgs)
      filename = sprintf("Data/_VLFdata_/%05dHz_%4d-%02d-%02d_H%02d.txt", qrgs(j), year,month,day,hh)
      vs = sprintf(";%7.2f\n", d.spec(b,find(d.freq_Hz==qrgs(j))));
      dd = [ds, reshape(vs, 9, length(vs)/9)'];      
      fid=fopen(filename, "w"); fwrite(fid, dd'); fclose(fid);

      plot(d.time,d.spec(:,find(d.freq_Hz==qrgs(j))));  
      datetick('x','HH:MM:SS'); 
      ylim([-120 -0]); 
      xlabel(sprintf("UTC of %d-%02d-%02d", year, month, day));
      ylabel("dB");
      title(sprintf("%5.1f kHz", qrgs(j)/1000));

      filename = sprintf("Data/_VLFplots_/%05dHz_%4d-%02d-%02d_H%02d.png", qrgs(j), year,month,day,hh)
      print(filename, "-dpng");

    endfor

    bf = d.freq_Hz>15e3 & d.freq_Hz<25e3;
    imagesc(d.freq_Hz(bf), d.time(b), d.spec(b, bf)); 
    datetick('y','HH:MM:SS'); 
    xlabel("frequency / Hz");
    ylabel(sprintf("UTC of %d-%02d-%02d", year, month, day));
    filename = sprintf("Data/_VLFplots_/SPEC____%4d-%02d-%02d_H%02d.png", year,month,day,hh)
    print(filename, "-dpng");
  endfor
endfunction