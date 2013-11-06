# -*- octave -*-

function d=read_data(fileName)
  fid=-1;
  switch fileName(end-2:end)
    case {"txt"}
      fid = fopen(fileName, "rt");
    case {"bz2"}
      fid = popen(["bzip2 -dc " fileName], "r");
    case {".gz"}
      fid = popen(["gzip -dc " fileName], "r");
    otherwise
      error ("invalid extension");
  endswitch

  l = "#";
  while length(l)<10 || ~strcmp(l(1:10), "# Time_UTC")
    l = fgetl(fid)
  endwhile
  [v,c] = fscanf(fid, "%d-%d-%d %d:%d:%f %f %f %f", [9,Inf]);
  fclose(fid);
  v=v';
  d.t = datenum(v(:,1), v(:,2), v(:,3), v(:,4), v(:,5), v(:,6));
  d.s = v(:,7);
endfunction