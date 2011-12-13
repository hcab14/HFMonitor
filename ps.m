% -*- octave -*-
% $Id$

%
% filenames: cell array of input file names
%            e.g. d=ps({'Data/_49m_A_/y2011-m05-d25_H13M00.mat'});
%
function d=ps(filenames)
  time={};
  spec={};
  for j=1:length(filenames)
    s=load(filenames{j});
    printf('loading %s ...\n', filenames{j});fflush(stdout);
    fns=sort(fieldnames(s));
    d.freq_Hz=s.freq_Hz;
    for i=1:length(fns)
      if fns{i}(1:4) == 'spec'
	tl=['time_' fns{i}(6:9)];
	time{1,end+1}= s.(tl)';
	spec{1,end+1}= s.(fns{i})';
      endif
    end
  end
  d.time= cell2mat(time)';
  % converting int16 -> double
  d.spec= 0.01*double(cell2mat(spec)');
end
