% loads a given image. name.jpg is the filename
% if show=0, then don't show. Otherwise show in figure(show)

% dependancy on matlab image proc. toolkit removed
% T. Breckon, 28/10/04

% requires the myrgb2gray matlab script

% N.B. returns a uint8 type image instead of double
% as with previous version. If required just
% use double(myjpgload(..,..) to get a double type image 
% returned.

function newimage = myjpgload(name,show)
  loaded = importdata(name,'jpg');
  newimage = myrgb2gray(loaded);
  if show > 0
      figure(show)
      colormap(gray)
      imagesc(newimage)
  end
