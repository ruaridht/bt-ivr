%% Copyright (C) 2004 Josep Mones i Teixidor
%%
%% This program is free software; you can redistribute it and/or modify
%% it under the terms of the GNU General Public License as published by
%% the Free Software Foundation; either version 2 of the License, or
%% (at your option) any later version.
%%
%% This program is distributed in the hope that it will be useful,
%% but WITHOUT ANY WARRANTY; without even the implied warranty of
%% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
%% GNU General Public License for more details.
%%
%% You should have received a copy of the GNU General Public License
%% along with this program; if not, write to the Free Software
%% Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

%% Perform an erosion morphological operation on a binary image.
%%
%% BW2 = erosion(BW1, SE) returns a binary image with the result of an erosion
%% operation on BW1 using neighbour mask SE.
%%
%% For each point in BW1, erode searchs its neighbours (which are
%% defined by setting to 1 their in SE). If all neighbours
%% are on (1), then pixel is set to 1. If any is off (0) then it is set to 0.
%%
%% Center of SE is calculated using floor((size(SE)+1)/2).
%%
%% Pixels outside the image are considered to be 0.
%%
%% BW2 = erode(BW1, SE, alg) returns the result of a erosion operation 
%% using algorithm alg. Only 'spatial' is implemented at the moment.
%%
%% BW2 = erosion(BW1, SE, ..., n) returns the result of n erosion
%% operations on BW1.
%%

%% Author:  Josep Mones i Teixidor <jmones@puntbarra.com>

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Matlab Port - T. Breckon 22/11/04

%% Notes for IVR course - define SE as an NxM matrix and not a
%% strel object (as strel belongs to the image proc. toolkit)

%% i.e. use:

%%  SE = ones(3,3);
%%  ErrodedImage = myimerode(BinaryImageInput,SE);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function BW2 = myimerode(BW1, SE, a, b)
  alg='spatial';
  n=1;
  if (nargin < 1 || nargin > 4)
    %% usage ("BW2 = erode(BW1, SE [, alg] [, n])");
  end
  if nargin ==  4
    alg=a;
    n=b;
  elseif nargin == 3
    if isstr(a)
      alg=a;
    else
      n=a;
    end
  end 

  %% if ~strcmp(alg, 'spatial')
    %% error("erode: alg not implemented.");
  %% end

 %% count ones in mask
  thr=sum(SE(:));

 %% "Binarize" BW1, just in case image is not [1,0]
  BW1=BW1~=0;

  for i=1:n
    %% create result matrix
    BW1=filter2(SE,BW1) == thr;
  end

  BW2=BW1;

%!demo
%! erode(ones(5,5),ones(3,3))
%! % creates a zeros border around ones.
%!assert(erode([0,1,0;1,1,1;0,1,0],[0,0,0;0,0,1;0,1,1])==[1,0,0;0,0,0;0,0,0]);
%!assert(erode([0,1,0;1,1,1;0,1,0],[0,1;1,1])==[1,0,0;0,0,0;0,0,0]);
