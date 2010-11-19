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

%% Perform a dilation morphological operation on a binary image.
%%
%% BW2 = dilate(BW1, SE) returns a binary image with the result of a dilation
%% operation on BW1 using neighbour mask SE.
%%
%% For each point in BW1, dilate search its neighbours (which are
%% defined by setting to 1 their in SE). If any of its neighbours
%% is on (1), then pixel is set to 1. If all are off (0) then it is set to 0.
%%
%% Center of SE is calculated using floor((size(SE)+1)/2).
%%
%% Pixels outside the image are considered to be 0.
%%
%% BW2 = dilate(BW1, SE, alg) returns the result of a dilation operation 
%% using algorithm alg. Only 'spatial' is implemented at the moment.
%%
%% BW2 = dilate(BW1, SE, ..., n) returns the result of n dilation
%% operations on BW1.
%%

%% Author:  Josep Mones i Teixidor <jmones@puntbarra.com>

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Matlab Port - T. Breckon 22/11/04

%% Notes for IVR course - define SE as an NxM matrix and not a
%% strel object (as strel belongs to the image proc. toolkit)

%% i.e. use:

%%  SE = ones(3,3);
%%  DilatedImage = myimdilate(BinaryImageInput,SE);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function BW2 = myimdilate(BW1, SE, a, b)
  alg='spatial';
  n=1;
  %%if (nargin < 1 || nargin > 4)
    %% usage ("BW2 = dilate(BW1, SE [, alg] [, n])");
  %%end
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

  %%if ~strcmp(alg, 'spatial')
  %%  error("dilate: alg not implemented.");
  %%end

  %% "Binarize" BW1, just in case image is not [1,0]
  BW1=BW1~=0;

  for i=1:n
    %% create result matrix
    BW1=filter2(SE,BW1)>0;
   end

  BW2=BW1;

%!demo
%! dilate(eye(5),ones(2,2))
%! % returns a thick diagonal.



%!assert(dilate(eye(3),[1])==eye(3));	# using [1] as a mask returns the same value
%!assert(dilate(eye(3),[1,0,0])==[[0;0],eye(2);0,0,0]);
%!assert(dilate(eye(3),[1,0,0,0])==[[0;0],eye(2);0,0,0]); # test if center is correctly calculated on even masks
