%% converts image uint8 greyscale image I to a black-white (binary) image.
%% The threshold value t should be in the range [0,1].

%% T. Breckon, 19/10/04

function BINARY = myim2bw (I, t)
      BINARY = (I >= (t * 255));
