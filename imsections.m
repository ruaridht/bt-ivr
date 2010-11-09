%
%
%
%
function imsect = imsections(image, secLoc)
    
    bwlabeled = bwlabel(image,4);
    [u,v] = find(bwlabeled==secLoc);
    
    imsect = imcomponent(u,v);
    
    %imshow(imsect);
    
end