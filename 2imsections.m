%
%
%
%
function imsect = imsections(bwlabeled, secLoc)
    
    [u,v] = find(bwlabeled==secLoc);
    
    m = length(u);
    n = length(v);
    
    imsect = zeros(640,480);
    
    for x = 1:m
        imsect(u(x),v(x)) = 1;
    end
     
end