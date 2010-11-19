function newimage = imcomponent(r,c)
    m = length(r);
    n = length(c);
    
    newimage = zeros(640,480);
    
    for x = 1:m
        newimage(r(x),c(x)) = 1;
    end
end