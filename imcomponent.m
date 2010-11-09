function newimage = imcomponent(r,c)
    m = length(r);
    n = length(c);
    for a = 1 : 480
       for b = 1 : 640
           newimage(a,b) = 0;
       end
    end
    
    for x = 1:m
        newimage(r(x),c(x)) = 1;
    end
end