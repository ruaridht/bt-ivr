function newimage = binarypic(name,mod)
    binary_pic = myjpgload(name,0);
    [m,n] = size(binary_pic);
    threshold = (mean(binary_pic,2))*mod;
    newimage = binary_pic;
    for r = 1 : m
       for c = 1 : n
               if (binary_pic(r,c) > threshold)
                       newimage(r,c) = 0;
               else

               newimage(r,c) = 255;

           end
       end
end