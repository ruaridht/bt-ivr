% Trim: takes a binary image and returns the N max areas present in the
% image.
% N = 11
% 
% 
%
function cornerformat = trim(file)
    
    bim = binarypic(file);

    bwlabeled = bwlabel(bim,4);
    regiondata = regionprops(bwlabeled, 'Area');
    
    areas = [regiondata.Area];
    
    m = length(areas);

    arraymajig = [0 0 0 0 0 0 0 0 0 0 0 0];

    n = length(arraymajig);
    
    for a = 1 : n
        maxed = 1;
        for b = 2 : m
           if (areas(b) > areas(maxed))
               maxed = b;
           end
           
        end
        arraymajig(a) = maxed;
        areas(maxed) = 0;
    end
    
    maxareas = arraymajig;
    
    %com1 = imageprops(bim, arraymajig(1));
    %com2 = imageprops(bim, arraymajig(2));
    %com3 = imageprops(bim, arraymajig(3));
    %com4 = imageprops(bim, arraymajig(4));
    %com5 = imageprops(bim, arraymajig(5));
    %com6 = imageprops(bim, arraymajig(6));
    %com7 = imageprops(bim, arraymajig(7));
    %com8 = imageprops(bim, arraymajig(8));
    %com9 = imageprops(bim, arraymajig(9));
    %com10 = imageprops(bim, arraymajig(10));
    %com11 = imageprops(bim, arraymajig(11));
    %com12 = imageprops(bim, arraymajig(12));
    
    %imagey = imsections(bim,arraymajig(11));
    %areay = bwarea(imagey);
    %imagey(303,522) = 0;
    %imshow(imagey);
    
    %coms = [com1; com2; com3; com4; com5; com6; com7; com8; com9; com10; com11; com12];
    %coms = com1;
    
    lowcompacsloc = [0 0 0 0 0 0];
    compacs = [0 0 0 0 0 0 0 0 0 0 0 0];
    
    for c = 1 : 12
        compacs(c) = compacty(bim, arraymajig(c));
    end
    
    for a = 1 : 6
        maxed = 1;
        for b = 2 : 12
            if (compacs(b) < compacs(maxed))
                maxed = b;
            end
        end
        lowcompacsloc(a) = arraymajig(maxed);
        compacs(maxed) = 100;
    end
    
    cornerlocs = [0 0 0 0 0];
    lowcompacsarea = [0 0 0 0 0 0];
     
    for a = 1 : 6
        lowcompacsarea(a) = bearea(imsections(bim,lowcompacsloc(a)));
    end
    
    for a = 1 : 5
        maxindex = 1;
        for b = 2 : 6
            if (lowcompacsarea(b) > lowcompacsarea(maxindex))
                maxindex = b;
            end
        end
        cornerlocs(a) = lowcompacsloc(maxindex);
        lowcompacsarea(maxindex) = 0;
    end
    
    c1com = centerofmass(bim,cornerlocs(1));
    c2com = centerofmass(bim,cornerlocs(2));
    c3com = centerofmass(bim,cornerlocs(3));
    c4com = centerofmass(bim,cornerlocs(4));
    c5com = centerofmass(bim,cornerlocs(5));
    %c6com = centreofmass(bim,cornerlocs(6));
    
    %imagey = imsections(bim,cornerlocs(1));
    %imagey(c1com(1),c1com(2)) = 0;
    %imshow(imagey);
    
    cornercoms = [c1com; c2com; c3com; c4com; c5com];
    
    % corners in format [bl br tl tr] (portrait, robot at bottom)
    
%    cornerformat = [0 0;0 0;0 0; 0 0;];
    
    atob = myeuclid(c1com, c2com);
    atoc = myeuclid(c1com, c3com);
    atoe = myeuclid(c1com, c5com);
    
    if atob > atoc
        
        if atoe > myeuclid(c2com,c5com)
            
            cornerformat = [c2com; c4com; c1com; c3com];
%             cornerformat(1) = c2com;
%             cornerformat(2) = c4com;
%             cornerformat(3) = c1com;
%             cornerformat(4) = c3com;
            words = 'case a'
% tests 1,3
        else

            cornerformat = [c3com; c4com; c2com; c1com];
%             cornerformat(1) = c1com;
%             cornerformat(2) = c3com;
%             cornerformat(3) = c2com;
%             cornerformat(4) = c4com;
            words = 'case b'
        end
%test 5        
    else     
        
        if atoe > myeuclid(c3com,c5com)
         
            cornerformat = [c4com; c3com; c2com; c1com];
%             cornerformat(1) = c3com;
%             cornerformat(2) = c4com;
%             cornerformat(3) = c1com;
%             cornerformat(4) = c2com;
            words = 'case c'
%test 4            
        else

            cornerformat = [c2com; c1com; c4com; c3com];
%             cornerformat(1) = c2com;
%             cornerformat(2) = c1com;
%             cornerformat(3) = c3com;
%             cornerformat(4) = c4com;
            words = 'case d'
        end
% test 2        
        
    end
end

function com = centerofmass(im, loc)
    image = imsections(im,loc);
    
    [H,W] = size(image);
    area = bearea(image);
        
    %com = (0,0);
    r = 0;
    c = 0;
    
    for x = 1 : W
        for y = 1 : H
               r = r + y*image(y,x);
               c = c + x*image(y,x);
        end
    end
    
    %com = [compactness r/area c/area];
    com = [round(r/area) round(c/area)];
end

% Bearea. It just works. Bearea, the next best thing since Chuck Norris.
% Bearea, the best a bear can get.
function myarea = bearea(im)
    [H,W] = size(im);
    
    count = 0;
    
    for a = 1 : H
        for b = 1 : W
            count = count + im(a,b);
        end
    end
    
    myarea = count;
end

function compac = compacty(im, loc)
    image = imsections(im,loc);
    
    area = bearea(image);
    perim = bearea(mybwperim(image));
     
    % compactness
    compac = (perim*perim)/(4*pi*area);
end

function dist = myeuclid(a , b)
    
    dist = sqrt((a(1) - b(1))*(a(1) - b(1)) + (a(2) - b(2))*(a(2) - b(2)));
    
end
