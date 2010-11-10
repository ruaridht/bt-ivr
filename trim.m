% Trim: takes a binary image and returns the N max areas present in the
% image.
% N = 11
% 
% 
%
function [robot, final, P] = trim(file,background,robot)
    
    % Get the binary image of the map
    bim = binarypic(file);
    
    % Subtract the background
    if background ~= 0
        
       tim = binarypic(background);
       
       bim = bim - tim;
       
    end

    % Show the background ... ?
    %imshow(bim);
    
    % Label the disconnected regions of bim
    bwlabeled = bwlabel(bim,4);
    % Get the areas associated to the labelled regions
    regiondata = regionprops(bwlabeled, 'Area');
    allareas = [regiondata.Area];
    
    areas = [];
    
    m = length(allareas);
   
    indexes = [];
    
    for i = 1 : m
        
        if allareas(i) > 20
           
           indexes = [indexes i];
           areas = [areas allareas(i)];
            
        end
               
    end
    
    % Create an array to store the indexes of the 10 largest areas in the
    % image (i.e. in 'areas')
    arraymajig = zeros(10);

    n = length(arraymajig);
    m = length(areas);
    
    % Populate arraymajig
    for a = 1 : n
        maxed = 1;
        for b = 2 : m
           if (areas(b) > areas(maxed))
               maxed = b;
           end
           
        end
        arraymajig(a) = indexes(maxed);
        areas(maxed) = 0;
    end
    
    % lowcompacsloc: stores the locations (in bwlabeled) of the 6 most
    % compact objects in the top ten largest (in arraymajig)
    lowcompacsloc = zeros(6);
    % compacs: stores the compactness of each image section in arraymajig
    compacs = zeros(10);
    
    % Populate compacs.
    for c = 1 : n
        compacs(c) = compacty(bwlabeled, arraymajig(c));
    end
    
    % Find the 6 most compact objects and return the locations (locations
    % in bwlabeled).
    for a = 1 : 6
        maxed = 1;
        for b = 2 : n
            if (compacs(b) < compacs(maxed))
                maxed = b;
            end
        end
        lowcompacsloc(a) = arraymajig(maxed);
        compacs(maxed) = 100000;
    end
    
    % cornerlocs: stores the locations of the 4 corners of the map, and the
    % closest robotic terminus.
    cornerlocs = zeros(5);
    % lowcompacsarea: the areas of the locations (from lowcompacsloc) in
    % bwlabeled
    lowcompacsarea = zeros(6);
    
    % Populate lowcompacsloc
    for a = 1 : 6
        lowcompacsarea(a) = bearea(imsections(bwlabeled,lowcompacsloc(a)));
    end
    
    % Populate cornerlocs
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
    
    % Get the Centre Of Mass of the objects at each cornerloc
    c1com = centerofmass(bwlabeled,cornerlocs(1));
    c2com = centerofmass(bwlabeled,cornerlocs(2));
    c3com = centerofmass(bwlabeled,cornerlocs(3));
    c4com = centerofmass(bwlabeled,cornerlocs(4));
    c5com = centerofmass(bwlabeled,cornerlocs(5));
    
    imagey = bim;
    imagey(c1com(1),c1com(2)) = 0;
    imagey(c2com(1),c2com(2)) = 0;
    imagey(c3com(1),c3com(2)) = 0;
    imagey(c4com(1),c4com(2)) = 0;
    imagey(c5com(1),c5com(2)) = 0;
    imshow(imagey);

    atob = myeuclid(c1com, c2com);
    atoc = myeuclid(c1com, c3com);
    atoe = myeuclid(c1com, c5com);
    btoe = myeuclid(c2com, c5com);
    ctoe = myeuclid(c3com, c5com);
    dtoe = myeuclid(c4com, c5com);
    
%     midpoint = (atoe +btoe + ctoe + dtoe)/4;
%     
%     bottom = [];
%     top = [];
%     
%     if atoe < midpoint    
%         bottom = [bottom; c1com];        
%     else        
%         top = [top; c1com];        
%     end
%     
%     if btoe < midpoint    
%         bottom = [bottom; c2com];        
%     else        
%         top = [top; c2com];        
%     end
%     
%     if ctoe < midpoint    
%         bottom = [bottom; c3com];        
%     else        
%         top = [top; c3com];        
%     end
%     
%     if dtoe < midpoint    
%         bottom = [bottom; c4com];        
%     else        
%         top = [top; c4com];        
%     end
%       
%     if (bottom(1,1) - bottom(2,1))*(bottom(1,1) - bottom(2,1)) < (bottom(1,2) - bottom(2,2))*(bottom(1,2) - bottom(2,2))
%        %portrait
%        
%        write = 'portrait';
%        
%        if bottom(1,1) < bottom(2,1)          
%            cornerformat = [c1com; c2com];
%            write = [write ' a'];
%        else           
%            cornerformat = [c2com; c1com];
%            write = [write ' b'];
%        end
%        
%        if top(1,1) < top(2,1)          
%            cornerformat = [cornerformat; c3com; c4com];
%            write = [write '1'];
%        else           
%            cornerformat = [cornerformat; c4com; c3com]; 
%            write = [write '2'];
%        end 
%        
%     else
%         %landscape
%         
%         write = 'landscape'
%         
%         if bottom(1,2) < bottom(2,2)          
%             cornerformat = [c1com; c2com];
%             write = [write ' a'];
%         else           
%             cornerformat = [c2com; c1com];
%             write = [write ' b'];
%         end
%        
%         if top(1,2) < top(2,2)          
%             cornerformat = [cornerformat; c3com; c4com];
%             write = [write '1'];  
%         else           
%             cornerformat = [cornerformat; c4com; c3com];
%             write = [write '2'];
%         end 
%         
%     end
%     
%     write = write
%     
%     cornerformat = cornerformat
    
    % cornerformat in format [bl br tl tr] (portrait, robot at bottom)
    % Note: the test cases here have not used background subtraction, which
    % will need to be omitted.
    
    if atob > atoc
        
        if atoe > myeuclid(c2com,c5com)
            
            cornerformat = [c2com; c4com; c1com; c3com]

            words = 'case a'
% tests 1,3
        else

            cornerformat = [c3com; c4com; c2com; c1com]

            words = 'case b'
        end
%test 5        
    else     
        
        if atoe > myeuclid(c3com,c5com)
         
            cornerformat = [c4com; c3com; c2com; c1com]

            words = 'case c'
%test 4            
        else

            cornerformat = [c2com; c1com; c4com; c3com]

            words = 'case d'
        end
% test 2        

    end % End of large if
    
    P = projecter(cornerformat);
    final = transfer(bim, P);
    %imshow(final);
%     robot = binarypic(robot);
%     robot = transfer(robot, P);
%     robot = robot - final;
    
end

function com = centerofmass(bwim, loc)
    image = imsections(bwim,loc);
    
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

function compac = compacty(bwim, loc)
    image = imsections(bwim,loc);
    
    area = bearea(image);
    perim = bearea(mybwperim(image));
     
    % compactness
    compac = (perim*perim)/(4*pi*area);
end

function dist = myeuclid(a , b)
    
    dist = sqrt((a(1) - b(1))*(a(1) - b(1)) + (a(2) - b(2))*(a(2) - b(2)));
    
end

function P = projecter(coms) 
    
    %UV=zeros(4,2);
    %XY=zeros(4,2);
    UV=[[30 , 50 ]',[30,310]',[420,50]',[420,310]']';    % target points
    XY=coms;    % source points

    P=esthomog(UV,XY,4);    % estimate homography mapping UV to XY
    
end

function finalout = transfer(img, P)
    % get input image and sizes
    
    inimage=img;
    [IR,IC]=size(inimage);

    outimage=zeros(460, 360,3);   % destination image
    %v=zeros(3,1); % We don't need to assign v here, but meh

    % loop over all pixels in the destination image, finding
    % corresponding pixel in source image
    for r = 1 : 460
        for c = 1 : 360
            v=P*[r,c,1]';        % project destination pixel into source
            y=round(v(1)/v(3));  % undo projective scaling and round to nearest integer
            x=round(v(2)/v(3));
            if (x >= 1) && (x <= IC) && (y >= 1) && (y <= IR)
                outimage(r,c,:)=inimage(y,x,:);   % transfer colour
            end
        end
    end

    finalout = (outimage/255);

end

