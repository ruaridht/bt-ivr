% Trim: takes a binary image and returns the N max areas present in the
% image.
% N = 11
% 
% 
%
function [robot, final, P] = webotify(file,background,robot,fromscratch)
    
    if fromscratch > 0
        
        %Take photos
        input('Take background photo, press any key to continue')
        background = '00000006.jpg';
        
        input('Take maze photo, press any key to continue')
        file = '00000005.jpg';   
        
        input('Take robot photo, press the any key to continue')
        robot = '00000004.jpg';
        
    end
    
    %Fire up the projection, work out what the situation is
    [robot, final, P, blockcase, robotcase, resa, resb, robcom] = initialise(file,background,robot);
    
    %Assign targets
    verttargs = round(resa*0.2*(0.53 + 1.1*blockcase));
    hortargs = resb*(0.5 + robotcase*0.25);
    t1 = [verttargs hortargs];
    t2 = [verttargs (resb - hortargs)];
    
    endtarget = [robcom(1) (robcom(2) - resb*robotcase*0.5)];
    
    final(t1(1),t1(2)) = 1;
    final(t2(1),t2(2)) = 1;
    final(endtarget(1),endtarget(2)) = 1;
    final((robcom(1)-1:robcom(1)+2),(robcom(2)-1:robcom(2)+2)) = 0;
    final(robcom(1), robcom(2)) = 1;
    imshow(final)
    
    
    %Moveout
       
    %Number of pixels between the robot and the target location
    xerxesAtLoc = 100;

    open_robot
    while (xerxesAtLoc > 50)
  
        
        send_command('D,1,1')
        
        read_command       
        image = getImage;  
        
        
        
        send_command('D,0,0')
        read_command
        
        pimage = transfer(image, P);
        pimage = pimage - final;  
        
        robotbwlabel = bwlabel(pimage,8);
        robotlargest = getlargest(robotbwlabel,0);
        robcom = centerofmass(robotlargest,1);                
        final(robcom(1),robcom(2)) = 0;
        
        figure, imshow(final)
        
        xerxesAtLoc = myeuclid(robcom, t1)
        
    end
    
    % Turn Xerxes 90 degrees (in the correct direction)
    send_command('D,1,-1');
    pause(5);
    send_command('D,0,0');
    
    xerxesAtLoc = 100;
    
    while (xerxesAtLoc > 40)
        send_command('D,1,1')
        read_command
        
        pause(1);
        
        send_command('D,0,0')
        read_command
        
        image = getImage;
        pimage = transfer(image, P);
        pimage = pimage - final;
        robotbwlabel = bwlabel(pimage,8);
        robotlargest = getlargest(robotbwlabel,0);
        robcom = centerofmass(robotlargest,1);
        
        final(robcom(1),robcom(2)) = 1;
        
        xerxesAtLoc = myeuclid(robcom, t2)
        
    end
    
    % Turn Xerxes 90 degrees (in the correct direction)
    send_command('D,1,-1');
    pause(5.5);
    send_command('D,0,0');
    
    xerxesAtLoc = 100;
    
    while (xerxesAtLoc > 40)
        send_command('D,1,1')
        read_command
        
        pause(1);
        
        send_command('D,0,0')
        read_command
        
        image = getImage;
        pimage = transfer(image, P);
        pimage = pimage - final;
        robotbwlabel = bwlabel(pimage,8);
        robotlargest = getlargest(robotbwlabel,0);
        robcom = centerofmass(robotlargest,1);
        
        final(robcom(1),robcom(2)) = 1;
        
        xerxesAtLoc = myeuclid(robcom, endtarget)
        
    end
    
    close_robot
    
    imshow(final);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [robot, final, P, blockcase, robotcase, resa, resb, robotcom] = initialise(file,background,robot)
    
    % Get the binary image of the map
    bim = binarypic(file,0.7);
    
    % Subtract the background
    if background ~= 0
        
       tim = binarypic(background,0.7);
       
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
    % If there are fewer objects present arrlen will be less than 10.
    arrlen = min(10,length(areas));
    arraymajig = zeros(arrlen);

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
    compacs = zeros(arrlen);
    
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
    
    cornerformat = findcorners(bwlabeled, cornerlocs);
    
    P = projector(cornerformat);
    final = transfer(bim, P);
     imshow(final)
    
    %possibly fix transform if we've botched it
    [resa resb] = size(final);
    midspace = final((resa*0.3):(resa*0.6), (1:resb));
    
    if bearea(midspace) < 100
            
       %we have a problem
       newcorners = [cornerformat(2,1) cornerformat(2,2); cornerformat(3,1) cornerformat(3,2); cornerformat(1,1) cornerformat(1,2); cornerformat(4,1) cornerformat(4,2)];
       P = projector(newcorners);
       final = transfer(bim,P);
       
       topspace = final((resa*0.2):(resa*0.3), (1:resb));
       botspace = final((resa*0.7):(resa*0.8), (1:resb));
       
       if bearea(topspace) < bearea(botspace)
       
            %newcorners = [cornerformat(2,1) cornerformat(2,2); cornerformat(3,1) cornerformat(3,2); cornerformat(1,1) cornerformat(1,2); cornerformat(4,1) cornerformat(4,2)];
            newcorners = [cornerformat(1,1) cornerformat(1,2); cornerformat(4,1) cornerformat(4,2); cornerformat(2,1) cornerformat(2,2); cornerformat(3,1) cornerformat(3,2)];
            P = projector(newcorners);
            final = transfer(bim,P);
       
       end
       
    end
  
% Finding out where the blocks are

    blocks = binarypic(robot,0.80);
    blocks = transfer(blocks, P);
     

    %closest to robot
    space1 = blocks((resa*0.3):(resa*0.4), (resb*0.45):(resb*0.50));
    areaspace1 = bearea(space1);
    
    %middlespace
    space2 = blocks((resa*0.5):(resa*0.6), (resb*0.45):(resb*0.50));
    areaspace2 = bearea(space2);
    
    %farspace
    space3 = blocks((resa*0.7):(resa*0.8), (resb*0.45):(resb*0.50));
    areaspace3 = bearea(space3);
    
    %Testing which block case we have
    if (areaspace1 < areaspace2) && (areaspace1 < areaspace3)
       
        blockcase = 1;
        final(round(resa*0.42):round(resa*0.87), round(resb*0.4):round(resb*0.6))  =  1;
        
    elseif (areaspace2 < areaspace3)
            
        blockcase = 2;
        final(round(resa*0.22):round(resa*0.43), round(resb*0.4):round(resb*0.6))  =  1;
        final(round(resa*0.65):round(resa*0.87), round(resb*0.4):round(resb*0.6))  =  1;
            
    else

        blockcase = 3;
        final(round(resa*0.22):round(resa*0.65), round(resb*0.4):round(resb*0.6))  =  1;
            
    end
    
    rspace1 = blocks((resa*0.2):(resa*0.3), (resb*0.2):(resa*0.4));
    %imshow(space3);
%    imshow(blocks);

    robot = binarypic(robot,0.65);
    robot = transfer(robot, P);
    robot = robot - final;
    
    robot = cleanup(robot,1,3,0);
    
    bwrobot = im2bw(robot);
    
    robotbwlabel = bwlabel(bwrobot,8);
    robotlargest = getlargest(robotbwlabel,0);
    
    robotcom = centerofmass(robotlargest,1);
    
    % How to get which side the robot is on
    if (robotcom(2) > 240)
        robotcase = 1;
    else
        robotcase = -1;
    end
    
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



function P = projector(coms) 
    
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

    outimage=zeros(460, 360);   % destination image
    %v=zeros(3,1); % We don't need to assign v here, but meh

    % loop over all pixels in the destination image, finding
    % corresponding pixel in source image
    for r = 1 : 460
        for c = 1 : 360
            v=P*[r,c,1]';        % project destination pixel into source
            y=round(v(1)/v(3));  % undo projective scaling and round to nearest integer
            x=round(v(2)/v(3));
            if (x >= 1) && (x <= IC) && (y >= 1) && (y <= IR)
                outimage(r,c)=inimage(y,x);   % transfer colour
            end
        end
    end

    finalout = outimage/255;

end

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
end

function cornerformat = findcorners(bwlabeled, cornerlocs)

    % Get the Centre Of Mass of the objects at each cornerloc
    c1com = centerofmass(bwlabeled,cornerlocs(1));
    c2com = centerofmass(bwlabeled,cornerlocs(2));
    c3com = centerofmass(bwlabeled,cornerlocs(3));
    c4com = centerofmass(bwlabeled,cornerlocs(4));
    c5com = centerofmass(bwlabeled,cornerlocs(5));

    atob = myeuclid(c1com, c2com);
    atoc = myeuclid(c1com, c3com);
    atoe = myeuclid(c1com, c5com);
    
    % cornerformat in format [bl br tl tr] (portrait, robot at bottom)
    % Note: the test cases here have not used background subtraction, which
    % will need to be omitted.
    
    if atob > atoc        
        if atoe > myeuclid(c2com,c5com)
            cornerformat = [c2com; c4com; c1com; c3com];
            %words = 'case a'            
% tests 1,3
        else
            cornerformat = [c3com; c4com; c2com; c1com];
            %words = 'case b'
        end
%test 5        
    else        
        if atoe > myeuclid(c3com,c5com)         
            cornerformat = [c4com; c3com; c2com; c1com];
            %words = 'case c'
%test 4            
        else
            cornerformat = [c2com; c1com; c4com; c3com];
            %words = 'case d'
        end
% test 2
    end % End of large if

end

function imsect = imsections(bwlabeled, secLoc)
    
    [u,v] = find(bwlabeled==secLoc);
    
    m = length(u);
    n = length(v);
    
    imsect = zeros(640,480);
    
    for x = 1:m
        imsect(u(x),v(x)) = 1;
    end
     
end

function image = getImage
    unix(['mplayer tv:// -tv driver=v4l:width=640:height=480:device=/dev/video0 -frames 4 -vo jpeg']);
    %unix(['mv 00000005.jpg ', filename, '.jpg']);
    %Im = importdata([filename, '.jpg'],'jpg');
    %image = importdata('00000005.jpg','jpg');
    image = binarypic('00000004.jpg',0.7);
end