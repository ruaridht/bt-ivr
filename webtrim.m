% Trim: takes a binary image and returns the N max areas present in the
% image.
% N = 11
% 
% 
%
function [robot, final, P] = webtrim
    
        
     %Take photos
    %input('Take maze photo, press any key to continue')
    %take_snap;
    %file = 'maze.ppm';
    pause(1);
    bim = imread('maze.ppm');
    pause(0.5);
    bim = im2bw(bim,0.9);
    bim = 1-bim;
    figure, imshow(bim);
    %bim = binarypic(file);
        
    input('Take robot photo, press the any key to continue')
    take_snap;
    pause(1);
    robot = imread('snap.ppm');
    pause(0.5);
    robot = rgb2gray(robot);
    robot = im2bw(robot,0.9);
    robot = 1-robot;
    figure, imshow(robot);
    
    %Fire up the projection, work out what the situation is
    [robot, final, P, blockcase, robotcase, resa, resb, robcom] = initialise(bim,robot);
        
    pause(0.5);
    
    %Assign targets
    verttargs = round(resa*0.2*(0.5 + 1.1*blockcase));
    hortargs = resb*(0.5 + robotcase*0.25);
    t1 = [verttargs hortargs]
    t2 = [verttargs (hortargs - resb*robotcase*0.5)]
    
    endtarget = [robcom(1) (robcom(2) - resb*robotcase*0.5)]
    
    final((t1(1)-1:t1(1)+2),(t1(2)-1:t1(2)+2)) = 0;
    final(t1(1), t1(2)) = 1;
    final(t2(1),t2(2)) = 0;
%     final((endtarget(1)-1:endtarget(1)+2),(endtarget(2)-1:endtarget(2)+2)) = 0;
%     final(endtarget(1),endtarget(2)) = 1;
    final((robcom(1)-1:robcom(1)+2),(robcom(2)-1:robcom(2)+2)) = 0;
    final(robcom(1), robcom(2)) = 1;
    
    figure, imshow(final)

    prevcom = robcom;
    
    % The turnrate is roughly how long it takes to turn 1 degree 
    % (at speed 1,-1)
    turnrate = 10/360;
    pauserate = 0.35;
    
    %Moveout
       
    %Number of pixels between the robot and the target location
    xerxesAtLoc = 100;

    while (xerxesAtLoc > 10)
    
        v1 = robcom - prevcom;
        v2 = t1 - robcom;
        
        angle = vangle(v1,v2);
        dist = (robcom(2) - t1(2));
        
        if (dist > 2)
            send_command('D,20,-20');
        elseif (dist < -2)
            send_command('D,-20,20');
        end
        pause(angle*turnrate);
        send_command('D,0,0');
        pause(1);
        if (xerxesAtLoc > 40)
            moveforward('D,50,50', 2);     
        else
            moveforward('D,20,20', 2);
        end
        
        prevcom = robcom;
        
        image = getImage;
        pause(1);
    
        pimage = transfer(image, P);
        pimage = pimage - final;  
        pimage = cleanup(pimage,1,1,0);    
        
        pause(1);
    
%         figure(300), imshow(pimage);
%         robotbwlabel = bwlabel(pimage,8);
%         pause(3);
%         robotlargest = getlargest(robotbwlabel,0);
%         figure(200), imshow(robotlargest);
%         pause(3);
        robcom = centerofmass(pimage,1);
       
        pimage((robcom(1)-1:robcom(1)+2),(robcom(2)-1:robcom(2)+2)) = 0;
        pimage(robcom(1), robcom(2)) = 1;
        pimage((t1(1)-1:t1(1)+2),(t1(2)-1:t1(2)+2)) = 0;
        pimage(t1(1),t1(2)) = 1;
        imshow(pimage);
        final(robcom(1),robcom(2)) = 0;
   
        xerxesAtLoc = myeuclid(robcom, t1)
        
    end
    
    figure, imshow(final)
    
    % Turn Xerxes 90 degrees (in the correct direction)
    %turn90(robotcase);
%     send_command('D,1,-1');
%     pause(120*turnrate);
%     send_command('D,0,0');
    
    %prevcom = robcom;
    
    xerxesAtLoc = myeuclid(robcom, t2);
    
    while (xerxesAtLoc > 15)
        
        v1 = robcom - prevcom;
        v2 = t2 - robcom;
        
        angle = vangle(v1,v2);
        
        dist = robotcase*(robcom(1) - t2(1));
        
        if (dist > 2)
            send_command('D,20,-20');
        elseif (dist < -2)
            send_command('D,-20,20');
        end
        pause(angle*turnrate);
        send_command('D,0,0');
        pause(1);
        if (xerxesAtLoc > 40)
            moveforward('D,40,40', 2);     
        else
            moveforward('D,20,20', 2);
        end
        
        prevcom = robcom;
        
        image = getImage;
        pause(1);
    
        pimage = transfer(image, P);
        pimage = pimage - final;  
        pimage = cleanup(pimage,1,1,0);    
        
        pause(1);
    
%         figure(300), imshow(pimage);
%         robotbwlabel = bwlabel(pimage,8);
%         pause(3);
%         robotlargest = getlargest(robotbwlabel,0);
%         figure(200), imshow(robotlargest);
%         pause(3);
        robcom = centerofmass(pimage,1);
       
        pimage((robcom(1)-1:robcom(1)+2),(robcom(2)-1:robcom(2)+2)) = 0;
        pimage(robcom(1), robcom(2)) = 1;
        pimage((t2(1)-1:t2(1)+2),(t2(2)-1:t2(2)+2)) = 0;
        pimage(t2(1),t2(2)) = 1;
        imshow(pimage);
        final(robcom(1),robcom(2)) = 0;
   
        xerxesAtLoc = myeuclid(robcom, t2)
                
%         if (dist > 5)
%             
%             send_command('D,20,-20');
%             
%         elseif (dist < -5)
%             send_command('D,-20,20');
%         end
%         
%         pause(angle*turnrate);
%         % Slowdown
%         if (xerxesAtLoc > 50)
%             moveforward('D,50,50', 2);     
%         else
%             moveforward('D,20,20', 2);
%         end
%         prevcom = robcom;
%        
%         image = getImage(thresh);
%         pimage = transfer(image, P);
%         pimage = pimage - final;
%         pimage = cleanup(pimage,1,3,0);     
%         
%         %figure(300),imshow(pimage);
%         robotbwlabel = bwlabel(pimage,8);
%         robotlargest = getlargest(robotbwlabel,0);
%         robcom = centerofmass(robotlargest,1)
%         
%         pimage(robcom(1),robcom(2)) = 0;
%         pimage(t2(1),t2(2)) = 1;
%         imshow(pimage);
%         final(robcom(1),robcom(2)) = 0;
%    
%         xerxesAtLoc = myeuclid(robcom, t2)
        
    end
    
    % Turn Xerxes 90 degrees (in the correct direction)
    %turn90(robotcase);
    
    xerxesAtLoc = myeuclid(robcom,endtarget);
    %prevcom = robcom;
    
    while (xerxesAtLoc > 2)
        
        v1 = robcom - prevcom;
        v2 = endtarget - robcom;
        
        angle = vangle(v1,v2);
        dist = (robcom(2) - endtarget(2));
                
        if (dist < -2)
            send_command('D,20,-20');
        elseif (dist > 2)
            send_command('D,-20,20');
        end
        
        pause(angle*turnrate);
        send_command('D,0,0');
        pause(1);
        
        if (xerxesAtLoc > 40)
            moveforward('D,40,40', 2);     
        elseif (xerxesAtLoc > 20)
            moveforward('D,10,10', 2);
        elseif (xerxesAtLoc > 5)
            moveforward('D,2,2,',2);
        end
        
        prevcom = robcom;
        
        image = getImage;
        pause(1);
    
        pimage = transfer(image, P);
        pimage = pimage - final;  
        pimage = cleanup(pimage,1,1,0);    
        
        pause(1);
    
%         figure(300), imshow(pimage);
%         robotbwlabel = bwlabel(pimage,8);
%         pause(3);
%         robotlargest = getlargest(robotbwlabel,0);
%         figure(200), imshow(robotlargest);
%         pause(3);
        robcom = centerofmass(pimage,1);
       
        pimage((robcom(1)-1:robcom(1)+2),(robcom(2)-1:robcom(2)+2)) = 0;
        pimage(robcom(1), robcom(2)) = 1;
        pimage((endtarget(1)-1:t1(1)+2),(endtarget(2)-1:endtarget(2)+2)) = 0;
        pimage(endtarget(1),endtarget(2)) = 1;
        imshow(pimage);
        final(robcom(1),robcom(2)) = 0;
   
        xerxesAtLoc = myeuclid(robcom, endtarget)
        
%         pause(angle*turnrate);
%         % Slowdown.  When xerxes is closer to the final target we want to
%         % go slower (to be more precise with out movement). 
%         % slower = more accurate
%         if (xerxesAtLoc > 50)
%             moveforward('D,3,3', 1);     
%         else
%             moveforward('D,1,1', 1);
%         end    
%         prevcom = robcom;
%        
%         image = getImage(thresh);
%         pimage = transfer(image, P);
%         pimage = pimage - final;
%         pimage = cleanup(pimage,1,3,0);     
%         
%         %imshow(pimage);
%         robotbwlabel = bwlabel(pimage,8);
%         robotlargest = getlargest(robotbwlabel,0);
%         robcom = centerofmass(robotlargest,1)
%         
%         pimage(robcom(1),robcom(2)) = 0;
%         pimage(endtarget(1),endtarget(2)) = 1;
%         imshow(pimage);
%         final(robcom(1),robcom(2)) = 0;
%    
%         xerxesAtLoc = myeuclid(robcom, endtarget)
        
    end
        
    imshow(final);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [robot, final, P, blockcase, robotcase, resa, resb, robotcom] = initialise(bim,robot)
    
    % Get the binary image of the map
    %bim = cleanup(bim,1,1,0);
    bim = im2bw(bim);
    bim = imresize(bim,2);
    
    robot = imresize(robot,2);
    
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
    lowcompacsloc = zeros(5);
    % compacs: stores the compactness of each image section in arraymajig
    compacs = zeros(arrlen);
    
    % Populate compacs.
    for c = 1 : n
        compacs(c) = compacty(bwlabeled, arraymajig(c));
    end
    
    % Find the 6 most compact objects and return the locations (locations
    % in bwlabeled).
    for a = 1 : 5
        maxed = 1;
        for b = 2 : n
            if (compacs(b) < compacs(maxed))
                maxed = b;
            end
        end
        lowcompacsloc(a) = arraymajig(maxed);
        %figure, imshow(imsections(bwlabeled,arraymajig(maxed)));
        compacs(maxed) = 100000;
    end
    
    % cornerlocs: stores the locations of the 4 corners of the map, and the
    % closest robotic terminus.
    cornerlocs = zeros(5);
    % lowcompacsarea: the areas of the locations (from lowcompacsloc) in
    % bwlabeled
    lowcompacsarea = zeros(5);
    
    % Populate lowcompacsloc
    for a = 1 : 5
        lowcompacsarea(a) = bearea(imsections(bwlabeled,lowcompacsloc(a)));
    end
    
    % Populate cornerlocs
    for a = 1 : 5
        maxindex = 1;
        for b = 2 : 5
            if (lowcompacsarea(b) > lowcompacsarea(maxindex))
                maxindex = b;
            end
        end
        cornerlocs(a) = lowcompacsloc(maxindex);
        lowcompacsarea(maxindex) = 0;
    end
    
    cornerformat = findcorners(bwlabeled, cornerlocs);
    
    %UV = [[30,50]',[0,310]',[400,80]',[420,310]']';
    %UV = [[15,25];[15,155];[210,25];[210,155]];
    
    %tform = cp2tform(cornerformat,UV,'projective');
    %tform = maketform('projective',cornerformat,UV);
    %final = imtransform(bim,tform);
    
    P = projector(cornerformat);
    final = transfer(bim, P);
    figure(4)
    imshow(final);
    
    %possibly fix transform if we've botched it
    [resa resb] = size(final);
    midspace = final((resa*0.3):(resa*0.6), (1:resb));
    
    if bearea(midspace) < 1
            
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

    blocks = transfer(robot, P);
     
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
        %final(round(resa*0.42):round(resa*0.87), round(resb*0.4):round(resb*0.6))  =  1;
        final(round(resa*0.42):round(resa*0.87), round(resb*0.45):round(resb*0.55))  =  1;
        
    elseif (areaspace2 < areaspace3)
            
        blockcase = 2;
        final(round(resa*0.22):round(resa*0.43), round(resb*0.45):round(resb*0.55))  =  1;
        final(round(resa*0.65):round(resa*0.87), round(resb*0.45):round(resb*0.55))  =  1;
            
    else

        blockcase = 3;
        final(round(resa*0.22):round(resa*0.65), round(resb*0.45):round(resb*0.55))  =  1;
            
    end

    robot = transfer(robot, P);
    robot = robot - final;
    
    robot = cleanup(robot,1,3,0);
    
    bwrobot = im2bw(robot);
    
    robotbwlabel = bwlabel(bwrobot,8);
    robotlargest = getlargest(robotbwlabel,0);
    

    robotcom = centerofmass(robotlargest,1);
    
    % How to get which side the robot is on
    if (robotcom(2) > 120)
        robotcase = 1
    else
        robotcase = -1
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
    
    %UV=[[30,50]',[30,310]',[420,50]',[420,310]']';    % target points
    UV = [[30,50];[30,310];[420,50];[420,310]];
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
    
    figure,imshow(outimage);

    %finalout = outimage/255;
    finalout = outimage;

end

function newimage = binarypic(name)

    binary_pic = imread(name);
    [m,n] = size(binary_pic);
    threshold = 240;
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
    figure(2)
    imshow(newimage);
end

function newimage = robbinarypic(name)

    binary_pic = imread(name);
    [m,n] = size(binary_pic);
    threshold = 240;

    newimage = binary_pic;
    G = newimage(:,:,1);
    for r = 1 : m
       for c = 1 : n
           if (binary_pic(r,c) > threshold)
                newimage(r,c) = 255;
           else
                newimage(r,c) = 0;
           end
       end
    end
    [m,n] = size(G);
    for a = 1 : m
        for b = 1 : n
            if (G(a,b) < threshold)
                newimage(a,b) = 255;
            end
        end
    end
    figure(3)
    imshow(newimage);
end

function cornerformat = findcorners(bwlabeled, cornerlocs)

    % Get the Centre Of Mass of the objects at each cornerloc
    c1com = centerofmass(bwlabeled,cornerlocs(1));
    c2com = centerofmass(bwlabeled,cornerlocs(2));
    c3com = centerofmass(bwlabeled,cornerlocs(3));
    c4com = centerofmass(bwlabeled,cornerlocs(4));
    c5com = centerofmass(bwlabeled,cornerlocs(5));

    %atob = myeuclid(c1com, c2com);
    %atoc = myeuclid(c1com, c3com);
    atoe = myeuclid(c1com, c5com);
   
    % Rearrange the points based on the distance to the small point
    btoe = myeuclid(c2com,c5com);
    ctoe = myeuclid(c3com,c5com);
    dtoe = myeuclid(c4com,c5com);

    a = [atoe c1com];
    b = [btoe c2com];
    c = [ctoe c3com];
    d = [dtoe c4com];
   
    bob = sortrows([a; b; c; d]);
    
    c1com = [bob(5) bob(9)];
    c2com = [bob(6) bob(10)];
    c3com = [bob(7) bob(11)];
    c4com = [bob(8) bob(12)];
    
    cornerformat = [c1com; c2com; c3com; c4com];
    %cornerformat = [c2com; c1com; c4com; c3com];
    
end

function imsect = imsections(bwlabeled, secLoc)
    
    [u,v] = find(bwlabeled==secLoc);
    
    m = length(u);
    n = length(v);
    
    imsect = zeros(320,240);
    
    for x = 1:m
        imsect(u(x),v(x)) = 1;
    end
     
end

function image = getImage
    
    pause(1);
    take_snap;
    pause(1);
    pic = imread('snap.ppm');
    pause(0.5);
    pic = rgb2gray(pic);
    pic = im2bw(pic,0.9);
    pic = 1-pic;
    image = imresize(pic,2);

%     take_snap;
%     pause(0.5);
%     pic = imread('snap.ppm');
%     pic = im2gray(pic);
%     pic = im2bw(pic,0.9);
%     image = 1 - pic;
%     image = imresize(image,2);
%     %image = binarypic(pic);

end

function turn90(direction)
    %direction: 1 for right, -1 for left.
    %string = ['D,' speed ',' -speed];
        pause(0.5);
    if direction == 1
        send_command('D,40,-40');
    else
        send_command('D,-40,40');
    end
    read_command;
        pause(0.5);
    pause(1.3);
    send_command('D,0,0');
        pause(0.5);
    read_command;
    
        
end

function degs = vangle(v1,v2)
    v1mag = sqrt(v1(1)*v1(1) + v1(2)*v1(2));
    v2mag = sqrt(v2(1)*v2(1) + v2(2)*v2(2));
    v1dotv2 = (v1(1)*v2(1) + v1(2)*v2(2));
    rad = acos(v1dotv2/(v1mag*v2mag));
    degs = round(180*rad/pi);
end

function moveforward(speed, time)
    send_command(speed);
    pause(time);
    send_command('D,0,0');
end
