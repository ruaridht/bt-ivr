% Trim: Performs robot manipulation.
% bim: the image of the maze, in binary form.
% robot: the image of the robot and blocks on the maze, in binary form.
% fromscratch: indicates whether we need to take new maze and
% robot photos
% NOTE: This file is mostly a direct copy of trim.m, adapted to work with
% Webots.
% NOTE: Outputs used for recording results.
% NOTE: Pauses required in code for communication between Matlab and
% Webots.
% NOTE: Xerxes is the name chosen for our robot.
% 
% William Bradshaw   Ruaridh Thomson
%     s0806628          s0786036
%
%       <(oO)< ^(OO)^ >(Oo)> 
% Sponsored by the Kirby Death Squad

function [result, robot, final, P] = webtrim

    % Take photos
    input('Take maze photo, press any key to continue')
    take_snap;
    pause(1);
    bim = imread('snap.ppm');
    pause(0.5);
    bim = im2bw(bim,0.9);
    bim = 1-bim;
    figure, imshow(bim);
        
    input('Take robot photo, press the any key to continue')
    take_snap;
    pause(1);
    robot = imread('snap.ppm');
    pause(0.5);
    robot = rgb2gray(robot);
    robot = im2bw(robot,0.9);
    robot = 1-robot;
    figure, imshow(robot);
    
    % Fire up the projection, work out what the situation is.
    % Initialise is called to get all information needed before movement of
    % the robot.
    [robot, final, P, blockcase, robotcase, resa, resb, robcom, result] = initialise(bim,robot);
    
    pause(0.5);
    
    % Assign targets.  verttargs is used in both cases, since the two
    % vertical (y) values should always be the same.
    % hortargs is the horizontal (x) value of the target location.
    % t1 and t2 are the two target locations, and endtarget is the final
    % position for the robot
    verttargs = round(resa*0.2*(0.5 + 1.1*blockcase));
    hortargs = resb*(0.5 + robotcase*0.25);
    t1 = [verttargs hortargs];
    t2 = [verttargs (hortargs - resb*robotcase*0.5)];
    
    % Since the maze is a mirror image of itself (down the height of it),
    % the final position is found by mirroring the robot's start position.
    endtarget = [robcom(1) (robcom(2) - resb*robotcase*0.5)];
    
    % Add colour to the target locations so we can distinguish between
    % them. (Including the robot's start location.)
    result((t1(1)-3):(t1(1)+3),(t1(2)-3:t1(2)+3),1) = 1;    
    result((t1(1)-3):(t1(1)+3),(t1(2)-3:t1(2)+3),2) = 1;
    result((t1(1)-2):(t1(1)+2),(t1(2)-2:t1(2)+2),3) = 1;
    
    result((t2(1)-2):(t2(1)+2),(t2(2)-2:t2(2)+2),1) = 1; 
    result((t2(1)-3):(t2(1)+3),(t2(2)-3:t2(2)+3),2) = 1;
    result((t2(1)-2):(t2(1)+2),(t2(2)-2:t2(2)+2),3) = 1;
    
    result((endtarget(1)-2):(endtarget(1)+2),(endtarget(2)-2:endtarget(2)+2),2) = 1; 
    result((endtarget(1)-3):(endtarget(1)+3),(endtarget(2)-3:endtarget(2)+3),1) = 1;
    result((endtarget(1)-2):(endtarget(1)+2),(endtarget(2)-2:endtarget(2)+2),3) = 1;

    result((robcom(1)-2):(robcom(1)+2),(robcom(2)-2:robcom(2)+2),1) = 1; 
    result((robcom(1)-3):(robcom(1)+3),(robcom(2)-3:robcom(2)+3),3) = 1;
    result((robcom(1)-2):(robcom(1)+2),(robcom(2)-2:robcom(2)+2),2) = 1;
    
    figure(5),imshow(result);

    % During operation we use vectors to calculate the angle the robot
    % needs to turn.  Initially we want the robot's location to be (0,0),
    % so we set the initial previous location to the robot's current
    % (start) location.
    % prevcom: The previous center of mass of the robot.
    prevcom = robcom;
    
    % The turnrate is roughly how long it takes to turn 1 degree 
    % (at speed 1,-1)
    turnrate = 10/360;
    
    % xerxesAtLoc: the number of pixels between the robot and the target location.
    % Initially this is just set to a suitable value so that we enter the
    % loop.  It is overwritten inside the loop.
    xerxesAtLoc = 100;

    % The robot's movement is divided into three sections corresponding to
    % the 3 sections of the maze we need to navigate through.  Initially we
    % want to move towards the gap in the centre of the maze, then move
    % through the gap, and finally move onto the overall target location
    % (endtarget).
    % This first loop manages moving towards the gap by moving towards the
    % first target, t1.  Once the robot is less than 10 pixels from this
    % location we consider it to have reached the location and move onto
    % the next phase.
    while (xerxesAtLoc > 10)
    
        % v1: the vector of the robot's movement.
        % v2: the vector from the robot to the target.
        % angle: the angle between v1 and v2. This is the angle we want the
        % robot to turn (i.e. turn so that it is now moving along v2
        % towards the target).
        % dist: the difference between the x values of the robot and
        % target.
        v1 = robcom - prevcom;
        v2 = t1 - robcom;
        
        angle = vangle(v1,v2);
        dist = (robcom(2) - t1(2));
        
        % Although not strictly necessary, we typically don't need the
        % robot to turn if its direction is close enough.
        if (dist > 2)
            send_command('D,20,-20');
        elseif (dist < -2)
            send_command('D,-20,20');
        end
        pause(angle*turnrate);
        
        % Due to the way Matlab communicates with Webots, we need to
        % explicitly tell the robot to stop moving after the turn.
        send_command('D,0,0');
        pause(1);
        
        % Until we get close enough to the target location we want to move
        % a bit quicker.  Once close enough we slow down.
        if (xerxesAtLoc > 40)
            moveforward('D,50,50', 2);     
        else
            moveforward('D,20,20', 2);
        end
        
        % Once moved we set the previous CoM to the old CoM before we get
        % the new one.
        prevcom = robcom;
        
        % image: the photo of the world's current situation
        % pimage: the image after projection, background removal, and
        % cleanup.
        % robcom: the new center of mass of the robot
        % As a result from the way we use final in initialise we can be
        % confident that the only object in the image after subtracting
        % final from pimage is the robot.  This assumes minimal camera
        % movement (though in webots the camera does not move).
        image = getImage;
        pause(1);
        pimage = transfer(image, P);
        pimage = imsubtract(pimage,final);
        pimage = cleanup(pimage,1,3,0);    
        pause(1);
        robcom = centerofmass(pimage,1);
       
        % Code used during testing.  Used to determine the accuracy of the
        % algorithm.
%         pimage((robcom(1)-1:robcom(1)+2),(robcom(2)-1:robcom(2)+2)) = 0;
%         pimage(robcom(1), robcom(2)) = 1;
%         pimage((t1(1)-1:t1(1)+2),(t1(2)-1:t1(2)+2)) = 0;
%         pimage(t1(1),t1(2)) = 1;
        
        % robresult: a copy of result with the robot super-imposed on top
        % to track the robot's movement.
        robresult = result;
        robresult(:,:,2) = pimage;
        
        figure(1), imshow(image);
        figure(2), imshow(pimage);
        figure(3), imshow(robresult);
        
        % Draw the CoM of the robot onto result to record the path of the
        % robot.
        result((robcom(1)-1):(robcom(1)+1),(robcom(2)-1:robcom(2)+1),1) = 0; 
        result((robcom(1)-2):(robcom(1)+2),(robcom(2)-2:robcom(2)+2),1) = 30;
        
        % Get the new distance of the robot from the target location.
        xerxesAtLoc = myeuclid(robcom, t1);
        
    end
    
    % Set the distance to the new target.
    xerxesAtLoc = myeuclid(robcom, t2);
    
    % Enter the second phase of the robot's movement - moving through the
    % middle section of the maze.
    while (xerxesAtLoc > 15)
        
        v1 = robcom - prevcom;
        v2 = t2 - robcom;
        angle = vangle(v1,v2);
        
        % Instead of using the x value (as in the first phase) we want to
        % use the y value since we are moving horizontally across the map.
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
        pimage = imsubtract(pimage,final);
        pimage = cleanup(pimage,1,3,0);    
        pause(1);
        robcom = centerofmass(pimage,1);
       
%         pimage((robcom(1)-1:robcom(1)+2),(robcom(2)-1:robcom(2)+2)) = 0;
%         pimage(robcom(1), robcom(2)) = 1;
%         pimage((t2(1)-1:t2(1)+2),(t2(2)-1:t2(2)+2)) = 0;
%         pimage(t2(1),t2(2)) = 1;
        
        robresult = result;
        robresult(:,:,2) = pimage;
        
        figure(1), imshow(image);
        figure(2), imshow(pimage);
        figure(3), imshow(robresult);
        
        % Draw the CoM of the robot onto result to record the path of the
        % robot.
        result((robcom(1)-1):(robcom(1)+1),(robcom(2)-1:robcom(2)+1),1) = 0; 
        result((robcom(1)-2):(robcom(1)+2),(robcom(2)-2:robcom(2)+2),1) = 30;
        
        xerxesAtLoc = myeuclid(robcom, t2);
        
    end

    xerxesAtLoc = myeuclid(robcom,endtarget);
    
    % The third (and final) phase of the robot's movement.  We want the
    % robot much closer to the target (endtarget), so we lower the boundary
    % of xerxesAtLoc.
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
        else
            moveforward('D,2,2,',2);
        end
        
        prevcom = robcom;
        
        image = getImage;
        pause(1);
        pimage = transfer(image, P);
        pimage = imsubtract(pimage,final);
        pimage = cleanup(pimage,1,3,0);
        pause(1);
        robcom = centerofmass(pimage,1);
       
%         pimage((robcom(1)-1:robcom(1)+2),(robcom(2)-1:robcom(2)+2)) = 0;
%         pimage(robcom(1), robcom(2)) = 1;
%         pimage((endtarget(1)-1:t1(1)+2),(endtarget(2)-1:endtarget(2)+2)) = 0;
%         pimage(endtarget(1),endtarget(2)) = 1;
        
        robresult = result;
        robresult(:,:,2) = pimage;
        
        figure(1), imshow(image);
        figure(2), imshow(pimage);
        figure(3), imshow(robresult);
        
        % Draw the CoM of the robot onto result to record the path of the
        % robot.
        result((robcom(1)-1):(robcom(1)+1),(robcom(2)-1:robcom(2)+1),1) = 0; 
        result((robcom(1)-2):(robcom(1)+2),(robcom(2)-2:robcom(2)+2),1) = 30;
        
        xerxesAtLoc = myeuclid(robcom, endtarget);
        
    end

    figure(5), imshow(theend);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Initialise takes the binary image of the maze and robot.
% Outputs:
% robot: an image containing just the robot.
% final: an image containing the final interpretation of the world (maze +
% blocks).
% P: the projection matrix.
% blockcase: since there can only be 3 variations of the blockcase, we
% represent this as an integer value (1, 2 or 3).
% resa, resb: though not strictly necessary, we output the resolution of
% final where resa is the height and resb is the width.
% robcom: the robot's initial center of mass.
% thresh: the threshold being used (incase any changes are made).
% result: coloured image to display coded areas


function [robot, final, P, blockcase, robotcase, resa, resb, robotcom, result] = initialise(bim,robot)
    
    % Create the image that will be presented when the robot reaches the
    % destination. result is a colour image with the target locations,
    % path of the robot and maze outine of the world.
    result = zeros(460, 360, 3);

    % bim: the binary image of the maze.
    % For effective projection we resize the images by a factor of 2.
    bim = im2bw(bim);
    bim = imresize(bim,2);
    robot = imresize(robot,2);
    
    % bwlabeled: label the disconnected regions of bim
    % allareas: get the areas associated to the labelled regions
    bwlabeled = bwlabel(bim,4);
    regiondata = regionprops(bwlabeled, 'Area');
    allareas = [regiondata.Area];
    
    % areas: an array to store all areas in bim greater than 20 pixels.
    % indexes: an array to store the indexes of the areas.
    areas = [];
    m = length(allareas);
    indexes = [];
    
    % Populating indexes and areas.
    for i = 1 : m
        if allareas(i) > 20
           indexes = [indexes i];
           areas = [areas allareas(i)];
        end
    end
    
    % arraymajig: an array to store the indexes of the 20 largest areas in the
    % image (i.e. in 'areas'). If there are fewer objects present arrlen
    % will be less than 20.
    % For Webots, the idea is that the image is significantly cleaner than
    % the real world (no noise/artefacts).  We therefore want to get the
    % corners by computing the compatness of every object rather than a
    % limited number.  20 is more than enough.
    arrlen = min(20,length(areas));
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
    % compacs: stores the compactness of each image section in arraymajig
    lowcompacsloc = zeros(6);
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
    % lowcompacsarea: the areas of the locations (from lowcompacsloc) in
    % bwlabeled
    cornerlocs = zeros(5);
    lowcompacsarea = zeros(6);
    
    % Populate lowcompacsarea
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
    
    % cornerformat: the center of masses of the corners arranged in the
    % format [bottomleft bottomright topleft topright].
    cornerformat = findcorners(bwlabeled, cornerlocs);
    
    % Useful debugging code incase the projection fails.
%      cf = cornerformat;
%      for i = 1 : 4
%          t1 = [cf(i) cf(i+4)];
%          bim((t1(1)-3):(t1(1)+3),(t1(2)-3:t1(2)+3)) = 0;
%          bim(t1(1),t1(2)) = 1;
%      end
%     figure(123),imshow(bim);
    
    P = projector(cornerformat);
    final = transfer(bim, P);
    
    % Consider any cases of projection failure.
    [resa resb] = size(final);
    midspace = final((resa*0.3):(resa*0.6), (1:resb));
    
    if bearea(midspace) < 1

       newcorners = [cornerformat(2,1) cornerformat(2,2); cornerformat(3,1) cornerformat(3,2); cornerformat(1,1) cornerformat(1,2); cornerformat(4,1) cornerformat(4,2)];
       P = projector(newcorners);
       final = transfer(bim,P);
       
       topspace = final((resa*0.2):(resa*0.3), (1:resb));
       botspace = final((resa*0.7):(resa*0.8), (1:resb));
       
       if bearea(topspace) < bearea(botspace)
       
            newcorners = [cornerformat(1,1) cornerformat(1,2); cornerformat(4,1) cornerformat(4,2); cornerformat(2,1) cornerformat(2,2); cornerformat(3,1) cornerformat(3,2)];
            P = projector(newcorners);
            final = transfer(bim,P);
       
       end
       
    end


    % Set the maze outline to the blue layer of result.
    result(:,:,3) = final;
    
    %Colour in the corners of result
    %UV=[[30 , 50 ]',[30,310]',[420,50]',[420,310]']';    % target points
    result(15:45,35:65,2) = final(15:45,35:65);
    result(15:45,295:325,2) = final(15:45,295:325);
    result(405:435,35:65,2) = final(405:435,35:65);
    result(405:435,295:325,2) = final(405:435,295:325);

    % Find where the blocks are.
    % blocks: the projection of the robot image.
    blocks = transfer(robot, P);
     
    % space1: the block section of the image closest to robot
    % areaspace1: the area of the section.
    space1 = blocks((resa*0.3):(resa*0.4), (resb*0.45):(resb*0.50));
    areaspace1 = bearea(space1);
    
    % Middle block section.
    space2 = blocks((resa*0.5):(resa*0.6), (resb*0.45):(resb*0.50));
    areaspace2 = bearea(space2);
    
    % Far block section.
    space3 = blocks((resa*0.7):(resa*0.8), (resb*0.45):(resb*0.50));
    areaspace3 = bearea(space3);
    
    % Testing which block case we have by comparing the area of each block
    % section. The space with the least area is the space with no block.
    % Also paint over the corresponding section in final where the blocks
    % are located. Then show these sections in the colour image.
    if (areaspace1 < areaspace2) && (areaspace1 < areaspace3)
       
        blockcase = 1;
        final(round(resa*0.40):round(resa*0.87), round(resb*0.45):round(resb*0.55))  =  1;
        result(round(resa*0.42):round(resa*0.87), round(resb*0.42):round(resb*0.58), 1)  =  0.75;
        result(round(resa*0.42):round(resa*0.87), round(resb*0.42):round(resb*0.58), 3)  =  0.75;
        
    elseif (areaspace2 < areaspace3)
            
        blockcase = 2;
        final(round(resa*0.20):round(resa*0.43), round(resb*0.45):round(resb*0.55))  =  1;
        final(round(resa*0.65):round(resa*0.87), round(resb*0.45):round(resb*0.55))  =  1;
        result(round(resa*0.22):round(resa*0.43), round(resb*0.42):round(resb*0.58),1)  =  0.75;
        result(round(resa*0.65):round(resa*0.87), round(resb*0.42):round(resb*0.58),1)  =  0.75;
        result(round(resa*0.22):round(resa*0.43), round(resb*0.42):round(resb*0.58),3)  =  0.75;
        result(round(resa*0.65):round(resa*0.87), round(resb*0.42):round(resb*0.58),3)  =  0.75; 
        
    else

        blockcase = 3;
        final(round(resa*0.20):round(resa*0.65), round(resb*0.45):round(resb*0.55))  =  1;
        result(round(resa*0.22):round(resa*0.65), round(resb*0.42):round(resb*0.58),1)  =  0.75;   
        result(round(resa*0.22):round(resa*0.65), round(resb*0.42):round(resb*0.58),3)  =  0.75;
            
    end

    % robot: the image containing the robot.
    % Since we painted the blocks into final, when we do image subtraction
    % the only object left should be the robot.
    robot = transfer(robot, P);
    robot = imsubtract(robot,final);
    robot = cleanup(robot,1,3,0);
    
    % bwrobot: robot converted to black and white (the is a safety step).
    bwrobot = im2bw(robot);
    
    % robotbwlabel: the labelled regions of bwrobot.
    % robotlargest: the image containing just the robot.
    robotbwlabel = bwlabel(bwrobot,8);
    robotlargest = getlargest(robotbwlabel,0);

    robotcom = centerofmass(robotlargest,1);
    
    % robotcase: the side of the maze the robot is starting on. We can find
    % this with regards to the center line down the image.
    if (robotcom(2) > 120)
        robotcase = 1;
    else
        robotcase = -1;
    end
    
end

% A function to get the center of mass of an object in a bwlabeled image.
% bwim: the bwlabeled image
% loc: the location of the object in bwim.
% com: the center of mass of that object.
function com = centerofmass(bwim, loc)
    % image: grab the object from bwlabeled image.
    image = imsections(bwim,loc);
    [H,W] = size(image);
    area = bearea(image);
        
    % r: the sum of the height
    % c: the sum of the width
    r = 0;
    c = 0;
    
    % Calculate
    for x = 1 : W
        for y = 1 : H
               r = r + y*image(y,x);
               c = c + x*image(y,x);
        end
    end
    
    % Get the averages (dividing by the area) and return them.
    com = [round(r/area) round(c/area)];
end

% Bearea. It just works. Bearea, the next best thing since Chuck Norris.
% Bearea, the best a bear can get.
% But seriously, bwarea doesn't get the precise area.
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

% Get the compactness of an object in an image.
% bwim: the bwlabeled image
% loc: the location of the object in the bwlabeled image
% compac: the compactness
function compac = compacty(bwim, loc)
    image = imsections(bwim,loc);
    
    area = bearea(image);
    perim = bearea(mybwperim(image));
     
    compac = (perim*perim)/(4*pi*area);
end

% The Euclidean distance between two points.
% dist: returns the distance.
function dist = myeuclid(a , b)
    
    dist = sqrt((a(1) - b(1))*(a(1) - b(1)) + (a(2) - b(2))*(a(2) - b(2)));
    
end

% Get the projection matrix by estimating the homography mapping between
% the location of four corner locations and the targets of these locations
% in the order [bottomleft bottomright topleft topright].
% P: returns the projection matrix.
% coms: the input corner locations
function P = projector(coms) 
    
    UV = [[30,50];[30,310];[420,50];[420,310]];  % target points   
    XY=coms;    % source points

    P=esthomog(UV,XY,4);    % estimate homography mapping UV to XY
    
end

% Using a projection matrix, project an image (also converting back to
% binary before returning).
% img: the image to be projected.
% P: the matrix to be used for projection.
function finalout = transfer(img, P)
    % Get input image and sizes
    % IR: the input row size
    % IC: the input column size
    inimage=img;
    [IR,IC]=size(inimage);

    % outimage: the destination image to store the projection.
    outimage=zeros(460, 360);

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

    finalout = outimage;

end

% A funtion to return the center of masses of the 4 corners in an image.
% bwlabeled: a bwlabeled image.
% cornerlocs: the location of the corners in bwalabeled, including the
% robotic terminus.
% cornerformat: the returned array of the CoMs in the format
%        [bottomleft bottomright topleft topright]
function cornerformat = findcorners(bwlabeled, cornerlocs)

    % Get the Centre Of Mass of the objects at each cornerloc
    c1com = centerofmass(bwlabeled,cornerlocs(1));
    c2com = centerofmass(bwlabeled,cornerlocs(2));
    c3com = centerofmass(bwlabeled,cornerlocs(3));
    c4com = centerofmass(bwlabeled,cornerlocs(4));
    c5com = centerofmass(bwlabeled,cornerlocs(5));

    % We know the last CoM is the terminus (based on areas). We get the
    % distance of the other corners to this.
    atoe = myeuclid(c1com, c5com);
    btoe = myeuclid(c2com,c5com);
    ctoe = myeuclid(c3com,c5com);
    dtoe = myeuclid(c4com,c5com);

    % Tuples holding the distances and the CoMs
    a = [atoe c1com];
    b = [btoe c2com];
    c = [ctoe c3com];
    d = [dtoe c4com];
   
    % We then sort the CoMs based on the distance values.
    charlie = sortrows([a; b; c; d]);
    
    c1com = [charlie(5) charlie(9)];
    c2com = [charlie(6) charlie(10)];
    c3com = [charlie(7) charlie(11)];
    c4com = [charlie(8) charlie(12)];
    
    cornerformat = [c1com; c2com; c3com; c4com];
    
end

% Takes a bwlabeled image and a location in the image and returns just the
% object.
% bwlabeled: the bwlabeled image
% secLoc: the location of the object in bwlabeled
% imsect: the result object in binary
function imsect = imsections(bwlabeled, secLoc)
    
    [u,v] = find(bwlabeled==secLoc);
    
    m = length(u);
    n = length(v);
    
    imsect = zeros(320,240);
    
    for x = 1:m
        imsect(u(x),v(x)) = 1;
    end
     
end

% Takes a photo using take_snap.m and returns the binary version of that
% image with respect to the threshold.
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

end

% Calculates the angle between two vectors using the inverse cosine.
% v1: the first vector
% v2: the second vector
% degs: the angle, in degrees, between the two vectors.
function degs = vangle(v1,v2)
    v1mag = sqrt(v1(1)*v1(1) + v1(2)*v1(2));
    v2mag = sqrt(v2(1)*v2(1) + v2(2)*v2(2));
    v1dotv2 = (v1(1)*v2(1) + v1(2)*v2(2));
    rad = acos(v1dotv2/(v1mag*v2mag));
    degs = round(180*rad/pi);
end

% Moves the robot forward with a specified speed for a specified time.
% speed: the speed
% time: the time
function moveforward(speed, time)
    % Sends the command to the robot to move at 'speed'
    send_command(speed);
    % Pauses execution for 'time'
    pause(time);
    % Sends the command to the robot to stop
    send_command('D,0,0');
    
    % This was commented for our own humour.
    % This was a long piece of coursework. :)
end
