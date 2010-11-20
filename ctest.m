function ctest()

    X = uint8([ 255 10 75; 44 225 100]);
    Y = uint8([ 50 50 50; 50 50 50 ]);
    Z = imabsdiff(X,Y)
    

%     turnrate = 20.7767/360;
%     
%     input('blank');
%     unix(['mplayer tv:// -tv driver=v4l:width=640:height=480:device=/dev/video0 -frames 6 -vo jpeg']);
%     final = '00000006.jpg';
%     
%     input('targ');
%     unix(['mplayer tv:// -tv driver=v4l:width=640:height=480:device=/dev/video0 -frames 5 -vo jpeg']);
%     target = '00000005.jpg';
%     
%     input('rob');
%     unix(['mplayer tv:// -tv driver=v4l:width=640:height=480:device=/dev/video0 -frames 4 -vo jpeg']);
%     picture = '00000004.jpg';
%     
%     final = binarypic(final, 0.7);
%     target = binarypic(target, 0.7);
%     picture = binarypic(picture, 0.7);
%     
% 
%    pimage = picture - target; 
%    
%    target = target - final;   
%     
%    pimage = cleanup(pimage,1,3,0);     
%    
%    figure(2)
%    imshow(pimage);
%    
%    figure(3)
%    imshow(target);
%    robotbwlabel = bwlabel(pimage,8);
%    targeted = bwlabel(target,8);
%    targeted = getLargest(targeted,0);
%    robotlargest = getlargest(robotbwlabel,0);
%  
%    
%    targ = centerofmass(targeted,1);
%    prevcom = centerofmass(robotlargest,1)  
% 
%    open_robot;
%             
%    send_command('D,5,5');
%    
%    pause(1);
%    
%    send_command('D,0,0');
%     
%    unix(['mplayer tv:// -tv driver=v4l:width=640:height=480:device=/dev/video0 -frames 4 -vo jpeg']);
%    picture = '00000004.jpg';
%    
%    picture = binarypic(picture, 0.7);
%    pimage = picture - target;
%    pimage = pimage - final;
%   
%    pimage = cleanup(pimage,1,3,0);     
%    
%    figure(4)
%    imshow(pimage);
%    robotbwlabel = bwlabel(pimage,8);
%    
%    robotlargest = getlargest(robotbwlabel,1);
%    
%    robcom = centerofmass(robotlargest,1)  
%    
%    v1 = robcom - prevcom
%    
%    v2 = targ - robcom
%    
%    vangle(v1,v2)
% 
%    xerxesAtLoc = myeuclid(robcom, targ)
%         
%    while (xerxesAtLoc > 15)
%         
%         v1 = robcom - prevcom;
%         v2 = targ - robcom;
%         
%         angle = vangle(v1,v2);
%                 
%         if (robcom(2) > targ(2))
%             send_command('D,1,-1');
%         elseif (robcom(2) < targ(2))
%             send_command('D,-1,1');         
%         end
%         
%         pause(angle*turnrate);
%         send_command('D,0,0');
%         
%         send_command('D,5,5');
%         
%         pause (1);
% 
%         send_command('D,0,0');
%         
%         %turnslightly(-offby,0.2);
%        
%         prevcom = robcom;
%         
%         unix(['mplayer tv:// -tv driver=v4l:width=640:height=480:device=/dev/video0 -frames 4 -vo jpeg']);
%         picture = '00000004.jpg';
%    
%         picture = binarypic(picture, 0.7);
%         pimage = picture - target;
%         pimage = pimage - final;
%   
%         pimage = cleanup(pimage,1,3,0);     
%    
%         robotbwlabel = bwlabel(pimage,8);
% 
%         robotlargest = getlargest(robotbwlabel,1);
%    
%         robcom = centerofmass(robotlargest,1)  
%    
%         xerxesAtLoc = myeuclid(robcom, targ)
%             
%    end
%     
%     close_robot;
   
end

function dist = myeuclid(a , b)
    
    dist = sqrt((a(1) - b(1))*(a(1) - b(1)) + (a(2) - b(2))*(a(2) - b(2)));
    
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

function degs = vangle(v1,v2)
    v1mag = sqrt(v1(1)*v1(1) + v1(2)*v1(2));
    v2mag = sqrt(v2(1)*v2(1) + v2(2)*v2(2));
    v1dotv2 = (v1(1)*v2(1) + v1(2)*v2(2));
    rad = acos(v1dotv2/(v1mag*v2mag));
    degs = round(180*rad/pi);
end