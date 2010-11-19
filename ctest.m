function ctest(direction)
    open_robot;
    
    send_command('D,4,-4');
    read_command;
    
    pause(1.3);
    
    send_command('D,0,0');
    read_command;
    
    close_robot;
end