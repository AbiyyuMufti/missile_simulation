function foldwings()
%FOLDWINGS Summary of this function goes here
%   Detailed explanation goes here
    try
        rosinit('http://MDY:11311/');
    catch exception
        disp("Already Connected to ROS MASTER");
    end
    fins1pub = rospublisher("/exocet_mm40b3/fins_fold1_position_controller/command", "std_msgs/Float64");
    fins2pub = rospublisher("/exocet_mm40b3/fins_fold2_position_controller/command", "std_msgs/Float64");
    fins3pub = rospublisher("/exocet_mm40b3/fins_fold3_position_controller/command", "std_msgs/Float64");
    fins4pub = rospublisher("/exocet_mm40b3/fins_fold4_position_controller/command", "std_msgs/Float64");
    wing1pub = rospublisher("/exocet_mm40b3/wings_fold1_position_controller/command", "std_msgs/Float64");
    wing2pub = rospublisher("/exocet_mm40b3/wings_fold2_position_controller/command", "std_msgs/Float64");
    wing3pub = rospublisher("/exocet_mm40b3/wings_fold3_position_controller/command", "std_msgs/Float64");
    wing4pub = rospublisher("/exocet_mm40b3/wings_fold4_position_controller/command", "std_msgs/Float64");
    
    foldmsg = rosmessage('std_msgs/Float64');
    foldmsg.Data = -2;
    fins1pub.send(foldmsg)
    fins2pub.send(foldmsg)
    fins3pub.send(foldmsg)
    fins4pub.send(foldmsg)
    wing1pub.send(foldmsg)
    wing2pub.send(foldmsg)
    wing3pub.send(foldmsg)
    wing4pub.send(foldmsg)
    pause(3);
end

