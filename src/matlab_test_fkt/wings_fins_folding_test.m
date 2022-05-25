N = ros.Node("WingsFoldControl",'http://MDY:11311/');
% try
%     rosinit('http://MDY:11311/');
% catch exception
%     disp("Already Connected to ROS MASTER")
% end
fins1pub = ros.Publisher(N,"/exocet_mm40b3/fins_fold1_position_controller/command", "std_msgs/Float64");
fins2pub = ros.Publisher(N,"/exocet_mm40b3/fins_fold2_position_controller/command", "std_msgs/Float64");
fins3pub = ros.Publisher(N,"/exocet_mm40b3/fins_fold3_position_controller/command", "std_msgs/Float64");
fins4pub = ros.Publisher(N,"/exocet_mm40b3/fins_fold4_position_controller/command", "std_msgs/Float64");
wing1pub = ros.Publisher(N,"/exocet_mm40b3/wings_fold1_position_controller/command", "std_msgs/Float64");
wing2pub = ros.Publisher(N,"/exocet_mm40b3/wings_fold2_position_controller/command", "std_msgs/Float64");
wing3pub = ros.Publisher(N,"/exocet_mm40b3/wings_fold3_position_controller/command", "std_msgs/Float64");
wing4pub = ros.Publisher(N,"/exocet_mm40b3/wings_fold4_position_controller/command", "std_msgs/Float64");

foldmsg = rosmessage('std_msgs/Float64');
foldmsg.Data = -2.0;
fins1pub.send(foldmsg)
fins2pub.send(foldmsg)
fins3pub.send(foldmsg)
fins4pub.send(foldmsg)
wing1pub.send(foldmsg)
wing2pub.send(foldmsg)
wing3pub.send(foldmsg)
wing4pub.send(foldmsg)

% while(true)
% 
% end