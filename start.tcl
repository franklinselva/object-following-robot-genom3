package require genomix
genomix::connect
genomix1 rpath /home/felix/work/lib/genom/ros/plugins/
genomix1 load CT_robot

proc init {} {
    CT_robot::SetVerbose 1
    CT_robot::Set_my_r 3 
    CT_robot::Set_my_g 2
    CT_robot::Set_my_b 105
    CT_robot::Set_my_seuil 40
    CT_robot::connect_port {local ImagePort remote /camera/image_raw}
} 
