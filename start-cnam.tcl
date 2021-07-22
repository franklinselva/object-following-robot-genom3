package require genomix
genomix::connect
genomix1 rpath /home/felix/work/lib/genom/ros/plugins/
genomix1 load cnam

proc init {} {
    cnam::SetVerbose 1
    cnam::Set_my_r 3 
    cnam::Set_my_g 2
    cnam::Set_my_b 105
    cnam::Set_my_seuil 40
    cnam::connect_port {local ImagePort remote /camera/image_raw}
} 
