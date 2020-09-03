package require genomix
genomix::connect
genomix1 rpath /home/felix/work/lib/genom/ros/plugins/
genomix1 load cnam

proc init {} {
    cnam::SetVerbose 1
    cnam::Set_my_r 213 
    cnam::Set_my_g 103
    cnam::Set_my_b 4
    cnam::Set_my_seuil 40
    cnam::connect_port {local ImagePort remote /TTRK/CameraMain/image}
} 
