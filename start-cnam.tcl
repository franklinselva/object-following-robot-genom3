package require genomix
genomix::connect
genomix1 rpath /opt/openrobots/lib/genom/ros/plugins/
genomix1 load cnam

proc init {} {
    cnam::SetVerbose 1
    cnam::Set_my_r 213 
    cnam::Set_my_g 103
    cnam::Set_my_b 4
    cnam::Set_my_seuil 40
    cnam::connect_port {local Image remote /TTRK/CameraMain/image}
} 
