```
alias jetcam='clear && gst-launch-1.0 -v udpsrc port=1234 caps = "application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264, payload=(int)96" ! rtph264depay $
alias waffle1='ssh pi@192.168.1.30'
alias waffle2='ssh pi@192.168.1.35'
alias nvidia5='ssh nvidia5@192.168.1.20'

alias package='~/Desktop/CreationScriptAvecNoeud.sh'
alias bras='roslaunch bras controller.launch'
alias tuto='read -p "Quel script voulez-vous lancer (buzzer,dynamixels,test_publisher,test_subscriber) : " s && rosrun tuto $s'
```
