echo 'Re-routing ardrone2_064123 to DSLnet'
nmcli con up id ARDrone_2_32
echo "./data/wifi.sh" | telnet 192.168.1.1

echo 'Connecting to DSLnet'
nmcli con up id DSL_DroneNet
