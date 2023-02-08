echo  'KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60",ATTRS{serial}=="30beec09a5b3e811b8ef935d362d7dce", MODE:="0777", GROUP:="dialout", SYMLINK+="UWBT0"' >/etc/udev/rules.d/uwbt0.rules


service udev reload
sleep 2
service udev restart
