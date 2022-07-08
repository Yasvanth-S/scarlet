sudo modprobe cp210x
echo 10c4 0001 | sudo tee /sys/bus/usb-serial/drivers/cp210x/new_id
echo 10c4 0002 | sudo tee /sys/bus/usb-serial/drivers/cp210x/new_id
echo 10c4 0003 | sudo tee /sys/bus/usb-serial/drivers/cp210x/new_id
echo 10c4 0004 | sudo tee /sys/bus/usb-serial/drivers/cp210x/new_id
echo 10c4 0005 | sudo tee /sys/bus/usb-serial/drivers/cp210x/new_id
