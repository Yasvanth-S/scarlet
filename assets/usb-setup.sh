sudo modprobe cp210x
echo 0001 0001 | sudo tee /sys/bus/usb-serial/drivers/cp210x/new_id
echo 0002 0002 | sudo tee /sys/bus/usb-serial/drivers/cp210x/new_id
echo 0003 0003 | sudo tee /sys/bus/usb-serial/drivers/cp210x/new_id
echo 0004 0004 | sudo tee /sys/bus/usb-serial/drivers/cp210x/new_id
echo 10c4 0005 | sudo tee /sys/bus/usb-serial/drivers/cp210x/new_id
echo 10c4 0006 | sudo tee /sys/bus/usb-serial/drivers/cp210x/new_id