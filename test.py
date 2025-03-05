import can
import usb
dev = usb.core.find(idVendor=0x1D50, idProduct=0x606F)

bus = can.Bus(interface="gs_usb", channel=dev.product, index=0, bitrate=1000000)


None