#!/usr/bin/env python3
import rospy
import serial
import os
from datetime import datetime
from std_msgs.msg import String


if __name__ == '__main__':
    rospy.init_node('QR_Okuyucu')
    pub = rospy.Publisher('qr_topic', String, queue_size=10)

    while True:
        try:
            port = rospy.get_param('/QR_Okuyucu/port', default="/dev/ttyUSB1")
            print("Port dinlemeye alındı: ", port)
            port_onay_kodu = "sudo chmod 666 " + port
            os.system(port_onay_kodu)
            seriiletisim = serial.Serial(port, bytesize=8, baudrate=115200)

            while not rospy.is_shutdown():
                try:
                    if seriiletisim.inWaiting() > 0:
                        gelen_veri = seriiletisim.read().decode("utf-8")
                        biriken_veri = ""
                        while gelen_veri != "\r":
                            biriken_veri += gelen_veri
                            gelen_veri = seriiletisim.read().decode("utf-8")
                        seriiletisim.read()
                        print(datetime.now().strftime(
                            "%H:%M:%S") + " ==>  " + biriken_veri)
                        pub.publish(biriken_veri)

                except Exception as e:
                    print(e)
                    print(
                        "Barkod Okuyucu ikinci try-except denemesinde hata meydana geldi!")
                    rospy.sleep(1)

            print("Barkod Okuyucudaki rospy çalışmıyor. Yeniden deneniyor...")
            rospy.sleep(1)

        except Exception as e:
            print(e)
            print("Barkod Okuyucu ilk try-except denemesinde hata meydana geldi!")
            rospy.sleep(1)
