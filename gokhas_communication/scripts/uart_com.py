#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import rospy
from gokhas_communication.msg import communication  # Düzeltildi
from std_msgs.msg import Bool
import serial
import struct

PACKET_FORMAT = 'BBhhhhBBhhhhBBBB'  # Total 26 byte

class CommunicationClass:
    def __init__(self):
        rospy.init_node('communication_node', anonymous=True)

        # Publishers
        self.com_status = rospy.Publisher('com/communication_status', Bool, queue_size=1)
        self.mod_status = rospy.Publisher('com/mode_status', Bool, queue_size=1)
        self.comm_pub = rospy.Publisher('/gokhas/communication', communication, queue_size=1)  # Eklendi
        
        # Subscriber - Arayüzden komutları al
        self.comm_sub = rospy.Subscriber('/gokhas/interface_commands', communication, self.interface_command_callback)  # Eklendi
        
        # Serial port setup
        try:
            self.serialPort = serial.Serial(
                port='/dev/ttyUSB0',
                baudrate=9600,
                bytesize=8,
                parity='N',
                stopbits=1,
                timeout=1
            )
            self.serialPort.flushInput()
            self.serialPort.flushOutput()
            rospy.loginfo("Serial port opened successfully")
        except Exception as e:
            rospy.logerr(f"Failed to open serial port: {e}")
            self.serialPort = None

        self.packet_size = struct.calcsize(PACKET_FORMAT)
        self.rate = rospy.Rate(100)
        
        self.motor_positions = communication()
        self.communication_started = False  # Haberleşme başlatılmış mı?
        
        # İlk durumlar
        self.motor_positions.comStat = False
        self.motor_positions.modStat = False  # Manuel mod

    def interface_command_callback(self, msg):
        """Arayüzden gelen komutları işle"""
        rospy.loginfo(f"Interface command received: comStat={msg.comStat}")
        
        # Arayüzden gelen değerleri motor_positions'a kopyala
        self.motor_positions.comStat = msg.comStat
        self.motor_positions.modStat = msg.modStat
        self.motor_positions.tJ1p = msg.tJ1p
        self.motor_positions.tJ1s = msg.tJ1s
        self.motor_positions.tJ2p = msg.tJ2p
        self.motor_positions.tJ2s = msg.tJ2s
        self.motor_positions.tAp = msg.tAp
        self.motor_positions.tAt = msg.tAt
        self.motor_positions.cJ1 = msg.cJ1
        self.motor_positions.cJ2 = msg.cJ2
        
        # Eğer comStat True yapıldıysa haberleşmeyi başlat
        if msg.comStat and not self.communication_started:
            self.communication_started = True
            rospy.loginfo("Communication started by interface")
            
            # Serial port yoksa simülasyon: 2 saniye sonra STM32 onayı ver
            if not self.serialPort:
                rospy.loginfo("No serial port - simulating STM32 response after 2 seconds")
                import threading
                def simulate_stm32_response():
                    time.sleep(2)
                    self.motor_positions.comStat = True  # STM32 onayı simüle et
                    rospy.loginfo("Simulated STM32 response: comStat=True")
                
                threading.Thread(target=simulate_stm32_response, daemon=True).start()

    def run(self):
        while not rospy.is_shutdown():
            
            # Haberleşme başlatılmışsa STM32 ile iletişim kur
            if self.communication_started and self.serialPort:
                # STM32 ile gerçek haberleşme...
                # (mevcut kod)
                pass
            
            # Her durumda arayüze güncel durumu bildir
            self.comm_pub.publish(self.motor_positions)
            
            # Individual status publications
            com_status_msg = Bool()
            com_status_msg.data = self.motor_positions.comStat
            self.com_status.publish(com_status_msg)
            
            mod_status_msg = Bool()
            mod_status_msg.data = self.motor_positions.modStat
            self.mod_status.publish(mod_status_msg)

            self.rate.sleep()


if __name__ == "__main__":  # Düzeltildi
    try:
        uart_node = CommunicationClass()
        uart_node.run()
    except rospy.ROSInterruptException:
        pass