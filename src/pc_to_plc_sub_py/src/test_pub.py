#!/usr/bin/env python3

import rospy
# vkljucitev tipa Int64 iz std_msgs
from beckhoff_msgs.msg import array5

if __name__ == '__main__':
    # node inicializaija
    rospy.init_node("number_publisher", anonymous=True)
    rospy.loginfo('This node started.')
    # deklaracija publisherja
    pub = rospy.Publisher("/omegas_delta_sub", array5, queue_size=10)
    # nastavitev frekvence izvajanja - 2 Hz
    rate = rospy.Rate(2)

    print('Sending number.')
    
    # dokler se node ne ugasne, izvajaj sledeco kodo
    while not rospy.is_shutdown():
        # deklaracija sporocila
        msg = array5()
        # definicija vrednosti sporocila
        msg.data[0] = 3
        # posiljanje sporocila
        pub.publish(msg)
        # zakasnitev, da se zagotovi ustrezno frekvenco izvajanja
        rate.sleep()