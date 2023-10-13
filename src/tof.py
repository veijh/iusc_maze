#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
import tfluna
import timeout_decorator

if __name__ == '__main__':
    try:
        rospy.init_node('tof_node', anonymous=True)

        # 创建一个发布者，发布类型为Float64的消息到名为 'tof' 的话题
        pub = rospy.Publisher('tof', Float64, queue_size=10)

        rate = rospy.Rate(10)  # 发布频率为10Hz

        with tfluna.TfLuna(baud_speed=115200, serial_name="/dev/ttyUSB0") as tfluna:
            tfluna.get_version()
            tfluna.set_samp_rate(10)
            while not rospy.is_shutdown():
                # 读取距离
                distance, strength, temperature = tfluna.read_tfluna_data()
                # 创建一个Float64消息
                message = Float64()
                message.data = distance  # 设置消息数据

                # 发布消息
                pub.publish(message)
                # rospy.loginfo("Published: %f" % message.data)

                rate.sleep()

    except rospy.ROSInterruptException:
        pass
