import rclpy
from rclpy.node import Node
from deepracer_interfaces_pkg.msg import EvoSensorMsg
import os
from datetime import datetime
import pickle

class dataSubscriber(Node):
        total_images = []
        total_lidar = []

        def __init__(self):
            super().__init__('data_subscriber')


            #Create data directory
            cur_dir = os.getcwd()
            
            c_date = datetime.today()
            date_str  = c_date.strftime("%Y_%m_%d_%H_%M_%S")
            self.data_dir = os.path.join(cur_dir, date_str)
            os.mkdir(self.data_dir)

            self.subscription = self.create_subscription(
                EvoSensorMsg,
                '/sensor_fusion_pkg/sensor_msg',
                self.listener_callback,
                100)
            #self.subscription  # prevent unused variable warning

        def listener_callback(self, msg):
            #MSg has two things 1) images and 2) lidar_data
            #images of type list
            images = msg.images
            #lidar is of type array
            lidar_data = msg.lidar_data
            #self.get_logger().info('I heard something')
            self.total_images.append(images)
            self.total_lidar.append(lidar_data)
            #Fill this up in RAM, then when it reaches a certain amount spin off seperate process to 
            #save this data to disk and continue to store in RAM. The data size itself is really not that
            #big so that's helpful as well
            print('Total data saved: ', len(self.total_images), len(self.total_lidar))

            

def main(args=None):
    rclpy.init(args=args)
    
    data_subscriber = dataSubscriber()
    print('Spinning Node')
    rclpy.spin(data_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    print('Destrotying Node')
    data_storage.data_subscriber()
    rclpy.shutdown()


if __name__ == '__main__':
    main()