# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
import numpy as np
import math
import cmath
import time

import RPi.GPIO as GPIO
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)
GPIO.setup(11, GPIO.IN)#PIR sensor
GPIO.setup(33,GPIO.OUT) #DC Mimport pigpio, time, crcmod.predefined, smbusimport pigpio, time, crcmod.predefined, smbusotor
p = GPIO.PWM(33,200) #GPIO13 as PWM output with 200 Hz frequency
p.start(0)
PIRcheck = 1

GPIO.setup(32, GPIO.OUT)#Servo Motor
s = GPIO.PWM(32,50)
s.start(7.5)

GPIO.setup(12,GPIO.OUT) #Solenoid Servo
ss = GPIO.PWM(12,50)
ss.start(7.5)

#D6t Sensor
import pigpio, crcmod.predefined, smbus


# constants
rotatechange = 0.5
speedchange = 0.2
occ_bins = [-1, 0, 100, 101]
stop_distance = 0.25
front_angle = 30
front_angles = range(-front_angle,front_angle+1,1)
scanfile = 'lidar.txt'
mapfile = 'map.txt'

def dcmotor():
	try:
		p.ChangeDutyCycle(100)
	except KeyboardInterrupt:
		p.stop()
		GPIO.cleanup()
		
def solenoid():
	try:
		n1 = 120
		ang1 = (n1/180*10) +2.5
		ss.ChangeDutyCycle(ang1)
	except KeyboardInterrupt:
		ss.stop()
		GPIO.cleanup

class OmronD6T(object):
   def __init__(self, rasPiChannel=1, omronAddress=0x0a, arraySize=16):
      self.MAX_RETRIES = 5
      self.roomTemp = 0
      self.omronAddress = omronAddress
      self.arraySize = arraySize
      self.BUFFER_LENGTH=(arraySize * 2) + 3       # data buffer size 
      self.CRC_ERROR_BIT = 0x04                    # the third bit
      self.CRC = 0xa4 / (16/arraySize)                             # if you take the CRC of the whole word including the PEC, this is what you get
      self.piGPIO = pigpio.pi()
      self.piGPIOver = self.piGPIO.get_pigpio_version()
      self.i2cBus = smbus.SMBus(1)
      time.sleep(0.1)                # Wait
      
      # initialize the device based on Omron's appnote 1
      self.retries = 0
      self.result = 0
      for i in range(0,self.MAX_RETRIES):
         time.sleep(0.05)                               # Wait a short time
         self.handle = self.piGPIO.i2c_open(rasPiChannel, omronAddress) # open Omron D6T device at address 0x0a on bus 1
	 #print (self.handle)
         if self.handle > 0:
            self.result = self.i2cBus.write_byte(self.omronAddress,0x4c)
            break
         else:
            print ('')
            print ('***** Omron init error ***** handle='+str(self.handle)+' retries='+str(self.retries))
            self.retries += 1
            
   # function to read the omron temperature array
   def read(self):
      self.temperature_data_raw=[0]*self.BUFFER_LENGTH
      self.temperature=[0.0]*self.arraySize         # holds the recently measured temperature
      self.values=[0]*self.BUFFER_LENGTH
      
      # read the temperature data stream - if errors, retry
      retries = 0
      for i in range(0,self.MAX_RETRIES):
         time.sleep(0.05)                               # Wait a short time
         (self.bytes_read, self.temperature_data_raw) = self.piGPIO.i2c_read_device(self.handle, self.BUFFER_LENGTH)
            
         # Handle i2c error transmissions
         if self.bytes_read != self.BUFFER_LENGTH:
            print ('')
            print ('***** Omron Byte Count error ***** - bytes read: '+str(self.bytes_read))
            
            self.retries += 1                # start counting the number of times to retry the transmission

         if self.bytes_read == self.BUFFER_LENGTH:
            # good byte count, now check PEC
            
            t = (self.temperature_data_raw[1] << 8) | self.temperature_data_raw[0]
            self.tPATc = float(t)/10
 #           if (degree_unit == 'F'):
            self.roomTemp = self.C_to_F(self.tPATc)

            # Convert Raw Values to Temperature ('F)
            a = 0
            for i in range(2, len(self.temperature_data_raw)-2, 2):
               self.temperature[a] = self.C_to_F(float((self.temperature_data_raw[i+1] << 8) | self.temperature_data_raw[i])/10)
               a += 1
            
            # Calculate the CRC error check code
            # PEC (packet error code) byte is appended at the end of each transaction. The byte is calculated as CRC-8 checksum, calculated over the entire message including the address and read/write bit. The polynomial used is x8+x2+x+1 (the CRC-8-ATM HEC algorithm, initialized to zero)
            self.crc8_func = crcmod.predefined.mkCrcFun('crc-8')
            
            for i in range(0,self.bytes_read):
               self.values[i] = self.temperature_data_raw[i]
                  
            self.string = "".join(chr(i) for i in self.values)
            self.crc = self.crc8_func(self.string.encode("utf-8"))
               
            if self.crc != self.CRC:
               print ('***** Omron CRC error ***** Expected '+'%02e'%self.CRC+' Calculated: '+'%02e'%self.crc)
               self.retries += 1                # start counting the number of times to retry the transmission
               self.bytes_read = 0           # error is passed up using bytes read
            else:
               break    # crc is good and bytes_read is good

      return self.bytes_read, self.temperature

   # function for Celsius to Fahrenheit conversion
   def C_to_F(self, degreesCelsius):
      return 9.0*degreesCelsius/5.0 + 32

omron = OmronD6T(rasPiChannel=1, omronAddress=0x0a, arraySize=16) #Change arraysize to 16?
    

#if __name__ == '__main__':
 #  omron = OmronD6T()
  # while True:
   #   omron.read()
    #  print ("ICTemp:",omron.roomTemp)
     # for i in range(0,len(omron.temperature)):
      #   print ("Cell",str(i)+":",omron.temperature[i])
      #print ('')
      #time.sleep(1)
# End of Add

# code from https://automaticaddison.com/how-to-convert-a-quaternion-into-euler-angles-in-python/
def euler_from_quaternion(x, y, z, w):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z # in radians

class AutoNav(Node):

    def __init__(self):
        super().__init__('auto_nav')
        
        # create publisher for moving TurtleBot
        self.publisher_ = self.create_publisher(Twist,'cmd_vel',10)
        # self.get_logger().info('Created publisher')
        
        # create subscription to track orientation
        self.odom_subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10)
        # self.get_logger().info('Created subscriber')
        self.odom_subscription  # prevent unused variable warning
        # initialize variables
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        
        # create subscription to track occupancy
        self.occ_subscription = self.create_subscription(
            OccupancyGrid,
            'map',
            self.occ_callback,
            qos_profile_sensor_data)
        self.occ_subscription  # prevent unused variable warning
        self.occdata = np.array([])
        
        # create subscription to track lidar
        self.scan_subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            qos_profile_sensor_data)
        self.scan_subscription  # prevent unused variable warning
        self.laser_range = np.array([])


    def odom_callback(self, msg):
        # self.get_logger().info('In odom_callback')
        orientation_quat =  msg.pose.pose.orientation
        self.roll, self.pitch, self.yaw = euler_from_quaternion(orientation_quat.x, orientation_quat.y, orientation_quat.z, orientation_quat.w)


    def occ_callback(self, msg):
        # self.get_logger().info('In occ_callback')
        # create numpy array
        msgdata = np.array(msg.data)
        # compute histogram to identify percent of bins with -1
        # occ_counts = np.histogram(msgdata,occ_bins)
        # calculate total number of bins
        # total_bins = msg.info.width * msg.info.height
        # log the info
        # self.get_logger().info('Unmapped: %i Unoccupied: %i Occupied: %i Total: %i' % (occ_counts[0][0], occ_counts[0][1], occ_counts[0][2], total_bins))

        # make msgdata go from 0 instead of -1, reshape into 2D
        oc2 = msgdata + 1
        # reshape to 2D array using column order
        # self.occdata = np.uint8(oc2.reshape(msg.info.height,msg.info.width,order='F'))
        self.occdata = np.uint8(oc2.reshape(msg.info.height,msg.info.width))
        # print to file
        # np.savetxt(mapfile, self.occdata)


    def scan_callback(self, msg):
        # self.get_logger().info('In scan_callback')
        # create numpy array
        self.laser_range = np.array(msg.ranges)
        # print to file
        # np.savetxt(scanfile, self.laser_range)
        # replace 0's with nan
        self.laser_range[self.laser_range==0] = np.nan


    # function to rotate the TurtleBot
    def rotatebot(self, rot_angle):
        # self.get_logger().info('In rotatebot')
        # create Twist object
        twist = Twist()
        
        # get current yaw angle
        current_yaw = self.yaw
        # log the info
        self.get_logger().info('Current: %f' % math.degrees(current_yaw))
        # we are going to use complex numbers to avoid problems when the angles go from
        # 360 to 0, or from -180 to 180
        c_yaw = complex(math.cos(current_yaw),math.sin(current_yaw))
        # calculate desired yaw
        target_yaw = current_yaw + math.radians(rot_angle)
        # convert to complex notation
        c_target_yaw = complex(math.cos(target_yaw),math.sin(target_yaw))
        self.get_logger().info('Desired: %f' % math.degrees(cmath.phase(c_target_yaw)))
        # divide the two complex numbers to get the change in direction
        c_change = c_target_yaw / c_yaw
        # get the sign of the imaginary component to figure out which way we have to turn
        c_change_dir = np.sign(c_change.imag)
        # set linear speed to zero so the TurtleBot rotates on the spot
        twist.linear.x = 0.0
        # set the direction to rotate
        twist.angular.z = c_change_dir * rotatechange
        # start rotation
        self.publisher_.publish(twist)

        # we will use the c_dir_diff variable to see if we can stop rotating
        c_dir_diff = c_change_dir
        # self.get_logger().info('c_change_dir: %f c_dir_diff: %f' % (c_change_dir, c_dir_diff))
        # if the rotation direction was 1.0, then we will want to stop when the c_dir_diff
        # becomes -1.0, and vice versa
        while(c_change_dir * c_dir_diff > 0):
            # allow the callback functions to run
            rclpy.spin_once(self)
            current_yaw = self.yaw
            # convert the current yaw to complex form
            c_yaw = complex(math.cos(current_yaw),math.sin(current_yaw))
            # self.get_logger().info('Current Yaw: %f' % math.degrees(current_yaw))
            # get difference in angle between current and target
            c_change = c_target_yaw / c_yaw
            # get the sign to see if we can stop
            c_dir_diff = np.sign(c_change.imag)
            # self.get_logger().info('c_change_dir: %f c_dir_diff: %f' % (c_change_dir, c_dir_diff))

        self.get_logger().info('End Yaw: %f' % math.degrees(current_yaw))
        # set the rotation speed to 0
        twist.angular.z = 0.0
        # stop the rotation
        self.publisher_.publish(twist)


    def pick_direction(self):
        # self.get_logger().info('In pick_direction')
        if self.laser_range.size != 0:
            # use nanargmax as there are nan's in laser_range added to replace 0's
            lr2i = np.nanargmax(self.laser_range)
            self.get_logger().info('Picked direction: %d %f m' % (lr2i, self.laser_range[lr2i]))
        else:
            lr2i = 0
            self.get_logger().info('No data!')

        # rotate to that direction
        self.rotatebot(float(lr2i))

        # start moving
        self.get_logger().info('Start moving')
        twist = Twist()
        twist.linear.x = speedchange
        twist.angular.z = 0.0
        # not sure if this is really necessary, but things seem to work more
        # reliably with this
        time.sleep(1)
        self.publisher_.publish(twist)


    def stopbot(self):
        self.get_logger().info('In stopbot')
        # publish to cmd_vel to move TurtleBot
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        # time.sleep(1)
        self.publisher_.publish(twist)


    def mover(self):
        try:
            # initialize variable to write elapsed time to file
            # contourCheck = 1

            # find direction with the largest distance from the Lidar,
            # rotate to that direction, and start moving
            self.pick_direction()

            while rclpy.ok():
                 if self.laser_range.size != 0:
                    # check distances in front of TurtleBot and find values less
                    # than stop_distance
                    lri = (self.laser_range[front_angles]<float(stop_distance)).nonzero()
                    # self.get_logger().info('Distances: %s' % str(lri))

                    # if the list is not empty
                    if(len(lri[0])>0):
                        # stop moving
                        self.stopbot()
                        for j in range(360):                   
                            rotate = ((j+self.yaw)%359)
                            self.rotatebot(rotate)
                            time.sleep(0.02)
                            bytes_read, temperature = omron.read()
                            temperature_array = [(temperature[0],temperature[1],temperature[2],temperature[3]),(temperature[4],temperature[5],temperature[6],temperature[7]),(temperature[8],temperature[9],temperature[10],temperature[11]),(temperature[12],temperature[13],temperature[14],temperature[15])]
                            if max(temperature) >= 90:
                                self.stopbot()
                                if max(temperature) == temperature_array[0][0] : #Top Left array
                                    self.rotatebot(self.yaw - 24)      #Move by 2 pixel units to the right
                                    n=(90-23)
                                    ang = (n/180*10) +2.5
                                    s.ChangeDutyCycle(ang)
                                elif max(temperature) == temperature_array[0][1] >= 113 : #More than or equal to 45 degree
                                    self.rotatebot(self.yaw - 12)      #Move by 1 pixel units to the right
                                    n=(90-23)
                                    ang = (n/180*10) +2.5
                                    s.ChangeDutyCycle(ang)
                                elif max(temperature) == temperature_array[0][2]:
                                    self.rotatebot(self.yaw + 12)
                                    n=(90-23)
                                    ang = (n/180*10) +2.5
                                    s.ChangeDutyCycle(ang)
                                elif  max(temperature) == temperature_array[0][3]:
                                    self.rotatebot(self.yaw + 24)
                                    n=(90-23)
                                    ang = (n/180*10) +2.5
                                    s.ChangeDutyCycle(ang)
                                elif max(temperature) == temperature_array[1][0]:
                                    self.rotatebot(self.yaw - 24)
                                    n=(90-12)
                                    ang = (n/180*10) +2.5
                                    s.ChangeDutyCycle(ang)
                                elif  max(temperature) == temperature_array[1][1]:
                                    self.rotatebot(self.yaw - 12)
                                    n=(90-12)
                                    ang = (n/180*10) +2.5
                                    s.ChangeDutyCycle(ang)
                                elif  max(temperature) == temperature_array[1][2]:
                                    self.rotatebot(self.yaw + 12)
                                    n=(90-12)
                                    ang = (n/180*10) +2.5
                                    s.ChangeDutyCycle(ang)
                                elif max(temperature) == temperature_array[1][3]:
                                    self.rotatebot(self.yaw + 24)
                                    n=(90-12)
                                    ang = (n/180*10) +2.5
                                    s.ChangeDutyCycle(ang)
                                elif  max(temperature) == temperature_array[2][0]:
                                    self.rotatebot(self.yaw - 24)
                                    n=(90+12)
                                    ang = (n/180*10) +2.5
                                    s.ChangeDutyCycle(ang)
                                elif max(temperature) == temperature_array[2][1]:
                                    self.rotatebot(self.yaw - 12)
                                    n=(90+12)
                                    ang = (n/180*10) +2.5
                                    s.ChangeDutyCycle(ang)
                                elif max(temperature) == temperature_array[2][2]:
                                    self.rotatebot(self.yaw + 12)
                                    n=(90+12)
                                    ang = (n/180*10) +2.5
                                    s.ChangeDutyCycle(ang)
                                elif max(temperature) == temperature_array[2][3]:
                                    self.rotatebot(self.yaw + 24)
                                    n=(90+12)
                                    ang = (n/180*10) +2.5
                                    s.ChangeDutyCycle(ang)
                                elif max(temperature) == temperature_array[3][0]:
                                    self.rotatebot(self.yaw - 24)
                                    n=(90+23)
                                    ang = (n/180*10) +2.5
                                    s.ChangeDutyCycle(ang)
                                elif max(temperature) == temperature_array[3][1]:
                                    self.rotatebot(self.yaw - 12)
                                    n=(90+23)
                                    ang = (n/180*10) +2.5
                                    s.ChangeDutyCycle(ang)
                                elif max(temperature) == temperature_array[3][2]:
                                    self.rotatebot(self.yaw + 12)
                                    n=(90+23)
                                    ang = (n/180*10) +2.5
                                    s.ChangeDutyCycle(ang)
                                elif max(temperature) == temperature_array[3][3]:
                                    self.rotatebot(self.yaw + 24)
                                    n=(90+23)
                                    ang = (n/180*10) +2.5
                                    s.ChangeDutyCycle(ang)
                                twist = Twist()
                                twist.linear.x = 0.05
                                twist.angular.z = 0.0
                                self.publisher_.publish(twist)
                                if self.laser_range[range(-5, 6, 1)] <= 0.40: #move to 40cm away from target
                                    self.stopbot()
                                    if max(temperature) == temperature_array[0][0] : #Top Left array
                                            self.rotatebot(self.yaw - 24)      #Move by 2 pixel units to the right
                                            n=(90-23)
                                            ang = (n/180*10) +2.5
                                            s.ChangeDutyCycle(ang)
                                    elif max(temperature) == temperature_array[0][1] >= 113 : #More than or equal to 45 degree
                                            self.rotatebot(self.yaw - 12)      #Move by 1 pixel units to the right
                                            n=(90-23)
                                            ang = (n/180*10) +2.5
                                            s.ChangeDutyCycle(ang)
                                    elif max(temperature) == temperature_array[0][2]:
                                            self.rotatebot(self.yaw + 12)
                                            n=(90-23)
                                            ang = (n/180*10) +2.5
                                            s.ChangeDutyCycle(ang)
                                    elif  max(temperature) == temperature_array[0][3]:
                                            self.rotatebot(self.yaw + 24)
                                            n=(90-23)
                                            ang = (n/180*10) +2.5
                                            s.ChangeDutyCycle(ang)
                                    elif max(temperature) == temperature_array[1][0]:
                                            self.rotatebot(self.yaw - 24)
                                            n=(90-12)
                                            ang = (n/180*10) +2.5
                                            s.ChangeDutyCycle(ang)
                                    elif  max(temperature) == temperature_array[1][1]:
                                            self.rotatebot(self.yaw - 12)
                                            n=(90-12)
                                            ang = (n/180*10) +2.5
                                            s.ChangeDutyCycle(ang)
                                    elif  max(temperature) == temperature_array[1][2]:
                                            self.rotatebot(self.yaw + 12)
                                            n=(90-12)
                                            ang = (n/180*10) +2.5
                                            s.ChangeDutyCycle(ang)
                                    elif max(temperature) == temperature_array[1][3]:
                                            self.rotatebot(self.yaw + 24)
                                            n=(90-12)
                                            ang = (n/180*10) +2.5
                                            s.ChangeDutyCycle(ang)
                                    elif  max(temperature) == temperature_array[2][0]:
                                            self.rotatebot(self.yaw - 24)
                                            n=(90+12)
                                            ang = (n/180*10) +2.5
                                            s.ChangeDutyCycle(ang)
                                    elif max(temperature) == temperature_array[2][1]:
                                            self.rotatebot(self.yaw - 12)
                                            n=(90+12)
                                            ang = (n/180*10) +2.5
                                            s.ChangeDutyCycle(ang)
                                    elif max(temperature) == temperature_array[2][2]:
                                            self.rotatebot(self.yaw + 12)
                                            n=(90+12)
                                            ang = (n/180*10) +2.5
                                            s.ChangeDutyCycle(ang)
                                    elif max(temperature) == temperature_array[2][3]:
                                            self.rotatebot(self.yaw + 24)
                                            n=(90+12)
                                            ang = (n/180*10) +2.5
                                            s.ChangeDutyCycle(ang)
                                    elif max(temperature) == temperature_array[3][0]:
                                            self.rotatebot(self.yaw - 24)
                                            n=(90+23)
                                            ang = (n/180*10) +2.5
                                            s.ChangeDutyCycle(ang)
                                    elif max(temperature) == temperature_array[3][1]:
                                            self.rotatebot(self.yaw - 12)
                                            n=(90+23)
                                            ang = (n/180*10) +2.5
                                            s.ChangeDutyCycle(ang)
                                    elif max(temperature) == temperature_array[3][2]:
                                            self.rotatebot(self.yaw + 12)
                                            n=(90+23)
                                            ang = (n/180*10) +2.5
                                            s.ChangeDutyCycle(ang)
                                    elif max(temperature) == temperature_array[3][3]:
                                            self.rotatebot(self.yaw + 24)
                                            n=(90+23)
                                            ang = (n/180*10) +2.5
                                            s.ChangeDutyCycle(ang)
             			## Let DC motor run for 10 seconds
             			## Push ping pong ball to DC motor after 3sec
                        dcmotor()
                        time.sleep(3)
                        solenoid()
                        time.sleep(10)
                        # find direction with the largest distance from the Lidar
                        # rotate to that direction
                        # start moving
                        self.pick_direction()
                    
                # allow the callback functions to run
                 rclpy.spin_once(self)

        except Exception as e:
            print(e)
        
        # Ctrl-c detected
        finally:
            # stop moving
            self.stopbot()


def main(args=None):
    rclpy.init(args=args)

    auto_nav = AutoNav()
    auto_nav.mover()

    # create matplotlib figure
    # plt.ion()
    # plt.show()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    auto_nav.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
