import omrond6t
omron = OmronD6T(rasPiChannel=1, omronAddress=0x0a, arraySize=8) #Change arraysize to 16?
bytes_read, temperature = omron.read()
print ("Bytes read:", bytes_read)
print ("Temperature List:", temperature) 

#Bytes Read: 19
#Temerature List: [73.55, 73.55, 73.55, 73.55, 73.55, 73.55, 73.55, 73.55]
#Temperature is an array

#Uncomment this
#Temperature_array = [(temperature[0],temperature[1],temperature[2],temperature[3]),(temperature[4],temperature[5],temperature[6],temperature[7]),(temperature[8],temperature[9],temperature[10],temperature[11]),(temperature[12],temperature[13],temperature[14],temperature[15])]

#For every 40cm, each pixel dimension is 8.4cm by 8.1cm
#if max(temperature) >= 113:
	#if max(temperature) == temperature_array[0][0] : #Top Left array
		#self.rotatebot(self.yaw - 24)      #Move by 2 pixel units to the right
		#n=(90-23)
		#ang = (n/180*10) +2.5
		#s.ChangeDutyCycle(ang)
	#else if max(temperature) == temperature_array[0][1] >= 113 : #More than or equal to 45 degree
		#self.rotatebot(self.yaw - 12)      #Move by 1 pixel units to the right
		#n=(90-23)
		#ang = (n/180*10) +2.5
		#s.ChangeDutyCycle(ang)
	#else if max(temperature) == temperature_array[0][2]:
		#self.rotatebot(self.yaw + 12)
		#n=(90-23)
		#ang = (n/180*10) +2.5
		#s.ChangeDutyCycle(ang)
	#else if max(temperature) == temperature_array[0][3]:
		#self.rotatebot(self.yaw + 24)
		#n=(90-23)
		#ang = (n/180*10) +2.5
		#s.ChangeDutyCycle(ang)
	#else if max(temperature) == temperature_array[1][0]:
		#self.rotatebot(self.yaw - 24)
		#n=(90-12)
		#ang = (n/180*10) +2.5
		#s.ChangeDutyCycle(ang)
	#else if max(temperature) == temperature array[1][1]:
		#self.rotatebot(self.yaw - 12)
		#n=(90-12)
		#ang = (n/180*10) +2.5
		#s.ChangeDutyCycle(ang)
	#else if max(temperature) == temperature array[1][2]:
		#self.rotatebot(self.yaw + 12)
		#n=(90-12)
		#ang = (n/180*10) +2.5
		#s.ChangeDutyCycle(ang)
	#else if max(temperature) == temperature array[1][3]:
		#self.rotatebot(self.yaw + 24)
		#n=(90-12)
		#ang = (n/180*10) +2.5
		#s.ChangeDutyCycle(ang)
	#else if max(temperature) == temperature array[2][0]:
		#self.rotatebot(self.yaw - 24)
		#n=(90+12)
		#ang = (n/180*10) +2.5
		#s.ChangeDutyCycle(ang)
	#else if max(temperature) == temperature array[2][1]:
		#self.rotatebot(self.yaw - 12)
		#n=(90+12)
		#ang = (n/180*10) +2.5
		#s.ChangeDutyCycle(ang)
	#else if max(temperature) == temperature array[2][2]:
		#self.rotatebot(self.yaw + 12)
		#n=(90+12)
		#ang = (n/180*10) +2.5
		#s.ChangeDutyCycle(ang)
	#else if max(temperature) == temperature array[2][3]:
		#self.rotatebot(self.yaw + 24)
		#n=(90+12)
		#ang = (n/180*10) +2.5
		#s.ChangeDutyCycle(ang)
	#else if max(temperature) == temperature array[3][0]:
		#self.rotatebot(self.yaw - 24)
		#n=(90+23)
		#ang = (n/180*10) +2.5
		#s.ChangeDutyCycle(ang)
	#else if max(temperature) == temperature array[3][1]:
		#self.rotatebot(self.yaw - 12)
		#n=(90+23)
		#ang = (n/180*10) +2.5
		#s.ChangeDutyCycle(ang)
	#else if max(temperature) == temperature array[3][2]:
		#self.rotatebot(self.yaw + 12)
		#n=(90+23)
		#ang = (n/180*10) +2.5
		#s.ChangeDutyCycle(ang)
	#else if max(temperature) == temperature array[3][3]:
		#self.rotatebot(self.yaw + 24)
		#n=(90+23)
		#ang = (n/180*10) +2.5
		#s.ChangeDutyCycle(ang)

	
