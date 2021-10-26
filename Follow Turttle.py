#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import math
from turtlesim.msg import Pose		
from geometry_msgs.msg import Twist
import json


pose = Pose()
pose2= Pose()		
		
poseFlag= False
poseFlag2= False

findFlag = False	# turtlelar bir birine değerse yani yakaladığın dair flag


with open('info.json','r') as files:
	content = files.read()
files.close()

veri =json.loads(content)

veri['Mesafe']





def callbackPose1(msg):
	print("Pose: ",msg.x ,msg.y ,msg.theta)
	global pose			
	global poseFlag
	pose = msg
	poseFlag = True			

def move1(speed): 		
	while poseFlag==False :
		rospy.sleep(0.01)	
	

	
	
	vel_msg = Twist()		
	#while True:	# sistem bizden surekli publish etmemizi istiyor bu nedenle while açtik
	dist = math.sqrt((pose.x-pose2.x)**2 + (pose.y-pose2.y)**2)	
	
	if dist > veri['Mesafe']:	# 1 birim mesafeden takip edilecek		
		vel_msg.linear.x = speed	# lineardeki hıza speed değerini atadı
		vel_pub.publish(vel_msg)	# vel_msg yi sisteme publish etti gonderdi --- topiğe veri gonderiyor

#	elif dist >0.1:				#takip 1 birim icine girdiginde hizi yariya dus
#		vel_msg.linear.x = speed * 0.5	
#		vel_pub.publish(vel_msg)
#		findFlag = True
		
	else:
		vel_msg.linear.x = 0		# burda durma kosulu sağlandıgından speed 0 olucak
		vel_pub.publish(vel_msg)
		global findFlag
		findFlag = True
	loop_rate.sleep()


def rotation1(speed):
	while poseFlag==False :
		rospy.sleep(0.01)	
	
	
	vel_msg = Twist()		
#	while True:	
	Q = math.atan2(pose2.y-pose.y,pose2.x-pose.x)	#pozisyonlar bakarak bir aci olusturuyo ve o tarafa dogru yonelecek
	
#	Q = math.atan2(y0-pose.y,x0-pose.x)
	Q = Q - pose.theta
#	print(Q)
	angleDiff=math.atan2(math.sin(Q),math.cos(Q)) 			#abs(Q - pose.theta)		
#		print("Angle diff: ",diff)

	if angleDiff <0 :
		speed = -speed 
		angleDiff = abs(angleDiff)
#		print("Angle diff: ",angleDiff)
	
#		print("angular speed: ",speed)

	if angleDiff>0.1:
		vel_msg.angular.z = speed	
		vel_pub.publish(vel_msg)	# vel_msg yi sisteme publish etti gonderdi
		
		
	else:
		vel_msg.angular.z = 0		# burda durma kosulu sağlandıgından speed 0 olucak
		vel_pub.publish(vel_msg)
#			break
		
	loop_rate.sleep()			#ros rate garanti ediyo

###################################################----------------------------------------################################################## turtle2 icin kodlar

def callbackPose2(msg):
	print("Pose: ",msg.x ,msg.y ,msg.theta)
	global pose2		
	global poseFlag2
	pose2= msg
	poseFlag2 = True			

def move2(speed):
				
	while poseFlag2==False :
		rospy.sleep(0.01)	
	
	#x0=pose.x			#bunlar .x .y ler ------rosmsg show turtle/Pose ------  diyerek gorebilriz 
	#y0=pose.y

	
	vel_msg2 = Twist()		
		
	vel_msg2.linear.x = speed	# lineardeki hıza speed değerini atadı
	vel_pub2.publish(vel_msg2)	# vel_msg yi sisteme publish etti gonderdi --- topiğe veri gonderiyor
	loop_rate.sleep()


def rotation2(speed):	#koselere geldiginde 90 derece yansima acisiyla devam edice hiz belirli angle degeri carpmadan onceki acisi
	 
	angle =pose2.theta

	vel_msg2 = Twist()
	if pose2.x>11 or pose2.x<0.1 or pose2.y>11 or pose2.y<0.1:
		
		diff=abs(pose2.theta-(angle+(math.pi/2)))		# 90 dereece açı yaparak devam edicek 	
		
		if diff>0.01:
			
			vel_msg2.angular.z = speed	
			vel_pub2.publish(vel_msg2)	# vel_msg2 yi sisteme publish etti gonderdi
		
		else:
			vel_msg2.angular.z = 0		# burda durma kosulu sağlandıgından speed 0 olucak
			vel_pub2.publish(vel_msg2)
			
	else :
		pass
	loop_rate.sleep()			#ros rate garanti ediyo

if __name__ == '__main__':
	rospy.init_node('controller',anonymous=True)
	rospy.Subscriber('/turtle1/pose', Pose, callbackPose1) 			
	

	vel_pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=5)	# burda da turtle a veri gondermek için kulanıyıruz 
		
############  turtle2 için gerekli publisher ve subscriber							
										
	rospy.Subscriber('/turtle2/pose', Pose, callbackPose2) 			
	vel_pub2 = rospy.Publisher('/turtle2/cmd_vel', Twist, queue_size=5)	


	loop_rate = rospy.Rate(5)		#ros cycle frekansı ,hangi siklikla donecek saniyede 5 kere olucakğı için ole yazdık (rosun çalışma hizi)
	
#	while not(findFlag) :			# yakaladıgında sistem durur ,takip sonsuza kadar devam etmemesi icin bunu yaptim
	while True:
		move2(veri['linearSpeed'] * 0.5)
		rotation2(veri['angularSpeed'])
		move1(veri['linearSpeed'])
		rotation1(veri['angularSpeed'])			#,pose2.x,pose2.y)
	


	rospy.spin()







