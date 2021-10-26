#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
import rospy
import numpy as np
from sensor_msgs.msg import NavSatFix, Imu

global gps #= NatSatFix()
global imu #= Imu()

#global std_acc

origin_lat = 40.9955106085895
origin_lon = 29.062988181734983

dt= 0.5





def gps_callBack(msg):
    global gps
    gps = msg
    # latitude = msg.latitude
    # longitude = msg.longitude
    # cov = msg.position_covariance

def imu_callBack(msg):
    global imu
    imu = msg
    # x_ivme = imu.linear_acceleration.x
    # y_ivme = imu.linear_acceleration.y
    # w_ = imu.angular_velocity.z
    #cov = imu.linear_acceleration_covariance



def get_local_coord( lat, lon):
        WORLD_POLAR_M = 6356752.3142
        WORLD_EQUATORIAL_M = 6378137.0

        eccentricity = math.acos(WORLD_POLAR_M/WORLD_EQUATORIAL_M)
        n_prime = 1/(math.sqrt(1 - math.pow(math.sin(math.radians(float(lat))),2.0)*math.pow(math.sin(eccentricity), 2.0)))
        m = WORLD_EQUATORIAL_M * math.pow(math.cos(eccentricity), 2.0) * math.pow(n_prime, 3.0)
        n = WORLD_EQUATORIAL_M * n_prime

        diffLon = float(lon) - float(origin_lon)
        diffLat = float(lat) - float(origin_lat)

        surfdistLon = math.pi /180.0 * math.cos(math.radians(float(lat))) * n
        surfdistLat = math.pi/180.00 * m

        x = diffLon * surfdistLon
        y = diffLat * surfdistLat

        return x,y










def Kalman(x_ivme,y_ivme,F, G, H, Q  ,R , gpsX , gpsY):
    #global
    # x_konum = x_ivme * (dt ** 2)
    # y_konum = x_ivme * (dt ** 2)
    while True :
        rospy.sleep(dt)


        x_Hiz = x_ivme * dt
        y_Hiz = y_ivme * dt
        gpsX , gpsY = get_local_coord(gps.latitude,gps.longitude)
        #stateX = np.array([x_konum, y_konum, x_Hiz, y_Hiz])
        #stateX = np.array([gpsX, gpsY, x_Hiz, y_Hiz])
        stateX = np.array([gpsX, gpsY, imu.linear_acceleration.x*dt, imu.linear_acceleration.y*dt])
        #stateX = np.array([gps.latitude, gps.longitude, imu.linear_acceleration.x*dt,
        #                    dt*imu.linear_acceleration.y])

        stateX = stateX.reshape(4, 1)

        #stateU = np.array([x_ivme, y_ivme])
        stateU = np.array([imu.linear_acceleration.x, imu.linear_acceleration.y])
        stateU = stateU.reshape(2, 1)

        X = np.dot(F, stateX) + np.dot(G, stateU)  # motion model

        Y = np.dot(H, stateX)  # meansurement model

        P = np.eye(F.shape[1])  # F boyutu kadarlik birim matris




        ########################################################## Prediction

        # P= F*P*F' + Q
        P = np.dot(np.dot(F,P) , F.T) + Q

        ######################################################### optimal gain
        #  K'= H*P*H'+R
        K = np.dot(np.dot(H,P),H.T) + R

        # K = P * H'* inv(H*P*H' + R)
        K = np.dot(np.dot(P,H.T), np.linalg.inv(K) )

        ########################################################## Correction
        # X = X' + K* (Y - (H*X') )
        newX = X + np.dot(K , (Y - np.dot(H,X)))

        I = np.eye(H.shape[1])

        # yeni kovaryans P = ( I - ( K * H )) * P'
        newP = np.dot( (I -np.dot(K,H)) , P)
        print("***********************")
        print(newX)
        #print("New Y : \n",newP)
        X = newX
        P = newP






if __name__ == '__main__':
    rospy.init_node('kalman_filter', anonymous=True)
    rospy.Subscriber('/gps/fix',NavSatFix,gps_callBack)
    rospy.Subscriber('/imu', Imu, imu_callBack)

    loop_rate = rospy.Rate(5)

    # formullere gore dizayn edildi
    F = np.array([1,0,dt,0,0,1,0,dt,0,0,1,0,0,0,0,1])
    F = F.reshape(4,4)

    G = np.array([(dt**2)/2 , 0 , 0 ,(dt**2)/2,dt , 0, 0 ,dt])
    G = G.reshape(4,2)

    H = np.array([1,0,0,0,0,1,0,0])
    H = H.reshape(2,4)

    #rospy.sleep(2)
    #global std_acc
    #std_acc = imu.linear_acceleration_covariance   #ivmenin saçılması  sabit //sifir olarak çıktı aldıkdık
    #print(std_acc)
    Q = np.array([[(dt**4)/4, 0,        (dt**3)/2, 0],
                  [0,        (dt**4)/4, 0,         (dt**3)/2],
                  [(dt**3)/2, 0,         dt**2,     0],
                  [0,        (dt**3)/2, 0,           dt**2]]) #* std_acc**2    #np.eye(4)*0.1

    R = np.eye(2) * 0.5

    ############################################################################

    rospy.sleep(1)  # gps verileri gelmesi için biraz bekle
    gpsX , gpsY = get_local_coord(gps.latitude,gps.longitude)
        # x = gps.latitude
        # y = gps.longitude
        # get_local_coord(lat,lon)

    #print(std_acc)
    x_ivme = imu.linear_acceleration.x
    y_ivme = imu.linear_acceleration.y
    Kalman(x_ivme,y_ivme, F, G, H, Q  ,R, gpsX , gpsY)


    rospy.spin()
