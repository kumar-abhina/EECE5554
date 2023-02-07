import rospy
import bagpy
from bagpy import bagreader
import pandas as pd
import seaborn as sea
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
import numpy as np
import sys
from scipy import stats

colors = ['green', 'blue', 'lime']

if __name__=='__main__':
    
    
    if len(sys.argv)>1:
        b = None
        if sys.argv[1]=='1':
            b = bagreader('/data/walking_data.bag')
            
            gps_msg = b.message_by_topic('/gps')
            
            my_gps_data = pd.read_csv(gps_msg)
        
            my_gps_data_utm_e = my_gps_data.utm_easting.to_numpy()
            my_gps_data_utm_n = my_gps_data.utm_northing.to_numpy()
            my_gps_data_utm_t = my_gps_data['gps_header.stamp.secs'].to_numpy()
            
            for every_element in range (0,len(my_gps_data_utm_e)):
                my_gps_data_utm_e[every_element] = my_gps_data_utm_e[every_element]%10000
        
            for every_element in range (0,len(my_gps_data_utm_n)):
                my_gps_data_utm_n[every_element] = my_gps_data_utm_n[every_element]%10000
                
        

            ## plot data
            
            fig,ax = bagpy.create_fig(1)
            ax[0].scatter(x='utm_easting',y = 'utm_northing',data = my_gps_data,s=.25,c = 'blue',label = 'UTM Coordinates')
            ax[0].set_xlabel('UTM_Easting')
            ax[0].set_ylabel('UTM_Northing')
            plt.axvline(x=6800, c="green", label="ORIGIN : (326800 m,4686500 m)")
            plt.axhline(y=6500, c="green",)
            plt.legend(loc = 'lower right', title = 'WALKING DATA PLOT',fontsize = 'large')
            plt.show()
            
            plt.xlabel('Histogram')
            plt.ylabel('UTM_Easting')
            plt.hist(my_gps_data_utm_e, 100)
            plt.title('UTM_Easting Histogram')
            plt.show()
            
            plt.xlabel('Histogram')
            plt.ylabel('UTM_Northing')
            plt.hist(my_gps_data_utm_n, 100)
            plt.title('UTM_Northing Histogram')
            plt.show()
            
    else:
        b = bagreader('/data/stationary_data_afternoon.bag')
        gps_msg = b.message_by_topic('/gps')
        my_gps_data = pd.read_csv(gps_msg)
  
        my_gps_data_utm_e = my_gps_data.utm_easting.to_numpy()
        my_gps_data_utm_n = my_gps_data.utm_northing.to_numpy()
        my_gps_data_utm_t = my_gps_data['gps_header.stamp.secs'].to_numpy()
        
        b2 = bagreader('/data/stationary_data_afternoon1.bag')
        gps_msg_2 = b2.message_by_topic('/gps')
        my_gps_data_2 = pd.read_csv(gps_msg_2)
    
        my_gps_data_utm_e_2 = my_gps_data_2.UTM_easting.to_numpy()
        my_gps_data_utm_n_2 = my_gps_data_2.UTM_northing.to_numpy()
        my_gps_data_utm_t_2 = my_gps_data_2['Header.stamp.secs'].to_numpy()
        
        b3 = bagreader('/data/stationary_data_afternoon_2.bag')
        gps_msg_3 = b3.message_by_topic('/gps')
        my_gps_data_3 = pd.read_csv(gps_msg_3)
    
        my_gps_data_utm_e_3 = my_gps_data_3.UTM_easting.to_numpy()
        my_gps_data_utm_n_3 = my_gps_data_3.UTM_northing.to_numpy()
        my_gps_data_utm_t_3 = my_gps_data_3['Header.stamp.secs'].to_numpy()
        
        
        for every_element in range (0,len(my_gps_data_utm_e_2)):
            my_gps_data_utm_e_2[every_element] = my_gps_data_utm_e_2[every_element]%100
        
        for every_element in range (0,len(my_gps_data_utm_n_2)):
            my_gps_data_utm_n_2[every_element] = my_gps_data_utm_n_2[every_element]%100
            
        for every_element in range (0,len(my_gps_data_utm_e)):
            my_gps_data_utm_e[every_element] = my_gps_data_utm_e[every_element]%100
        
        for every_element in range (0,len(my_gps_data_utm_n)):
            my_gps_data_utm_n[every_element] = my_gps_data_utm_n[every_element]%100
        
        for every_element in range (0,len(my_gps_data_utm_e_3)):
            my_gps_data_utm_e_3[every_element] = my_gps_data_utm_e_3[every_element]%100
        
        for every_element in range (0,len(my_gps_data_utm_n_3)):
            my_gps_data_utm_n_3[every_element] = my_gps_data_utm_n_3[every_element]%100
        
     
        
        fig,ax = bagpy.create_fig(1)
        ax[0].scatter(x='utm_easting',y = 'utm_northing',data = my_gps_data,s=25,c = 'blue',label = 'UTM Readings taken at 9.25PM')
        ax[0].scatter(x='UTM_easting',y = 'UTM_northing',data = my_gps_data_2,s=25,c = 'red',label = 'UTM Readings taken at 2PM')
        ax[0].scatter(x='UTM_easting',y = 'UTM_northing',data = my_gps_data_3,s=25,c = 'orange',label = 'UTM Readings taken at 4:20PM')
        ax[0].set_xlabel('UTM_Easting')
        ax[0].set_ylabel('UTM_Northing')
        plt.axvline(x=49, c="green", label="ORIGIN : (327149 m,4686662 m)")
        plt.axhline(y=62, c="green",)
        plt.legend(loc = 'lower right', title = 'STATIONARY DATA PLOT',fontsize = 'large')
        plt.show()
        fig = plt.figure()
        ax = plt.axes(projection='3d')
        ax.scatter3D(my_gps_data_utm_e,my_gps_data_utm_n,my_gps_data_utm_t)
        