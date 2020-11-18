#!/usr/bin/python3.5
# -*-coding:utf_8 -*

""" Little description from 'function.py' """

# Import library
# from function import *

import time
from math import *
from pylab import *
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as animation

# Fonctions
# Fonctions
def OpenFig():
    fig=plt.figure();
    ax=fig.add_subplot(111, projection='3d');
    return ax
def Display_body_segments_and_joints(ax, clear, pause, name_file) :
    x_=[]; y_=[]; z_=[];
    file=open(name_file, "r");
    for line in file:
        LineRead=line.rstrip('\n\r').split(";; ");
        x_.append(float(LineRead[0])); y_.append(float(LineRead[1])); z_.append(float(LineRead[2]));
    file.close();

    x_body=[]; y_body=[]; z_body=[];
    for element in x_: x_body.append(element);
    for element in y_: y_body.append(element);
    for element in z_: z_body.append(element);

    plt.ion();
    if clear==True: ax.clear();
    ax.scatter3D(asarray(x_body), asarray(y_body), asarray(z_body), c="k", marker="o", alpha=1.0);

    x_body_r=np.array([[x_body[0]], [x_body[1]], [x_body[2]], [x_body[3]], [x_body[4]]]);
    y_body_r=np.array([[y_body[0]], [y_body[1]], [y_body[2]], [y_body[3]], [y_body[4]]]);
    z_body_r=np.array([[z_body[0]], [z_body[1]], [z_body[2]], [z_body[3]], [z_body[4]]]);
    ax.plot_wireframe(x_body_r, y_body_r, z_body_r, color='black');
    x_body_l=np.array([[x_body[0]], [x_body[5]], [x_body[6]], [x_body[7]], [x_body[8]]]);
    y_body_l=np.array([[y_body[0]], [y_body[5]], [y_body[6]], [y_body[7]], [y_body[8]]]);
    z_body_l=np.array([[z_body[0]], [z_body[5]], [z_body[6]], [z_body[7]], [z_body[8]]]);
    ax.plot_wireframe(x_body_l, y_body_l, z_body_l, color='black');

    ax.set_xlabel('x (m)'); ax.set_ylabel('y (m)'); ax.set_zlabel('z (m)');
    # ax.set_xlim(-1, 1); ax.set_ylim(-1, 1); ax.set_zlim(-1, 1);
    ax.set_xlim(-0.55, 0.55); ax.set_ylim(-0.55, 0.55); ax.set_zlim(-0.7, 0.4);

    plt.show(); plt.pause(pause); plt.ioff();
    pass
def Display_space_points(ax, x, y, z, clear, pause, r_color, g_color, b_color, marker, alpha_value) :
    plt.ion();
    if clear==True: ax.clear();

    i=0;
    while i<len(x):
        C = np.array([b[i], g[i], r[i]]);
        ax.scatter(asarray(x[i]), asarray(y[i]), asarray(z[i]), c=C/255.0, marker=marker, alpha=alpha_value);
        i+=1;


    ax.set_xlabel('x (m)'); ax.set_ylabel('y (m)'); ax.set_zlabel('z (m)');
    # ax.set_xlim(-0.55, 0.55); ax.set_ylim(-0.55, 0.55); ax.set_zlim(-0.7, 0.4);
    # ax.set_xlim(-1, 1); ax.set_ylim(-1, 1); ax.set_zlim(-1, 1);
    ax.set_xlim(-0.4, 0.4); ax.set_ylim(-0.3, 0.5); ax.set_zlim(0, 1.4);

    plt.show(); plt.pause(pause); plt.ioff();
    pass
def calcul_elapsed_time(start_time, msg):
    end_time=time.time()
    elapsed_time=int(end_time-start_time);
    elapsed_time_h=int(elapsed_time/3600);
    elapsed_time_m=int(elapsed_time/60)-(elapsed_time_h*60);
    elapsed_time_s=elapsed_time-(elapsed_time_m*60)-(elapsed_time_h*3600);

    if (elapsed_time_h!=0) :
        print(  "\x1B[1m"+"\x1B[1m"+" "+msg+" -> "+str(elapsed_time_h)+"h "+str(elapsed_time_m)+"min "+str(elapsed_time_s)+"s\x1B[0m\n");
    elif ( (elapsed_time_m!=0) & (elapsed_time_h==0)) :
        print(  "\x1B[1m"+"\x1B[1m"+" "+msg+" -> "+str(elapsed_time_m)+"min "+str(elapsed_time_s)+"s\x1B[0m\n");
    elif ( (elapsed_time_s!=0) & (elapsed_time_m==0) & (elapsed_time_h==0)) :
        print(  "\x1B[1m"+"\x1B[1m"+" "+msg+" -> "+str(elapsed_time_s)+"s\x1B[0m\n");
    return elapsed_time # in second

def ReadFile(namefile_database):
    file=open(namefile_database, "r"); x=[]; y=[]; z=[]; r=[]; g=[]; b=[];
    for line in file:
        l=line.rstrip('\n\r').split(" ");
        x.append(float(l[0])); y.append(float(l[1])); z.append(float(l[2]));
        r.append(float(l[3])); g.append(float(l[4])); b.append(float(l[5]));
    file.close();
    return x, y, z, r, g, b;


print("\n\x1B[1m"+"Lancement de l'application : 'display_pts.py'"+"\x1B[0m");

_start_test=time.time(); #*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*
namefile_database = "/tmp/pts.txt"; x=[]; y=[]; z=[]; r=[]; g=[]; b=[];
x, y, z, r, g, b = ReadFile(namefile_database)
calcul_elapsed_time(_start_test, "Récupérer les données"); #*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#

# Affichage option
no_break  = 0.1; yes_break = 1*60; waitbuttonpress = True;

ax = OpenFig();
Display_space_points(ax, x, y, z, False, yes_break, r, g, b, ".", 0.5);

if (waitbuttonpress) :
    plt.waitforbuttonpress();
