# VVCM_ForwardKinematics

This MatlabApp is designed by MATLAB 2022b. It is the open sorce code of the paper _"Forward Kinematics of Multi-Robot System with a Deformable Sheet"_.

It contains 5 MATLAB files. The length unit in these function is millimeter (mm). They are:
1. VVCM.mlapp      		MATLAB Apps for user interface creation.
2. FormationFeasible.m   	Used to judge whether the robot formation is feasible.
3. VVCM_CQP.m		Used to calculate the Forward Kinematics when taut cable group is known.
4. VVCM_FK.m    		Used to find all possible Forward Kinematics Solutions.
5. PlotRobot.m		Used to plot all the Forword Kinematics solutions.


User Instructions: The users should input robot number $N$,  the height of the holding points $z_r$,  vertices of the sheet $\mathcal{V}_N^0$ and positions of the robot team $\mathcal{R}_N$.
The app will output the all the object position  $\mathbb{P}_o = \[ ^{(1)}{\rm{p}}_o, ..., ^{(M)}{\rm{p}}_o \]$, Contact Point Position  $\mathbb{V}_o = \[^{(1)}\rm{v}_o, ..., ^{(M)}\rm{v}_o\]$., and the Taut Cable Group $\mathbb{I}_k = \{^{(1)}{{I}}_k, ..., ^{(M)}{{I}}_k\}$. Then, plot them in Figure.
This is the screenshot of the [user interface](VVCM_FK_APP.png)
1. Run VVCM.mlapp  in MATLAB APP Designer or you can install the file VVCM_ForwardKinematics.mlappinstall and run it.
2. Set the robot number ($N$) in the box;
3. Set the height of the holding points ($z_r$) in the box;
4. Click the yellow button (Sheet Vertices Selection) to select the vertices of the sheet ($\mathcal{V}_N^0$) by clicking the mouse, or you can set it by text input.
For example, after set the robot number $N=8$, you can copy the following data in the box
   233.871,190.9621;581.7972,126.8222;890.553,494.1691;413.5945,820.6997;
   
The meaning of the above array is $\mathcal{V}_N^0= [x_{v1},y_{v1};...;x_{v8},y_{v8};]$ . Please follow the above format. The app will read the data you input, and plot it in the app.

6.  Click the yellow button (Robot Vertices Selection) to select the positions of the robot team ($\mathcal{R}_N$), or you can set it by text input.
For example, after step 4, you can copy the following data in the box
385.9447,310.4956;565.6682,266.7638;683.1797,453.3528;478.1106,555.3936;
    
The meaning of the above array is $\mathcal{R}_N = [x_1,y_1;...;x_8,y_8;]$. Please follow the above format. The app will read the data you input, and plot it in the app.

8. Click the yellow button (Forward Kinematics Calculation) and run the algorithm.

The Output variables will display in the right side of the app.

Users can click the yellow button (Reset) to try again.
