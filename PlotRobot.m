function p = PlotRobot(z_r, ID, Rn, Po)
%This function is to plot all the Forward Kinematics solutions. The length unit in this function is millimeter (mm).
%{  
    For an N-robot team, which has M Forword Kinematics Solutions. The
    Input Variables are:
    z_r: the height of the holding points (unit: mm).

    ID (M x N): the state of the cables. For the i^th FK solution, if ID(i,:)=
    [1,2,...,N], it means all the cables are taut. If ID(i,j)=0, it means
    in the i^th FK solution, the cable j is slack.

    Rn (N x 2): the positions of the N-robot team. Rn(i,:) = [x,y]

    Po (M x 3): all the object position in FK solutions. Po(i,:) = [x_o, y_o, z_o]
    
%}    
    N = length(Rn(:,1)); %the number of the robots
    M = length(Po(:,1)); %the number of all the FK solutions
     
    for k=1:M
        
        figure(k);   %plot each solution in a figure
        
        xlabel("x(mm)");ylabel("y(mm)");zlabel("z(mm)");
        
        % user-modifiable
        view([45 45]); %Set view angle. 
        axis([0 1000 0 1000 0 1000]);hold on; %Set drawing range. 

         
        % Draw the robot position and outline of robot formation (color: gray)
        for i= 1: N-1
            line([Rn(i,1) Rn(i+1,1)],[Rn(i,2) Rn(i+1,2)],'Marker', 'o', 'MarkerEdgeColor', [0.5,0.5,0.5],'LineStyle',':','color',[0.5,0.5,0.5]);hold on;
        end
            line([Rn(N,1) Rn(1,1)],[Rn(N,2) Rn(1,2)],'Marker', 'o', 'MarkerEdgeColor', [0.5,0.5,0.5],'LineStyle',':','color',[0.5,0.5,0.5]);hold on;


        % Draw the holding points (color: gray)
        for i= 1: N   
            plot3(Rn(i,1),Rn(i,2),z_r,'Marker', 'o', 'MarkerEdgeColor', [0.5,0.5,0.5]);hold on;
        end

        

        % Draw the support rods (color: gray; line: dotted line)
        for i= 1: N   
             plot3([Rn(i,1) Rn(i,1)],[Rn(i,2) Rn(i,2)],[z_r 0],'LineStyle',':','color',[0.5,0.5,0.5]);hold on;  
        end

        % Draw the taut cable, its corresponding robot and holding point 
        % (color: blue, line: solid line)
        j=0;
        for i = 1:N
            if ID(k,i) == 0 
            else
                j = j+1;
                ID_taut(j) = i;
                plot3([Rn(ID(k,i),1) Po(k,1)],[Rn(ID(k,i),2) Po(k,2)],[z_r Po(k,3)],'b-','LineWidth',1);hold on;  
                plot3(Rn(ID(k,i),1),Rn(ID(k,i),2),z_r,'Marker', 'o', 'MarkerEdgeColor', 'blue');hold on;
                plot3(Rn(ID(k,i),1),Rn(ID(k,i),2),0,'Marker', 'o', 'MarkerEdgeColor', 'blue');hold on;
            end
        end  

        % Draw the outline of the robot formation whose corresponding cable is taut 
        % (color: blue, line: solid line)
        for i= 1: j-1
            line([Rn(ID_taut(i),1) Rn(ID_taut(i+1),1)],[Rn(ID_taut(i),2) Rn(ID_taut(i+1),2)],'Marker', 'o', 'MarkerEdgeColor', 'blue','LineStyle','-.','color','blue');hold on;
        end
            line([Rn(ID_taut(j),1) Rn(ID_taut(1),1)],[Rn(ID_taut(j),2) Rn(ID_taut(1),2)],'Marker', 'o', 'MarkerEdgeColor', 'blue','LineStyle','-.','color','blue');hold on;


        %Draw the object position (color: red)
        plot3(Po(k,1),Po(k,2),Po(k,3),'marker', 'o', 'markerfacecolor','red','markeredgecolor', 'blue'); hold on;
        
        %Draw the projection point of the object (color: red)
        plot3(Po(k,1),Po(k,2),0,'ro');hold on; 

        %Draw projection line (color: black, line: dotted line)
        plot3([Po(k,1) Po(k,1)],[Po(k,2) Po(k,2)],[Po(k,3) 0],'LineStyle','-.','color',[0.5,0.5,0.5]);hold on; 
    end
    
        %Plot success
        p=1;

end

