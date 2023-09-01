function [M, Po,Vo,ID,Tn,ispossible] = VVCM_FK(zr,Vn,Rn)
% This function is to find all possible Forward Kinematics Solutions. See
% algorithm 1 in our paper
%{
    The length unit in this function is millimeter (mm).
    Input Variables: 

    z_r: the height of the holding points (unit: mm).
    Vn (N x 2): the shape of the initial sheet. Vn(i,:) = [x_v,y_v];
    Rn (N x 2): the positions of the N-robot team. Rn(i,:) = [x,y];
   
    Output Variables:
    M: the number of the FK solutions
    Po (M x 3): all the object position.        Po(i,:) = [x_o, y_o, z_o];
    Vo (M x 2): all the contact point position.     Vo(i,:) = [x_vo, y_vo];

    ID (M x N): all the cable state. if ID(i,:) =
    [1,2,...,N], it means all the cables are taut. If ID(i,j)=0, it means in the i^th FK solution, the j^th cable i is slack.

    Tn: all the number of the taut cables Tn = {taut_number_1, ...,taut_number_M}

    ispossible: 0: there is not solution in this case; 1: there is at least
    one solution in this case
%}

   
    N  = length(Vn(:,1));%robot number

    M = 0; %initial the number of the FK solutions 
    
    if FormationFeasible(Vn,Rn)
        for k = 3:N %the taut calbe number in each case

            taut_num = nchoosek(N,k);%the combination number of the taut cable group with  cables taut
            taut_vec = nchoosek(1:1:N,k);%all possible taut cable group with k cables taut

            for j = 1: taut_num
                %slove the Forward Kinematics with the taut cable group taut_vec
                 [Vo_temp,Po_temp,ID_temp,Taut_num_temp,ispossible] = VVCM_CQP(zr,Vn,Rn,taut_vec(j,:));
                 if ispossible ==1
                   M = M + 1;
                   Po(M,:)     = Po_temp;
                   Vo(M,:)     = Vo_temp;
                   ID(M,:)     = ID_temp;
                   Tn(M) = Taut_num_temp;                           
                 end
            end 
        end
        
        if M>=1
             ispossible = 1;
        else
            %if M=0, there is no solution
             ispossible = 0;
             Po = [0,0,0];
             Vo = [0,0];
             ID = zeros(N,0);
             Tn = 0;
             M = 0;    
        end
         
    else
        %if the formation is infeasible, there is no solution
         ispossible = 0;
         Po = [0,0,0];
         Vo = [0,0];
         ID = zeros(N,0);
         Tn = 0;
         M = 0;      
    end
end

