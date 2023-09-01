function [Vo,Po,ID,taut_number,ispossible] = VVCM_CQP(zr,Vn,Rn,ID_taut)
%This function is calculate the Forward Kinematics when taut cable group is known.
%{   
    The length unit in this function is millimeter (mm).
    The Forward Kinematics Algorithm in the paper consists of 4 steps.
    Step 1: Selet a taut cable group ID_taut = {i_1, i_2, ..., i_k}

    Step 2: Form closure condition

    Step 3: Constrained Quadratic Problem (CQP) solving

    Step 4: Force closure condition

    Input Variables:
    z_r: the height of the holding points (unit: mm).
    Vn (N x 2): the shape of the initial sheet. Vn(i,:) = [x_v,y_v];
    Rn (N x 2): the positions of the N-robot team. Rn(i,:) = [x,y];
    ID_taut (1 x k): the taut cable group, ID_taut = {i_1, i_2, ..., i_k}.

    Output Variables:
    Vo (1 x 2): the contact point position. Vo = [x_vo, y_vo];
    Po (1 x 3): the object position.        Po = [x_o, y_o, z_o];
    ID (1 x N): the state of all the cables. if ID =
    [1,2,...,N], it means all the cables are taut. If ID(i)=0, it means the cable i is slack.
    taut_number: the number of the taut cables
    ispossible: 1: there is a solution in this case; 0: there is not
    solution in this case

%}
    
    N = length(Vn(:,1)); %the number of the robots
    
    
    ID_all = (1:N);  
    ID_slack = setdiff(ID_all,ID_taut);% the slack cable group
    slack_number = length(ID_slack);% the slack cable number


    %the number of the taut cables
    k = length(ID_taut);

    
    %Vn= [x_v,y_v]
    %Rn = [x,y]
    xv = zeros(N,1);yv = zeros(N,1);    
    x = zeros(N,1);y = zeros(N,1);
    for i=1:N
        xv(i) = Vn(i,1);
        yv(i) = Vn(i,2); 

        x(i) = Rn(i,1);
        y(i) = Rn(i,2); 
    end


    A1 = zeros(k-1,4);
    A2 = zeros(N-k,4);

    b1 = zeros(k-1,1);
    b2 = zeros(N-k,1);

    id1 = ID_taut(1);

    % the linear equation constructed by equation(5) in our paper
    for i=2:k
        id = ID_taut(i);
        A1(i-1,:) = [x(id)-x(id1), y(id)-y(id1), -xv(id)+xv(id1),-yv(id)+yv(id1)];
        b1(i-1,:) = round((xv(id1)^2+yv(id1)^2-x(id1)^2-y(id1)^2)/2 - (xv(id)^2+yv(id)^2-x(id)^2-y(id)^2)/2,2);
    end
    
    for i =1:N-k
        id = ID_slack(i);
        A2(i,:) = [x(id)-x(id1), y(id)-y(id1), -xv(id)+xv(id1),-yv(id)+yv(id1)];
        b2(i,:) = round((xv(id1)^2+yv(id1)^2-x(id1)^2-y(id1)^2)/2 - (xv(id)^2+yv(id)^2-x(id)^2-y(id)^2)/2,2);
    end

    A1_bar = [A1,b1];


   % Form Closure Judgement 
   if rank(A1) == rank(A1_bar) % equation (6) in our paper
        
       rank_OK = 1;%Form Closure feasible 
       %Find the maximum independent taut cables, equation (11) in our paper
        [~,id_A1]=rref(A1_bar');
        A11_bar = A1_bar(id_A1,:);
        A11 = A11_bar(:,1:4);
        b11 = A11_bar(:,5);
 
        %Lagrange function constructed: f=0.5*x_bar'*H*x_bar + c'*x_bar + f0
        %equation (12) in our paper
        H = diag([2,2,-2,-2]);
        c = 2*[-x(id1),-y(id1),xv(id1),yv(id1)]';
        f0 = x(id1)^2 + y(id1)^2 - xv(id1)^2 - yv(id1)^2;
    
        %Lagrange Multiplier method, equation (15) in our paper
        D = -(A11*H^(-1)*A11')^(-1);
        C = -D*A11*H^(-1);
        B = H^(-1) - H^(-1)*A11'*C;
        
        %calculate the Constrainted Quadratic Problem (CQP) 
        x_bar = -B*c + C'*b11; %equation (16) in our paper

        % Set the precision of calculation results, 0.01
        x_o  = round(x_bar(1),2);
        y_o  = round(x_bar(2),2);
        x_vo = round(x_bar(3),2);
        y_vo = round(x_bar(4),2);

        %calculate the height of the object,
        z_o = round(zr - sqrt(-(0.5*x_bar'*H*x_bar + c'*x_bar + f0)),2);%equation (18) in our paper

        %Force Closure Judgement
        x_taut = zeros(k,1);
        y_taut = zeros(k,1);
        for  i=1:k
            x_taut(i) = x(ID_taut(i));
            y_taut(i) = y(ID_taut(i));
        end

        %the projected point of the object [x_o,y_o] should be within the
        %sub-formation of the robot attached with the taut cable
        [in, on]=inpolygon(x_o,y_o,x_taut,y_taut);
        if in==1 && on==0
            force_OK = 1;%Force Closure feasible 
        else
            force_OK = 0;%Force Closure infeasible 
            ispossible = 0;% the solution is infeasible
        end

        if  slack_number == 0    
            rest_OK = 1; %no slack cables, all cables are taut
        else
            if round(A2*x_bar,2) > round(b2,2)
               rest_OK = 1; %other cables are slack, the solution of CQP is feasible
            else
                rest_OK = 0; %other cables are not slack, the solution of CQP is infeasible
                
                
                %Clear the output when there is no solution
                ispossible =0;% the solution is infeasible
                 Vo = [0,0];
                 Po = [0,0,0];
                 ID = 0*(1:N); 
            end     
        end

         if rank_OK && rest_OK && force_OK
             % form closure feasible, CQP feasible and force closure feasible
             ispossible =1;
             Vo = [x_vo,y_vo];
             Po = [x_o,y_o,z_o];

             taut_number = length(ID_taut);

             %calculate the state of all the cables. 0 represents the slack state;
             ID = 0*(1:N);
             for i=1:N
                 for j = 1:taut_number
                     if i == ID_taut(j)
                         ID(i) = i;
                         break;
                     end
                 end
             end
         else
             %Clear the output when there is no solution
             ispossible =0; % solution infeasible
             Vo = [0,0];
             Po = [0,0,0];
             ID = 0*(1:N);  
             taut_number = 0;
         end
        
   else
       
        rank_OK = 0;%form closure infeasible
        %Clear the output when there is no solution
        ispossible = 0;
        Vo = [0,0];
        Po = [0,0,0];
        ID = 0*(1:N);
        taut_number = 0;
   end


end

