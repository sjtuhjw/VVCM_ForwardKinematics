function isfeasible = FormationFeasible(Vn,Rn)
%This function is to guarantee that the formation is smaller than the original sheet.
%{
    Input Variables:
    Vn (N x 2): the shape of the initial sheet. Vn(i,:) = [x_v,y_v];

    Rn (N x 2): the positions of the N-robot team. Rn(i,:) = [x,y]
%}


 N = length(Vn(:,1)); %the number of the robots 
 isfeasible = 1;
     for i=1:N
         for j=1:N
             if i ~=j
                 if norm(Rn(i,:) - Rn(j,:)) > norm(Vn(i,:) - Vn(j,:))
                     isfeasible = 0;
                 end
               
             end  
         end
     end
end

