classdef robot_spatial_3R
    properties
        l1
        l2
        l3
        
        q1
        q2
        q3
        
        p
        
        dhtable
        T
        A0_1
        A1_2
        A2_3
        
    end
    
    methods
        function self = robot_spatial_3R()
            self.l1 = sym('l1', 'real');
            self.l2 = sym('l2', 'real');
            self.l3 = sym('l3', 'real');

            self.q1 = sym('q1', 'real');
            self.q2 = sym('q2', 'real');   
            self.q3 = sym('q3', 'real');  
   
            alpha = [pi/2 0 0];
            a=[0 self.l2 self.l3];
            d=[self.l1 0 0];
            theta=[self.q1 self.q2 self.q3];

            self.dhtable=[alpha',a',d',theta'];
            
            [self.T, A] = DHMatrix(self.dhtable);
            self.A0_1=A{1};
            self.A1_2=A{2};
            self.A2_3=A{3};
           
            f_r_4D = get_f_r(self.T); %(X Y Z PHI)

            self.p = [f_r_4D(1); f_r_4D(2); f_r_4D(3)];
        end
        
        function p = direct(self, l1, l2, l3, q1, q2, q3)
            p = subs(self.p, {self.l1, self.l2, self.l3, self.q1, self.q2, self.q3}, {l1, l2, l3, q1, q2, q3});
        end
        
        function [q1, q2, q3] = not_working__inverse(self, l1, l2, l3, p, sign_sin_1_pos_or_neg, sign_sin_3_pos_or_neg)
            % p = [px_d py_d pz_d]'
            px = p(1);
            py = p(2);
            pz = p(3);
            
            if sign_sin_3_pos_or_neg == 'neg'
                sign_3 = -1;
            else
                sign_3 = 1;
            end
            
            cos_3 = (px*2+py^2 + (pz-l1)^2-l2^2-l3^2)/(2*l2*l3);
            if cos_3 > 1 || cos_3 < -1
                fprintf("cos_3 not belong to [-1, 1], p is out of workspace");
                q1 = -1;
                q2 = -1;
                q3 = -1;
                return;
            end
            sin_3 = sign_3*sqrt(1-cos_3^2);
            q3 = atan2(sin_3, cos_3);
            
            
            if sign_sin_1_pos_or_neg == 'neg'
                sign_1 = -1;
            else
                sign_1 = 1;
            end
            
            den = sign_1*sqrt(px^2+py^2);
            if den ~= 0
                cos_1 = px/den;
                sin_1 = py/den;
                q1 = atan2(sin_1, cos_1);
            else
                frpintf("INFO: px^2 + py^2 = 0 then q1 has infinite solutions! We use q1=0");
                q1 = 0;
            end
            
            
            A = [l2+l3*cos(q3), -l3*sin_3;
                l3*sin_3, l2+l3*cos(q3)];
            
            b = [cos_1*px+sin_1*py;
                pz-l1];

            if det(A) ~= 0 
                x =  linsolve(A,b);

                cos_2 = x(1);
                sin_2 = x(2);
                q2 = atan2(sin_2, cos_2);
            else
                frpintf("INFO: px^2 + py^2 + (pz-l1)^2 = 0 then q2 has infinite solutions! We use q2=0");
                q2 = 0;
            end
        end
    end
end
