classdef robot_planar_2R
    properties
        l1
        l2
        
        q1
        q2
        
        p
        
        dhtable
        T
        A0_1
        A1_2
    end
    
    methods
        function self = robot_planar_2R()
            self.l1 = sym('l1', 'real');
            self.l2 = sym('l2', 'real');

            self.q1 = sym('q1', 'real');
            self.q2 = sym('q2', 'real');   
            
            self.p = [self.l1*cos(self.q1)+self.l2*cos(self.q1+self.q2);
                        self.l1*sin(self.q1)+self.l2*sin(self.q1+self.q2);
                        0];
                    
            alpha = [0 0];
            a=[self.l1 self.l2];
            d=[0 0];
            theta=[self.q1 self.q2];

            self.dhtable=[alpha',a',d',theta'];
            
            [self.T, A] = DHMatrix(self.dhtable);
            self.A0_1=A{1};
            self.A1_2=A{2};
        end
        
        function p = direct(self, l1, l2, q1, q2)
            p = subs(self.p, {self.l1, self.l2, self.q1, self.q2}, {l1, l2, q1, q2});
        end
        
        function [q1, q2] = inverse(self, l1, l2, p, sign_sin_pos_or_neg)
            % p = [px_d py_d pz_d]'
            
    
            if sign_sin_pos_or_neg == "neg"
                sign = -1;
            else
                sign = 1;
            end
            
            px = p(1);
            py = p(2);
     
            cos_2 = (px^2 + py^2 -l1^2 - l2^2)/(2*l1*l2);
%             if cos_2 > 1 || cos_2 < -1
%                 q1 = -1000;
%                 q2 = -1000;
%                 fprintf("no solutions")
%                 return 
%             end
            
            sin_2 = sign*sqrt(1-cos_2^2);
            q2 = atan2(sin_2, cos_2);
            
            det_1 = l1^2 + l2^2 + 2*l1*l2*cos(q2);
                    
            if det_1 ~= 0
                sin_1 = (py*(l1+l2*cos(q2))-px*l2*sin(q2))/det_1;
                cos_1 = (px*(l1+l2*cos(q2))+py*l2*sin(q2))/det_1;
                q1 = atan2(sin_1, cos_1);
            else
                fprintf("INFO: det_1 == 0, so alternative method will be used")
                q1 = atan2(py, px) - atan2(l2*sin_2, l1+l2*cos_2);
                q1 = wrapToPi(q1); %express q1 in [-pi, pi]
            end
        end
    end
end

