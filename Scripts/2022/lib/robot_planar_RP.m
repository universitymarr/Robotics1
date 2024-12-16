classdef robot_planar_RP
    properties
        q1
        q2
        
        p
    end
    
    methods
        function self = robot_planar_RP()

            self.q1 = sym('q1', 'real');
            self.q2 = sym('q2', 'real');   
            
            self.p = [self.q2*cos(self.q1);
                       self.q2*sin(self.q1);
                       0];
        end
        
        function p = direct(self, q1, q2)
            p = subs(self.p, {self.q1, self.q2}, {q1, q2});
        end
        
        function [q1, q2] = inverse(self, p, sign_sin_pos_or_neg)
            % p = [px_d py_d pz_d]'
            
    
            if sign_sin_pos_or_neg == "neg"
                sign = -1;
            else
                sign = 1;
            end
            
            px = p(1);
            py = p(2);
     
            q2 = sign*sqrt(px^2 + py^2);
            q1 = atan2(py/q2, px/q2);
        end
    end
end