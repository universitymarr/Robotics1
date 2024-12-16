classdef robot_planar_3R
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
        
        phi
        
    end
    
    methods
        function self = robot_planar_3R()
            self.l1 = sym('l1', 'real');
            self.l2 = sym('l2', 'real');
            self.l3 = sym('l3', 'real');

            self.q1 = sym('q1', 'real');
            self.q2 = sym('q2', 'real');   
            self.q3 = sym('q3', 'real');  
   
            alpha = [0 0 0];
            a=[self.l1 self.l2 self.l3];
            d=[0 0 0];
            theta=[self.q1 self.q2 self.q3];

            self.dhtable=[alpha',a',d',theta'];
            
            [self.T, A] = DHMatrix(self.dhtable);
            self.A0_1=A{1};
            self.A1_2=A{2};
            self.A2_3=A{3};
           
            f_r_4D = get_f_r(self.T); %(X Y Z PHI)

            self.p = [f_r_4D(1); f_r_4D(2); f_r_4D(3)];
            self.phi = f_r_4D(4);
        end
        
        function p = direct(self, l1, l2, l3, q1, q2, q3)
            p = subs(self.p, {self.l1, self.l2, self.l3, self.q1, self.q2, self.q3}, {l1, l2, l3, q1, q2, q3});
        end
        
        function [q1, q2, q3] = inverse(self, l1, l2, l3, p, phi, sign_sin_pos_or_neg)
            % p = [px_d py_d pz_d]'
            % phi = angle end effector wrt robot's base (phi = q1+q2+q3)
            
            % To get inverse kinematic we decompose computation in two
            % parts as described in sept 2020 exam ex 3

            pt2 = p - [l3*cos(phi) l3*sin(phi) 0]'; %pos third link's base
     
            robot_2r = robot_planar_2R();
            [q1, q2] = robot_2r.inverse(l1, l2, pt2, sign_sin_pos_or_neg);
            
            q3 = phi - q1 - q2;
        end
    end
end