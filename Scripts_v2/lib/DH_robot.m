classdef DH_robot
    properties       
        p %end effector position
        %phi % orientation end effector
        dhtable
        T
        A
        
    end
    
    methods
        function self = DH_robot(alpha, a, d, theta)
            self.dhtable=[alpha,a,d,theta];
            
            [self.T, self.A] = DHMatrix(self.dhtable);

            %f_r_4D = get_f_r(self.T); %(X Y Z PHI)

            self.p = affine_get_translation(self.T); %[f_r_4D(1); f_r_4D(2); f_r_4D(3)];
            %self.phi = f_r_4D(4);
        end
    end
end