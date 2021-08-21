classdef UpperLimb
    %Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        sl
    end
    
    methods
        function obj = UpperLimb
            %Construct an instance of this class
            %   Detailed explanation goes here
            obj.sl = obj.make;
        end
        
        function move(pos)
            obj.sl.plot(pos);
        end
        
        function out = fk(t1,t2)
            t = obj.sl.fkine([t1,t2]);
            x = t(1,4);
            y = t(2,4);
            out = [x;y];
        end
        
        function out = ik(x,y)
            s=x^2+y^2;
            l1=1;
            l2=1;

            theta12 = 2*atan(sqrt(((l1+l2)^2-s)/(s-(l1-l2)^2)));
            if(theta12 < 0)
                theta12 = -theta12;
            end
            theta11 = atan(y/x)+atan(l2 * sin(-theta12)/(l1+l2 * cos(-theta12)));
            out=[theta11,theta12];
        end
    end
    
    methods(Static)
        function outputArg = make
            L(1) = Link([0,0,1,0]); % Upper-arm
            L(2) = Link([0,0,1,0]); % Forearm
            outputArg = SerialLink(L,'name','Upper-Limb');
        end
    end
end

