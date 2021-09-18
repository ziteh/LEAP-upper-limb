classdef UpperLimb
    properties
        serialLink
        l1
        l2
        l3
        d2
    end
    
    methods
        function obj = UpperLimb
            obj.serialLink = obj.MakeSerialLink;
            obj.l1 = obj.serialLink.links(1).a;
            obj.l2 = obj.serialLink.links(2).a;
            obj.l3 = obj.serialLink.links(3).a;
            obj.d2 = obj.serialLink.links(2).d;
        end
        
        function out = ForwardKinematics(obj, theta1, theta2, theta3)
            t01 = [cos(theta1) 0 sin(theta1) 0;sin(theta1) 0 -cos(theta1) 0;0 1 0 0;0 0 0 1];
            t12 = [cos(theta2) -sin(theta2) 0 obj.l2*cos(theta2);sin(theta2) cos(theta2) 0 obj.l2*sin(theta2);0 0 1 obj.d2;0 0 0 1];
            t23 = [cos(theta3) -sin(theta3) 0 obj.l3*cos(theta3);sin(theta3) cos(theta3) 0 obj.l3*sin(theta3);0 0 1 0;0 0 0 1];
            out = t01 * t12 * t23;
        end
        
        function out = InverseKinematics(obj, x, y ,z)
        end

    end

    methods(Access = private)
        function out = MakeSerialLink(~)
            L(1)=Link([0,0,0,pi/2]);
            L(2)=Link([0,0.2,0.5,0]);
            L(3)=Link([0,0,0.5,0]);
            out = SerialLink(L,'name','Upper-Limb');
        end
    end
    
    methods(Static)
    end
end

