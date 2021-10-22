classdef UpperLimb

    properties
        serialLink
        l0
        l1
        l2
        d1
    end

    methods

        function obj = UpperLimb
            obj.serialLink = obj.MakeSerialLink;
            obj.l0 = obj.serialLink.links(1).a;
            obj.l1 = obj.serialLink.links(2).a;
            obj.l2 = obj.serialLink.links(3).a;
            obj.d1 = obj.serialLink.links(2).d;
        end

        function out = GetPosition(obj, theta1, theta2, theta3)
            x = obj.d1 * sin(theta1) + obj.l1 * cos(theta1) * cos(theta2) + obj.l2 * cos(theta1) * cos(theta2 + theta3);
            y = -obj.d1 * cos(theta1) + obj.l1 * sin(theta1) * cos(theta2) + obj.l2 * sin(theta1) * cos(theta2 + theta3);
            z = obj.l1 * sin(theta2) + obj.l2 * sin(theta2 + theta3);
            out = [x; y; z];
        end

        function out = ForwardKinematics(obj, theta1, theta2, theta3)
            t01 = [cos(theta1) 0 sin(theta1) 0; sin(theta1) 0 -cos(theta1) 0; 0 1 0 0; 0 0 0 1];
            t12 = [cos(theta2) -sin(theta2) 0 obj.l1 * cos(theta2); sin(theta2) cos(theta2) 0 obj.l1 * sin(theta2); 0 0 1 obj.d1; 0 0 0 1];
            t23 = [cos(theta3) -sin(theta3) 0 obj.l2 * cos(theta3); sin(theta3) cos(theta3) 0 obj.l2 * sin(theta3); 0 0 1 0; 0 0 0 1];
            out = t01 * t12 * t23;
        end

        function out = ForwardKinematics2R(obj, theta2, theta3)
            t12 = [cos(theta2) -sin(theta2) 0 obj.l1 * cos(theta2); sin(theta2) cos(theta2) 0 obj.l1 * sin(theta2); 0 0 1 obj.d1; 0 0 0 1];
            t23 = [cos(theta3) -sin(theta3) 0 obj.l2 * cos(theta3); sin(theta3) cos(theta3) 0 obj.l2 * sin(theta3); 0 0 1 0; 0 0 0 1];
            out = t12 * t23;
        end

        function out = InverseKinematics(obj, x, y, z)
            % Source: P.27~35 https://slideplayer.com/slide/13107056/

            r2 = sqrt(x^2 + y^2 + (z - (0))^2);
            c3 = (r2^2 - (obj.l1^2 + obj.l2^2)) / (2 * obj.l1 * obj.l2);
            cb = (r2^2 + obj.l1^2 - obj.l2^2) / (2 * r2 * obj.l1);

            theta1 = atan2(y, x);

            theta2a = atan2(z - (0), sqrt(x^2 + y^2)) + atan2(sqrt(1 - cb^2), cb);
            theta2b = atan2(z - (0), sqrt(x^2 + y^2)) + atan2(-sqrt(1 - cb^2), cb);

            theta3a = atan2(-sqrt(1 - c3^2), c3);
            theta3b = atan2(sqrt(1 - c3^2), c3);

            out = [theta1, theta2a, theta3a; theta1, theta2b, theta3b];
        end

        function out = InverseKinematics2(obj, x, y, z)
            % Wrong
            % Source: http://engineeronadisk.com/notes_software/aia5.html

            a = sqrt(x^2 + y^2);
            b = (a^2 + obj.l1^2 - obj.l2^2 + z^2) / (2 * obj.l1);

            theta1 = atan(y / x);

            theta2a = atan(b / sqrt(x^2 + a^2 - b^2)) - atan(a / z);
            theta3a = atan((z - obj.l1 * sin(theta2a)) / (a - obj.l1 * cos(theta2a))) - theta2a;

            theta2b = atan(b / -sqrt(x^2 + a^2 - b^2)) - atan(a / z);
            theta3b = atan((z - obj.l1 * sin(theta2b)) / (a - obj.l1 * cos(theta2b))) - theta2b;

            out = [theta1, theta2a, theta3a; theta1, theta2b, theta3b];
        end

        function out = InverseKinematics2R(obj, x, z)
            theta21 = 2 * atan2(sqrt((obj.l1 + obj.l2)^2 - (x^2 + z^2)), sqrt((x^2 + z^2) - (obj.l1 - obj.l2)^2));
            theta22 =- theta21;

            theta11 = atan2(z, x) + atan2(obj.l2 * sin(theta21), obj.l1 + obj.l2 * cos(theta21));
            theta12 = atan2(z, x) + atan2(obj.l2 * sin(theta22), obj.l1 + obj.l2 * cos(theta22));

            out = [theta11, -theta21; theta12, -theta22];
        end

    end

    methods (Access = private)

        function out = MakeSerialLink(~)
            L(1) = Link([0, 0, 0, pi / 2]);
            %L(2)=Link([0,0.2,0.5,0]);
            L(2) = Link([0, 0, 0.5, 0]);
            L(3) = Link([0, 0, 0.5, 0]);
            out = SerialLink(L, 'name', 'Upper-Limb');
        end

    end

    methods (Static)
    end

end
