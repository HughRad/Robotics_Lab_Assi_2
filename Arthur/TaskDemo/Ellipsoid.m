classdef Ellipsoid < handle
    properties
        UR3robot
        CR5robot
        tranCR5
        tranUR3
        URmeshSize
        URmeshOffset
        CR5meshSize
        CR5meshOffset
        CR5Links
        UR3Links
        tapvert
    end

    methods
        function obj = Ellipsoid(UR3,CR5,tapvert)
            obj.UR3robot = UR3;
            obj.CR5robot = CR5;

            obj.tranUR3 = zeros(4, 4, obj.UR3robot.n + 1);
            obj.tranCR5 = zeros(4, 4, obj.CR5robot.n + 1);

            obj.tranUR3(:,:,1) = obj.UR3robot.base;
            obj.tranCR5(:,:,1) = obj.CR5robot.base;

            obj.CR5meshSize = [0.07,0.23,0.09;0.07,0.2,0.07;...
                        0.07,0.14,0.07;0.07,0.14,0.07;0.07,0.07,0.07];

            obj.CR5meshOffset = [0,0,0.09;0,0,-0.07;...
                        0,0,-0.03;0,0,0;0,0,0];
            
            obj.URmeshSize = [0.2, 0.06, 0.09; 0.18, 0.06, 0.06;...
                0.06, 0.06, 0.06; 0.06, 0.1, 0.06; 0.06, 0.06, 0.06];

            obj.URmeshOffset = [0, 0, 0.12; 0, 0, 0; 0, 0, 0; 0, 0, 0; 0, 0, 0];

            obj.CR5Links = CR5.links;
            obj.UR3Links = UR3.links;

            obj.tapvert = tapvert;
        end

        function collision = URmesh(obj, q)
           for i = 1:obj.UR3robot.n
                obj.tranUR3(:,:,i+1) = obj.tranUR3(:,:,i)*trotz(q(i))*transl(obj.UR3Links(i).a,0,obj.UR3Links(i).d)*trotx(obj.UR3Links(i).alpha);
           end

           tranCenter = obj.tranUR3;

           for i = 1:5
                % Create and display ellipsoids for each link
                tranCenter(1:3,4,i+2) = obj.tranUR3(1:3,4,i+1)+((obj.tranUR3(1:3,4,i+2) - obj.tranUR3(1:3,4,i+1))/2);
                [x, y, z] = ellipsoid(obj.URmeshOffset(i,1), obj.URmeshOffset(i,2), obj.URmeshOffset(i,3), obj.URmeshSize(i,1), obj.URmeshSize(i,2), obj.URmeshSize(i,3),5);
                elippoints = [x(:), y(:), z(:)]';
                TranElip = tranCenter(:,:,i+2) * [elippoints; ones(1, size(elippoints,2))];
                TranElip = TranElip(1:3,:);
                % x1 = reshape(TranElip(1,:), size(x));
                % y1 = reshape(TranElip(2,:), size(y));
                % z1 = reshape(TranElip(3,:), size(z));
                % surf(x1, y1, z1);

                MeshTriang = delaunayTriangulation(unique(TranElip', 'rows'));
                
                for p = 1:1:size(obj.tapvert,1)
                    ID = pointLocation(MeshTriang,obj.tapvert(p,:));
                    if ~isnan(ID)
                        disp('Warning, An appraching collision is about to occure, pausing program...')
                        collision = true;
                        return;
                    end
                end
           end
           collision = false;
           return;
        end

        function collision = CR5mesh(obj, q)
            for i = 1:obj.CR5robot.n
                if i == 2 || i == 3
                    obj.tranCR5(:,:,i+1) = obj.tranCR5(:,:,i)*trotz(q(i))*transl(0,abs(obj.CR5Links(i).a),obj.CR5Links(i).d)*trotx(obj.CR5Links(i).alpha); 
                elseif i == 1 || i == 4 || i == 6
                    obj.tranCR5(:,:,i+1) = obj.tranCR5(:,:,i)*trotz(q(i))*transl(obj.CR5Links(i).a,0,obj.CR5Links(i).d)*trotx(obj.CR5Links(i).alpha); 
                else
                    obj.tranCR5(:,:,i+1) = obj.tranCR5(:,:,i)*trotz(q(i))*transl(-obj.CR5Links(i).a,0,-obj.CR5Links(i).d)*trotx(obj.CR5Links(i).alpha); 
                end
            end
        
            tranCenter = obj.tranCR5;

            for i = 1:5
                tranCenter(1:3,4,i+2) = obj.tranCR5(1:3,4,i+1)+((obj.tranCR5(1:3,4,i+2) - obj.tranCR5(1:3,4,i+1))/2);
                [x, y, z] = ellipsoid(obj.CR5meshOffset(i,1), obj.CR5meshOffset(i,2), obj.CR5meshOffset(i,3), obj.CR5meshSize(i,1), obj.CR5meshSize(i,2), obj.CR5meshSize(i,3),5);
                elippoints = [x(:), y(:), z(:)]';
                TranElip = tranCenter(:,:,i+2) * [elippoints; ones(1, size(elippoints,2))];
                TranElip = TranElip(1:3,:);
                % x1 = reshape(TranElip(1,:), size(x));
                % y1 = reshape(TranElip(2,:), size(y));
                % z1 = reshape(TranElip(3,:), size(z));
                % surf(x1, y1, z1);

                MeshTriang = delaunayTriangulation(unique(TranElip', 'rows'));

                for p = 1:1:size(obj.tapvert,1)
                    ID = pointLocation(MeshTriang,obj.tapvert(p,:));
                    if ~isnan(ID)
                        disp('Warning, An appraching collision is about to occure, pausing program...')
                        collision = true;
                        return;
                    end
                end
            end
            collision = false;
            return;
        end
    end
end