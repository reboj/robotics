%% Create the SAWYER robot as an SerialLink object based on the set up of UR10
classdef SAWYER < handle
    properties
        %> Robot model
        model;
        
        %> workspace
        workspace = [-4 4 -4 4 -4 4];   
               
        %> If we have a tool model which will replace the final links model, combined ply file of the tool model and the final link models
        toolModelFilename = []; % Available are: 'DabPrintNozzleTool.ply';        
        toolParametersFilename = []; % Available are: 'DabPrintNozzleToolParameters.mat';        
    end
    
    methods%% Class for SAWYER robot simulation
        function self = SAWYER(toolModelAndTCPFilenames)
            if 0 < nargin
                if length(toolModelAndTCPFilenames) ~= 2
                    error('Please pass a cell with two strings, toolModelFilename and toolCenterPointFilename');
                end
                self.toolModelFilename = toolModelAndTCPFilenames{1};
                self.toolParametersFilename = toolModelAndTCPFilenames{2};
            end
            
            self.GetSAWYERRobot();
            self.PlotAndColourRobot();%robot,workspace);

            drawnow            
            % camzoom(2)
            % campos([6.9744    3.5061    1.8165]);

%             camzoom(4)
%             view([122,14]);
%             camzoom(8)
%             teach(self.model);
        end

        %% GetSAWYERRobot and Define the DH parameters of the robot based on the drawing on Universal robot
        % Given a name (optional), create and return a SAWYER robot model
        function GetSAWYERRobot(self)
            pause(0.001);
            name = ['SAWYER',datestr(now,'yyyymmddTHHMMSSFFF')];

            
           
           
             
%              L1 = Link('d',0.317,'a',0.081,'alpha',pi/2,'qlim',[-2*pi,2*pi], 'offset', 0);
%              L2 = Link('d',-0.1925,'a',0,'alpha',-pi/2,'qlim', [-2*pi,2*pi], 'offset', 270);
%              L3 = Link('d',-0.4,'a',0,'alpha',-pi/2,'qlim', [-2*pi,2*pi], 'offset', 0);
%              L4 = Link('d',-0.1685,'a',0,'alpha',pi/2,'qlim',[-2*pi,2*pi],'offset', 180); 
%              L5 = Link('d',0.4,'a',0,'alpha',-pi/2,'qlim',[-2*pi,2*pi], 'offset', 0);
%              L6 = Link('d',0.1363,'a',0,'alpha',pi/2,'qlim',[-2*pi,2*pi], 'offset', 180);
%              L7 = Link('d',0.13375,'a',0,'alpha',0,'qlim',[-2*pi,2*pi], 'offset', 270);

%             
L1 = Link('d',0.317,'a',0.081,'alpha',pi/2,'qlim',[-2*pi,2*pi], 'offset', 200);
L2 = Link('d',-0.1925,'a',0,'alpha',-pi/2,'qlim', [-2*pi,2*pi], 'offset', 80);
L3 = Link('d',-0.4,'a',0,'alpha',-pi/2,'qlim', [-2*pi,2*pi], 'offset', 110);
L4 = Link('d',-0.1685,'a',0,'alpha',pi/2,'qlim',[-2*pi,2*pi],'offset', 110); 
L5 = Link('d',0.4,'a',0,'alpha',-pi/2,'qlim',[-2*pi,2*pi], 'offset', 110);
L6 = Link('d',0.1363,'a',0,'alpha',pi/2,'qlim',[-2*pi,2*pi], 'offset', 110);
L7 = Link('d',0.13375,'a',0,'alpha',0,'qlim',[-2*pi,2*pi], 'offset', 110);
             
     
            
            self.model = SerialLink([L1 L2 L3 L4 L5 L6 L7],'name',name);
        end

        %% PlotAndColourRobot
        % Given a robot index, add the glyphs (vertices and faces) and
        % colour them in if data is available 
        function PlotAndColourRobot(self)%robot,workspace)
            for linkIndex = 0:self.model.n
                [ faceData, vertexData, plyData{linkIndex + 1} ] = plyread(['SAWYERLink',num2str(linkIndex),'.ply'],'tri'); %#ok<AGROW>
                self.model.faces{linkIndex + 1} = faceData;
                self.model.points{linkIndex + 1} = vertexData;
            end

            if ~isempty(self.toolModelFilename)
                [ faceData, vertexData, plyData{self.model.n + 1} ] = plyread(self.toolModelFilename,'tri'); 
                self.model.faces{self.model.n + 1} = faceData;
                self.model.points{self.model.n + 1} = vertexData;
                toolParameters = load(self.toolParametersFilename);
                self.model.tool = toolParameters.tool;
                self.model.qlim = toolParameters.qlim;
                warning('Please check the joint limits. They may be unsafe')
            end
            % Display robot
            self.model.plot3d(zeros(1,self.model.n),'noarrow','workspace',self.workspace);
            if isempty(findobj(get(gca,'Children'),'Type','Light'))
                camlight
            end  
            self.model.delay = 0;

            % Try to correctly colour the arm (if colours are in ply file data)
            for linkIndex = 0:self.model.n
                handles = findobj('Tag', self.model.name);
                h = get(handles,'UserData');
                try 
                    h.link(linkIndex+1).Children.FaceVertexCData = [plyData{linkIndex+1}.vertex.red ...
                                                                  , plyData{linkIndex+1}.vertex.green ...
                                                                  , plyData{linkIndex+1}.vertex.blue]/255;
                    h.link(linkIndex+1).Children.FaceColor = 'interp';
                catch ME_1
                    disp(ME_1);
                    continue;
                end
            end
        end        
    end
end