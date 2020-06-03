function [robot] = PlotSawyer(self)

            L1 = Link('d',0.317,'a',0.081,'alpha',pi/2,'offset',-pi/2,'qlim', [-pi,pi], 'offset',(270 *(pi/180)));

            L2 = Link('d',-0.1925,'a',0,'alpha',-pi/2,'offset',-pi/2,'qlim', [-pi/4,pi/4], 'offset', 0);

            L3 = Link('d',0.4,'a',0,'alpha',-pi/2,'offset',0,'qlim', [-pi,pi], 'offset', 0);

            L4 = Link('d',-0.1685,'a',0,'alpha',pi/2,'offset',0,'qlim', [-pi,pi], 'offset', 0);

            L5 = Link('d',0.4,'a',0,'alpha',-pi/2,'offset',0,'qlim', [-pi,pi], 'offset', 0);

            L6 = Link('d',0.1363,'a',0,'alpha',pi/2,'offset',0,'qlim', [-pi,pi], 'offset', 0);

            L7 = Link('d',0.13375,'a',0,'alpha',0,'offset',0,'qlim', [-pi,pi], 'offset', 0);

            robot = SerialLink([L1 L2 L3, L4 L5 L6 L7]);
            scale = 0.5;
            workspace = [-4 4 -4 4 -4 4];
            robot.base = transl(0,0,-0.9);
            q = zeros(1,7);
            robot.plot(q,'scale',scale);
           
         
            
end

