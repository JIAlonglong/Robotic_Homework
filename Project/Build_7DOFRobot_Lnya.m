% %Build Robot by D_H methods

none=0;
revolute=1;
prismatic=2;
fixed=3;
ToDeg = 180/pi;
ToRad = pi/180;
UX = [1 0 0]';
UY = [0 1 0]';
UZ = [0 0 1]';

Link    = struct('name','Body' , 'th',  0, 'dz', 0, 'dx', 0, 'alf',90*ToRad,'az',UZ,'Type',none);     % az 
Link(1)= struct('name','Base' , 'th',  0*ToRad, 'dz', 0, 'dx', 0, 'alf',0*ToRad,'az',UZ,'Type',none);        %Base To 1
Link(2) = struct('name', 'J1',   'th', 180*ToRad, 'dz', 0, 'dx', 89.459, 'alf', 90*ToRad, 'az', UZ,'Type',revolute);
Link(3) = struct('name', 'J2',   'th', 0*ToRad, 'dz', -425, 'dx', 0, 'alf', 0, 'az', UZ,'Type',revolute);
Link(4) = struct('name', 'J3',   'th', 0*ToRad, 'dz', -392.25, 'dx', 0, 'alf', 0, 'az', UZ,'Type',revolute);
Link(5) = struct('name', 'J4',   'th', 0*ToRad, 'dz', 0, 'dx', 109.15, 'alf', 90*ToRad, 'az', UZ,'Type',revolute);
Link(6) = struct('name', 'J5',   'th', 0*ToRad, 'dz', 0, 'dx', 94.65, 'alf', -90*ToRad, 'az', UZ,'Type',revolute);
Link(7) = struct('name', 'J6',   'th', 0*ToRad, 'dz', 0, 'dx', 82.3, 'alf', 0, 'az', UZ,'Type',revolute);
Link(8) = struct('name','J7',  'th',  0*ToRad, 'dz',  30,  'dx', 0, 'alf', 0, 'az',UZ,'Type',fixed);     %7 TO E
%Link(4) = struct('name','J3' , 'th',  0*ToRad, 'dz', 0, 'dx', 200, 'alf',0*ToRad,'az',UZ);          %3 TO E

% 
%       Link1             Link2
% % -------------[     ]------(O)
% %                            |
% %                            |
% %                            []
% %                            |
% %                            |
                           