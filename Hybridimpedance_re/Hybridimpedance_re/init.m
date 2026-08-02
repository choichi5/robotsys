% initial Program

%kp      = 1000.0;
kg      = 1;
g       = 9.81;
Link11  = 1.00;
Link12  = 1.00;
Link11g = Link11/2.0;
Link12g = Link12/2.0;
Mass11  = 0.50;
Mass12  = 0.50;
I11     = Mass11*Link11^2/12.0;
I12     = Mass12*Link12^2/12.0;
%%% straight line 
q11     = 45.0*pi/180.0;
q12     = 90.0*pi/180.0;
%%% circle input
%q11     = 30.0*pi/180.0;
%q12     = 120.0*pi/180.0;
Q      = [ q11
           q12
          ];
dQ     = [ 0.0
            0.0
          ];
%x=0.0;
%y=0.0;
%x01_d       = Link11*cos(q11)+Link12*cos(q11+q12);
%y01_d       = Link11*sin(q11)+Link12*sin(q11+q12);
%X_d         = [ x01_d
 %               y01_d   ];

%save('desire.txt','desire','-ascii','-tabs');


%%% END %%%