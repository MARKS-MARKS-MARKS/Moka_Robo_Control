L1=Link('d',0,'a',0,'alpha',0,'modified');
L2=Link('d',0,'a',450.199,'alpha',-pi/2,'modified');
L3=Link('d',0,'a',0,'alpha',pi,'modified');
L4=Link('d',0,'a',99.566,'alpha',pi/2,'modified');
L5=Link('d',0,'a',0,'alpha',-pi/2,'modified');
L6=Link('d',0,'a',0,'alpha',pi/2,'modified');
MR07S=SerialLink([L1 L2 L3 L4 L5 L6],'name','MR07S','manufacturer','innfos');

mm=1;
n=[0 0 1];
r=80*mm;
c=[200 200 100]*mm;
step=80;
P = drawing_circle(n,r,c,step);
%求逆解
ikInitGuess=zeros(1,6);
for i=1:length(P)
    T(:,:,1)=transl(P(i,:))*rpy2tr([pi,0,0]);
    config=MR07S.ikunc(T(:,:,i),ikInitGuess);
    ikInitGuess=config;
    qrt(i,:)=config;
end
W=[-800,800,-800,800,-800,800];
MR07S.plot(qrt,'tilesize',150,'workspace',W,'view','x','trail',{'r','LineWidth',2})
