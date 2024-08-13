clear all;clc;close all
A=importdata('map.txt');
B(1:120,1:180,1)=A(1:120,1:180)*0;
B(1:120,1:180,2)=A(1:120,1:180)*255;
B(1:120,1:180,3)=A(1:120,1:180)*255;
B=uint8(255-B);
clear A;
fileID = fopen('led.txt','w');
fprintf(fileID,'%d',0);
fclose(fileID);
for i=1:100000
    tic
    C=B;
    X=importdata('Xparticle.txt');
    Y= importdata('Yparticle.txt');
    if (length(X)==length(Y))
        if (mod(i,3)==0)
            x=max(1,round((100*X)/0.5));
            y=max(1,round((100*(.6-Y))/0.5));
            for j=1:length(x)
              C(y(j),x(j),1)=0;
            end
                  
            f1=figure(1);
            imshow(C)
            set(f1, 'Position', [800 420 400 250])
        end
        f2=figure(2);
        plot(X,Y,'.')
        
        axis([0 .9 0 .6])
        set(f2, 'Position', [800 60 500 300]);
        
        for j=0:85
            xRange5=find(j/100<=X & X<=j/100+0.05);
            for p=0:55
                yRange5=find(p/100<=Y(xRange5) & Y(xRange5)<=p/100+0.05);
                xyRange5{j+1,p+1}=xRange5(yRange5);
                lenMat5(j+1,p+1)=length(xyRange5{j+1,p+1});
            end  
        end
        [a5,b5]=find(lenMat5==max(lenMat5(:)));
        x5(i)=mean(X(xyRange5{a5(1),b5(1)}));
        y5(i)=mean(Y(xyRange5{a5(1),b5(1)}));
        L5(i)=length(xyRange5{a5(1),b5(1)});
        p5=L5(i)/length(X);

        for j=0:80
            xRange10=find(j/100<=X & X<=j/100+0.1);
            for p=0:50
                yRange10=find(p/100<=Y(xRange10) & Y(xRange10)<=p/100+0.1);
                xyRange10{j+1,p+1}=xRange10(yRange10);
                lenMat10(j+1,p+1)=length(xyRange10{j+1,p+1});
            end  
        end
        [a10,b10]=find(lenMat10==max(lenMat10(:)));
        x10(i)=mean(X(xyRange10{a10(1),b10(1)}));
        y10(i)=mean(Y(xyRange10{a10(1),b10(1)}));
        L10(i)=length(xyRange10{a10(1),b10(1)});
        p10=L10(i)/length(X);
        if (p5)>=.8
            fileID = fopen('led.txt','w');
            fprintf(fileID,'%d',1);
            fclose(fileID);
        elseif ((p10)>=.6 && (p5)<0.8)
            fileID = fopen('led.txt','w');
            fprintf(fileID,'%d',2);
            fclose(fileID);
        else
            fileID = fopen('led.txt','w');
            fprintf(fileID,'%d',0);
            fclose(fileID);
        end
%         for j=0:75
%             xRange15=find(j/100<=X & X<=j/100+0.15);
%             for p=0:45
%                 yRange15=find(p/100<=Y(xRange15) & Y(xRange15)<=p/100+0.15);
%                 xyRange15{j+1,p+1}=xRange15(yRange15);
%                 lenMat15(j+1,p+1)=length(xyRange15{j+1,p+1});
%             end  
%         end
%         [a15,b15]=find(lenMat15==max(lenMat15(:)));
%         x15(i)=mean(X(xyRange15{a15(1),b15(1)}));
%         y15(i)=mean(Y(xyRange15{a15(1),b15(1)}));
%         L5(i)=length(xyRange10{a15(1),b15(1)});
        
        hold on
%         plot(x5,y5,'.r','markersize',10);
        rectangle('position',[(a5(1)-1)/100,(b5(1)-1)/100,.05,.05],'curvature',[0 0],'EdgeColor','r','linewidth',2)
        text((a5(1)+5)/100,(b5(1)+5)/100,num2str(p5),'color','r','fontsize',15);
        rectangle('position',[(a10(1)-1)/100,(b10(1)-1)/100,.1,.1],'curvature',[0 0],'EdgeColor','g','linewidth',2)
        text((a10(1)+12)/100,(b10(1)+12)/100,num2str(p10),'color','g','fontsize',15);
%         rectangle('position',[(a15(1)-1)/100,(b15(1)-1)/100,.15,.15],'curvature',[0 0],'EdgeColor','k','linewidth',1)

        hold off
    end
    toc
end




