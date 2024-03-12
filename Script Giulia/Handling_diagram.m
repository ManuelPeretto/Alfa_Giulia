
function [alfaf_alfar,ay_g_agg,cont]=Handling_diagram(Force,Alfa,Tyre,Vehicle)

Fyf_Fzf=Force.Fyf ./ Vehicle.Fzf;
Fyr_Fzr=Force.Fyr ./ Vehicle.Fzr;
alfaf_vec=Alfa.alfaf;
alfar_vec=Alfa.alfar;

n=40;
alfaf_alfar=zeros(1,n);
max_ay_g=min(max(Fyr_Fzr),max(Fyf_Fzf));
ay_g=linspace(0,max_ay_g,n); % creo vettore con gli step di accellerazione ay/g, di cui voglio cercare le intersezioni
ay_g_agg=zeros(1,n+20);

indice=1;

for jj=2:n
    Fgf=Fyf_Fzf-ay_g(jj);
    Fgr=Fyr_Fzr-ay_g(jj);
    
    Posizione_f=Cerca_cambio_segno(Fgf);
    Posizione_r=Cerca_cambio_segno(Fgr);
    
    

    if numel(Posizione_f)==numel(Posizione_r)
        for jk=1:numel(Posizione_f)
        cont=indice+jk-1;
        alfaf_alfar(cont)=alfaf_vec(Posizione_f(jk))-alfar_vec(Posizione_r(jk));
        ay_g_agg(cont)=ay_g(jj);
        end
    elseif numel(Posizione_f)>=numel(Posizione_r)
        cont=indice+1;
        alfaf_alfar(indice)=alfaf_vec(Posizione_f(1))-alfar_vec(Posizione_r(1));
        alfaf_alfar(cont)=alfaf_vec(Posizione_f(2))-alfar_vec(Posizione_r(1));
        ay_g_agg(indice)=ay_g(jj);
        ay_g_agg(cont)=ay_g(jj);
    elseif numel(Posizione_f)<=numel(Posizione_r)
        cont=indice+1;
        alfaf_alfar(indice)=alfaf_vec(Posizione_f(1))-alfar_vec(Posizione_r(1));
        alfaf_alfar(cont)=alfaf_vec(Posizione_f(1))-alfar_vec(Posizione_r(2)); 
        ay_g_agg(indice)=ay_g(jj);
        ay_g_agg(cont)=ay_g(jj);
    end       
    indice=cont+1;
end 

M=round(max(max(Fyr_Fzr),max(Fyf_Fzf))+0.1 , 2);

figure
subplot(1,2,1)
hold on
plot(alfaf_vec,Fyf_Fzf);
plot(alfar_vec,Fyr_Fzr);
xlabel('\alpha [rad]') , ylabel ('Fy/Fz');
ylim([0 M]);
yline(Tyre.mu,'--r');
mu_txt = [' $\mu$ = ' , num2str(Tyre.mu)];
legend('front','rear',mu_txt,Interpreter='latex',fontsize=14);

subplot(1,2,2)
title('Handling Diagram')
set(gca,'XDir','reverse')
set(gca,YAxisLocation = 'right');
hold on
scatter(alfaf_alfar(1:cont),ay_g_agg(1:cont));
xlabel('\alphaf-\alphar [rad]') , ylabel('ay/g');
ylim([0 M]);
xline(0,'--');
yline(Tyre.mu,'--r')