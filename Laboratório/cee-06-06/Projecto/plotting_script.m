%% *Plotting script*
%
%  June2017, João Ribafeita
%  
%  
%  Used for plotting lab data
%  For a matter of convenience, the filenames used were not the ones created
%  by fp_plot. Instead, files were renamed 'lab0x_testeyy', where x stands
%  for the lab session of file creation and yy stands for the test number
%  within the lab session.
%  Parameters used are indicated, according to notes taken during lab
%  sessions, after running each test.

%% *ENVIAR .txt com testes e parâmetros?*

%% *Inicialização*
clear all
close all
clc;
warning off
%%
%  Gráficos com dados obtidos nos laboratórios.
%  Testes agrupados, dentro do possível, pela variação de um parâmetro,
%  mantendo os outros constantes, de modo a analisar o seu efeito sobre o
%  sistema.
%  Na legenda, (P) indica um teste em que foram utilizados os mesmos
%  parâmetros de um teste anterior (também indicado), mas foram induzidas
%  perturbações no sistema, de modo a analisar a sua resposta e robustez.
%  
%  Utilizados subplots diferentes para \alpha e \beta, de modo a facilitar a
%  visualização de vários testes (relativos ao mesmo parâmetro) ao mesmo
%  tempo.
%  
%  Utilizadas figuras diferentes para sinal de controlo e ângulos do
%  pêndulo, de modo a facilitar a sua inclusão no relatório e consequente
%  formataçãodo mesmo.
%% Variação de Rr

% "array" of files to be loaded (tests grouped by line, tests in each column
files=({'lab1_teste01' 'lab1_teste02' '' '';...
        'lab2_teste07' 'lab2_teste03' 'lab2_teste08' '';...
        'lab3_teste09' 'lab3_teste10' 'lab3_teste11' 'lab3_teste12'});
% "array" of legends to be used in the plots
legenda_rr=({'R_r=1' 'R_r=0.8' '' '';...%caso base
             'R_r=0.8' 'R_r=1' 'R_r=1.5' '';...%Q_r_{1,1}=100, Q_r_{3,3}=10
             'R_r=0.04' 'R_r=0.04 (P)' 'R_r=0.1' 'R_r=0.07'});%%Q_r_{1,1}=275, Q_r_{3,3}=600
% "array" of titles to be used in the plots
titulos_rr=({'Efeitos da variação de R_r, com Q_{r_{1,1}}=10 e Q_{r_{3,3}}=1';...
            'Efeitos da variação de R_r, com Q_{r_{1,1}}=100 e Q_{r_{3,3}}=10';...
            'Efeitos da variação de R_r, com Q_{r_{1,1}}=275 e Q_{r_{3,3}}=600'});
% plotting
figura=1000;
grupos=size(files);

for grupo_testes=1:grupos(1)
   fim=0;
   teste=1;
   while teste<=grupos(2) && fim ==0 %plots each test in the "group"
       if(isempty(files{grupo_testes,teste}) == 0)
           fp_plot('load', files(grupo_testes,teste),figura);
       else
           fim=1; %ends cycle if no more tests in the group
       end
       teste=teste+1;
   end
   figure(figura)
   subplot(211);
   title(char(titulos_rr(grupo_testes)));
   legend(legenda_rr(grupo_testes,1:teste-1), 'Location', 'northwest');
   plot(linspace(0,60,120), zeros(120,1), '--g', 'Linewidth',1.5, 'DisplayName', '0');
   subplot(212);
   ylim([-15 15])
   plot(linspace(0,60,120), zeros(120,1), '--g', 'Linewidth',1.5);
   figure(figura+1);
   title(char(titulos_rr(grupo_testes)));
   ylim([-5 5])
   legend(legenda_rr(grupo_testes,1:teste-1), 'Location', 'northwest')
   plot(linspace(0,60,120), zeros(120,1), '--g', 'Linewidth',1.5, 'DisplayName', '0');
   figura=figura+2;
end

%% Variação de Qr(1,1)

figura=figura+2;
% "array" of files to be loaded (tests grouped by line, tests in each column
files=({'lab2_teste01' 'lab2_teste02' '' '' '';...
        'lab1_teste04' 'lab2_teste03' 'lab2_teste04' 'lab2_teste05' '';...
        'lab3_teste01' 'lab3_teste03' 'lab3_teste05' 'lab3_teste06' 'lab3_teste08';...
        'lab3_teste02' 'lab3_teste04' 'lab3_teste07' '' ''});

% "array" of legends to be used in the plots
legenda_qr1=({'Q_{r_{1,1}}=10' 'Q_{r_{1,1}}=100' '' '' '';
             'Q_{r_{1,1}}=100' 'Q_{r_{1,1}}=100' 'Q_{r_{1,1}}=80' 'Q_{r_{1,1}}=40' '';
             'Q_{r_{1,1}}=14.59' 'Q_{r_{1,1}}=67' 'Q_{r_{1,1}}=131.3129' 'Q_{r_{1,1}}=200' 'Q_{r_{1,1}}=250';...
             'Q_{r_{1,1}}=14.59(P)' 'Q_{r_{1,1}}=67(P)' 'Q_{r_{1,1}}=200(P)' '' ''});
         
% "array" of titles to be used in the plots
titulos_qr1=({'Efeitos da variação de Q_{r_{1,1}}, com Q_{r_{3,3}}=1 e R_r=1';...
              'Efeitos da variação de Q_{r_{1,1}}, com Q_{r_{3,3}}=10 e R_r=1';...
              'Efeitos da variação de Q_{r_{1,1}}, com Q_{r_{3,3}}=364.7563 e R_r=0.04';...
              'Efeitos da variação de Q_{r_{1,1}}, com Q_{r_{3,3}}=364.7563 e R_r=0.04 (P)'});
% plotting
grupos=size(files);

for grupo_testes=1:grupos(1)
   fim=0;
   teste=1;
   while teste<=grupos(2) && fim ==0 %plots each test in the "group"
       if(isempty(files{grupo_testes,teste}) == 0)
           fp_plot('load', files(grupo_testes,teste),figura);
       else
           fim=1; %ends cycle if no more tests in the group
       end
       teste=teste+1;
   end
   figure(figura)
   subplot(211);
   title(char(titulos_qr1(grupo_testes)));
   legend(legenda_qr1(grupo_testes,1:teste-1), 'Location', 'northwest');
   plot(linspace(0,60,120), zeros(120,1), '--g', 'Linewidth',1.5, 'DisplayName', '0');
   subplot(212);
   ylim([-15 15])
   plot(linspace(0,60,120), zeros(120,1), '--g', 'Linewidth',1.5);
   figure(figura+1);
   title(char(titulos_qr1(grupo_testes)));
   ylim([-5 5])
   legend(legenda_qr1(grupo_testes,1:teste-1), 'Location', 'northwest')
   plot(linspace(0,60,120), zeros(120,1), '--g', 'Linewidth',1.5, 'DisplayName', '0');
   figura=figura+2;
end


%% Variação de Qr(3,3)
figura=figura+2;

% "array" of files to be loaded (tests grouped by line, tests in each column
files=({'lab1_teste05' 'lab1_teste06' '' '';
        'lab2_teste02' 'lab1_teste04' 'lab2_teste03' 'lab2_teste06'});

% "array" of legends to be used in the plots
legenda_qr3=({'Q_{r_{3,3}}=10' 'Q_{r_{3,3}}=20' '' '';
              'Q_{r_{3,3}}=1' 'Q_{r_{3,3}}=10' 'Q_{r_{3,3}}=10' 'Q_{r_{3,3}}=20'});

% "array" of titles to be used in the plots
titulos_qr3=({'Efeitos da variação de Q_{r_{3,3}}, com Q_{r_{1,1}}=50 e R_r=1';...
              'Efeitos da variação de Q_{r_{3,3}}, com Q_{r_{1,1}}=100 e R_r=1'});

% plotting
grupos=size(files);

for grupo_testes=1:grupos(1)
   fim=0;
   teste=1;
   while teste<=grupos(2) && fim ==0 %plots each test in the "group"
       if(isempty(files{grupo_testes,teste}) == 0)
           fp_plot('load', files(grupo_testes,teste),figura);
       else
           fim=1; %ends cycle if no more tests in the group
       end
       teste=teste+1;
   end
   figure(figura)
   subplot(211);
   ylim([-80 80])
   title(char(titulos_qr3(grupo_testes)));
   legend(legenda_qr3(grupo_testes,1:teste-1), 'Location', 'northwest');
   plot(linspace(0,60,120), zeros(120,1), '--g', 'Linewidth',1.5, 'DisplayName', '0');
   subplot(212);
   ylim([-15 15])
   plot(linspace(0,60,120), zeros(120,1), '--g', 'Linewidth',1.5);
   figure(figura+1);
   title(char(titulos_qr3(grupo_testes)));
   ylim([-5 5])
   legend(legenda_qr3(grupo_testes,1:teste-1), 'Location', 'northwest')
   plot(linspace(0,60,120), zeros(120,1), '--g', 'Linewidth',1.5, 'DisplayName', '0');
   figura=figura+2;
end

%% TESTES AO LQE
%% Caso base
%  Caso presente no enunciado do lab, utilizando os parâmetros do lqr
%  determinados anteriormente como sendo os mais adequados

figura=figura+2;
% "array" of files to be loaded (tests grouped by line, tests in each column
files=({'lab3_teste09' 'lab3_teste10'});

% "array" of legends to be used in the plots
legenda_lqe=({'LQE base' 'LQE base (P)'});

% "array" of titles to be used in the plots
titulos_lqe=({'Caso LQE base, para Q_{r_{1,1}}=275, Q_{r_{3,3}}=600 e R_r=0.04'});

% plotting
grupos=size(files);

for grupo_testes=1:grupos(1)
   fim=0;
   teste=1;
   while teste<=grupos(2) && fim ==0 %plots each test in the "group"
       if(isempty(files{grupo_testes,teste}) == 0)
           fp_plot('load', files(grupo_testes,teste),figura);
       else
           fim=1; %ends cycle if no more tests in the group
       end
       teste=teste+1;
   end
   figure(figura)
   subplot(211);
   ylim([-80 80])
   title(char(titulos_lqe(grupo_testes)));
   legend(legenda_lqe(grupo_testes,1:teste-1), 'Location', 'northwest');
   plot(linspace(0,60,120), zeros(120,1), '--g', 'Linewidth',1.5, 'DisplayName', '0');
   subplot(212);
   ylim([-15 15])
   plot(linspace(0,60,120), zeros(120,1), '--g', 'Linewidth',1.5);
   figure(figura+1);
   title(char(titulos_lqe(grupo_testes)));
   ylim([-5 5])
   legend(legenda_lqe(grupo_testes,1:teste-1), 'Location', 'northwest')
   plot(linspace(0,60,120), zeros(120,1), '--g', 'Linewidth',1.5, 'DisplayName', '0');
   figura=figura+2;
end

%% Variação de Re
figura=figura+2;

% "array" of files to be loaded (tests grouped by line, tests in each column
files=({'lab3_teste13' 'lab3_teste15';
        'lab3_teste14' 'lab3_teste16'});

% "array" of legends to be used in the plots
legenda_lqe=({'Re=0.1' 'Re=0.01';
              'Re=0.05, G=0.95' 'Re=0.001, G=1'});

% "array" of titles to be used in the plots
titulos_lqe=({'Efeito da variação de Re, com Q_{r_{1,1}}=275, Q_{r_{3,3}}=600, R_r=0.04, G=1, Qe=0.1';
              'Efeito da variação de Re, com Q_{r_{1,1}}=275, Q_{r_{3,3}}=600, R_r=0.04, Qe=0.05'});

% plotting
grupos=size(files);

for grupo_testes=1:grupos(1)
   fim=0;
   teste=1;
   while teste<=grupos(2) && fim ==0 %plots each test in the "group"
       if(isempty(files{grupo_testes,teste}) == 0)
           fp_plot('load', files(grupo_testes,teste),figura);
       else
           fim=1; %ends cycle if no more tests in the group
       end
       teste=teste+1;
   end
   figure(figura)
   subplot(211);
   ylim([-60 60])
   title(char(titulos_lqe(grupo_testes)));
   legend(legenda_lqe(grupo_testes,1:teste-1), 'Location', 'northwest');
   plot(linspace(0,60,120), zeros(120,1), '--g', 'Linewidth',1.5, 'DisplayName', '0');
   subplot(212);
   ylim([-15 15])
   plot(linspace(0,60,120), zeros(120,1), '--g', 'Linewidth',1.5);
   figure(figura+1);
   title(char(titulos_lqe(grupo_testes)));
   ylim([-5 5])
   legend(legenda_lqe(grupo_testes,1:teste-1), 'Location', 'northwest')
   plot(linspace(0,60,120), zeros(120,1), '--g', 'Linewidth',1.5, 'DisplayName', '0');
   figura=figura+2;
end

%% Variação de Qe
figura=figura+2;

% "array" of files to be loaded (tests grouped by line, tests in each column
files=({'lab3_teste16' 'lab3_teste17'});

% "array" of legends to be used in the plots
legenda_lqe=({'Qe=0.05' 'Qe=0.01'});

% "array" of titles to be used in the plots
titulos_lqe=({'Efeito da variação de Qe, com Q_{r_{1,1}}=275, Q_{r_{3,3}}=600, R_r=0.04, G=1 e Re=0.001';});

% plotting
grupos=size(files);

for grupo_testes=1:grupos(1)
   fim=0;
   teste=1;
   while teste<=grupos(2) && fim ==0 %plots each test in the "group"
       if(isempty(files{grupo_testes,teste}) == 0)
           fp_plot('load', files(grupo_testes,teste),figura);
       else
           fim=1; %ends cycle if no more tests in the group
       end
       teste=teste+1;
   end
   figure(figura)
   subplot(211);
   ylim([-80 80])
   title(char(titulos_lqe(grupo_testes)));
   legend(legenda_lqe(grupo_testes,1:teste-1), 'Location', 'northwest');
   plot(linspace(0,60,120), zeros(120,1), '--g', 'Linewidth',1.5, 'DisplayName', '0');
   subplot(212);
   ylim([-15 15])
   plot(linspace(0,60,120), zeros(120,1), '--g', 'Linewidth',1.5);
   figure(figura+1);
   title(char(titulos_lqe(grupo_testes)));
   ylim([-5 5])
   legend(legenda_lqe(grupo_testes,1:teste-1), 'Location', 'northwest')
   plot(linspace(0,60,120), zeros(120,1), '--g', 'Linewidth',1.5, 'DisplayName', '0');
   figura=figura+2;
end
