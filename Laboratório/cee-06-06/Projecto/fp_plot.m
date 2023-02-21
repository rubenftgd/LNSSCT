function fp_plot( op, fname, figure_number)
% Run this after fp_ini.m and one of fp_*.slx
% to obtain plots, or simply the signals

% March2017, J. Gaspar
% June2017 "MODIFIED" by João Ribafeita

% more operations asked via arg 'op'
switch op
    case 'load'
        fp_plot_data_load(fname)                    %loads file
        [t, ang1, ang2, ctrl_u]= fp_get_signals;    %loads signals
        max_index=fallen_pendulum(ang2);            %detects fallen pendulum
        fp_plot_signals(figure_number, max_index)   %plots signals
    otherwise
        error('invalid op')
end

return; % end of main function

% loads file given by main script
function fp_plot_data_load(fname)
% find all z_fp_data_*.mat
load( char(fname), 'pendulum' );
assignin( 'base', 'pendulum', pendulum );
% fprintf(1, '-- Pendulum data loaded from "%s"\n', fname);

% loads signal data from cell arrays, converting angles to degrees
function [t, ang1, ang2, ctrl_u]= fp_get_signals
pendulum= evalin('base', 'pendulum');
t= pendulum.time;
ang1= rad2deg(pendulum.signals(1).values(:,1));
ang2= rad2deg(pendulum.signals(1).values(:,2));
ctrl_u= pendulum.signals(2).values;

% detects if and when pendulum has fallen - \beta>15º
function [max_index] = fallen_pendulum(ang2)
     %detects if pendulum has fallen during test
     max_index = find(abs(ang2)>=15,1);
     %if pendulum has not fallen or test was stopped before t=60s, plot all
     if(isempty(max_index) == 1)
           max_index=length(ang2);
     end
     
function fp_plot_signals(figure_number, max_index)
%gets signals
[t, ang1, ang2, ctrl_u]= fp_get_signals;
%creates new figure, figure # given by main script
figure(figure_number);
   subplot(211);
      hold all;
      plot(t(1:max_index),ang1(1:max_index), 'LineWidth',0.75)
      legend('AutoUpdate','on')
      xlabel('time [sec]');
      ylabel('angle \alpha [º]')
   subplot(212);
      hold all;
      plot(t(1:max_index),ang2(1:max_index), 'LineWidth',0.75)
      xlabel('time [sec]');
      ylabel('angle \beta [º]')
figure(figure_number+1)
      hold all;
      plot(t(1:max_index),ctrl_u(1:max_index), 'LineWidth',0.75)
      legend('AutoUpdate','on')
      xlabel('t [sec]');
      ylabel('u [V]')