function HabitsGUI_wifi(n_row, n_col)
% HabitsGUI_wifi shows Graphic User Interface (GUI) for HABITS:
%   Home-cage Assissted Behavioral Inovation and Training System
%
%      Input Arguments: cage number in row and column
%           n_row x n_col (default 8 x 13)
%
%      Function:
%           Display Trials info & Messages received from Homecages;
%           Send Commands to Homecages;
%
%       Created on                04/25/2021   by Y.H. yaoyaoh90@gmail.com

%% Main GUI
handles.hfigure = figure('name','Habits GUI WiFi', 'numbertitle','off', 'MenuBar','none',...
    'CloseRequestFcn', @closeGUI_Callback, 'Position', get(0,'Screensize'));
if nargin < 2
    n_row = 6;   % default: rows
    n_col = 15;  % default: columns
end
handles.total_cage_num = n_row * n_col;

% SD udp file 
handles.finish_flag = 0; % only work in the function : fetch all files
handles.route_file = '';
handles.auto_upload_cage = 0;
handles.auto_upload_cage_list = 1:1:90; % update every 10 min
handles.SDFinishSignal = false;

left_panel_width = 0.1;
margin_border = 0.015;
margin_between_cage = 0.005;
cage_panel_height = (1-margin_border*2-margin_between_cage*(n_row-1))/n_row;
cage_panel_width = (1-left_panel_width-margin_border*2-margin_between_cage*(n_col-1))/n_col;

handles.default_color = [.7 .9 .8];  % default background color [.93 .93 .93];
hsv_map = hsv(24);
handles.mymap = hsv_map(1:8,:); % 1:8 from red to yellow to green

% ip address
[~,hostname] = system('hostname');
hostname = string(strtrim(hostname));
handles.localIP = resolvehost(hostname,"address");
handles.localPort = 52021;
handles.remotePort = 52021;
handles.remotePortSD = handles.remotePort + 1;
handles.ip_base = 100;

handles.folder_path = fileparts(which('HabitsGUI_wifi.m'));
addpath(handles.folder_path);
%% Main GUI: Cage Panel
for i_row = 1:n_row         % rows
    for i_col = 1:n_col     % columns

        this_cage = n_col*(i_row-1)+i_col; % cage number

        if ~exist([handles.folder_path '/Data/Cage',int2str(this_cage)], 'dir')
            mkdir([handles.folder_path '/Data/Cage',int2str(this_cage)]);
        end

        % Panel for each cage
        handles.hpanel_cage(this_cage) = uipanel('Title',['Cage ' num2str(this_cage)],...
            'FontWeight','bold',...
            'Position',[(cage_panel_width+margin_between_cage)*(i_col-1)+left_panel_width+margin_border+0.005
            1-margin_border-cage_panel_height-margin_between_cage - (cage_panel_height+margin_between_cage)*(i_row-1)
            cage_panel_width
            cage_panel_height]);

        % Mice Name
        handles.hedit_mouseID(this_cage) = uicontrol('Parent',handles.hpanel_cage(this_cage),...
            'Style','edit', 'String','YH00',...
            'Units', 'normalized',        'Tag', num2str(this_cage),...
            'Callback',@edit_mouseID_Callback,...
            'Position',[0.01, 0.84, 0.3, 0.15]);
        if ~exist([handles.folder_path '/Data/Cage',num2str(this_cage),'/YH00'], 'dir')
            mkdir([handles.folder_path '/Data/Cage',num2str(this_cage),'/YH00']);
        end

        % trial numbers in 24 hr.
        handles.htext_trailNum24hr(this_cage) = uicontrol('Parent',handles.hpanel_cage(this_cage),...
            'style','text',...
            'String', 'Trial#/d',        'Units', 'normalized',...
            'Position',[0.32, 0.84, 0.35, 0.15]);

        % Weight
        handles.htext_weight(this_cage) = uicontrol('Parent',handles.hpanel_cage(this_cage),...
            'style','text',...
            'String', '00.0 g',        'Units', 'normalized',...
            'Position',[0.68, 0.84, 0.31, 0.15]);

        % Select a start date button
        handles.hedit_date(this_cage) = uicontrol('Parent',handles.hpanel_cage(this_cage),...
            'Style','edit','String','dd-mmm',...
            'Units', 'normalized',        'Tag', num2str(this_cage),...
            'UserData', NaN, ...
            'Position',[0.01, 0.68, 0.4 0.15]);

        handles.hbutton_chooseStartDate(this_cage) = uicontrol('Parent',handles.hpanel_cage(this_cage),...
            'Style','pushbutton','String','...',...
            'Units', 'normalized',        'Tag', num2str(this_cage),...
            'Callback',@button_chooseStartDate_Callback,...
            'Position',[0.42, 0.68, 0.15 0.15]);

        % Days
        handles.htext_days(this_cage) = uicontrol('Parent',handles.hpanel_cage(this_cage),...
            'Style','text','String','00.0 d',...
            'Units', 'normalized',        'Tag', num2str(this_cage),...
            'Position',[0.59, 0.68, 0.4, 0.15]);

        % IP address
        dot_ind = find(handles.localIP == '.');
        handles.hedit_ip_address(this_cage) = uicontrol('Parent',handles.hpanel_cage(this_cage),...
            'Style','edit', 'String',[handles.localIP(1:dot_ind(end)), num2str(handles.ip_base+this_cage)],...
            'Units', 'normalized',        'Tag', num2str(this_cage),...
            'Tooltip', 'IP Address of the Cage', ...
            'Callback',@edit_ip_Callback,...
            'Position',[0.01, 0.52, 0.65, 0.15]);

        handles.ip_address{this_cage} = [handles.localIP(1:dot_ind(end)), num2str(handles.ip_base+this_cage)];

        handles.hbutton_open(this_cage) = uicontrol('Parent',handles.hpanel_cage(this_cage),...
            'Style','pushbutton','String','Open',...
            'Units', 'normalized',        'Tag', num2str(this_cage),...
            'Callback',@button_open_Callback,...
            'Position',[0.68, 0.52, 0.31, 0.15]);

        % Plot button
        handles.hbutton_plot(this_cage) = uicontrol('Parent',handles.hpanel_cage(this_cage),...
            'Style','pushbutton','String','Plot',...
            'Units', 'normalized',        'Tag', num2str(this_cage),...
            'Callback',@button_plot_Callback,...
            'Position',[0.05, 0.36, 0.4, 0.15],...
            'Enable','on');

        % Open Message txt file
        handles.hbutton_msg(this_cage) = uicontrol('Parent',handles.hpanel_cage(this_cage),...
            'Style','pushbutton','String','Msg',...
            'Units', 'normalized',        'Tag', num2str(this_cage),...
            'Callback',@button_msg_Callback,...
            'Position',[0.5, 0.36, 0.4, 0.15]);

        % Trial # and Perf100
        handles.htext_trailperf(this_cage) = uicontrol('Parent',handles.hpanel_cage(this_cage),...
            'style','text',...
            'String', 'Trial# - ??%',        'Units', 'normalized',...
            'Position',[0.01, 0.25, 0.6, 0.1]);
        handles.htext_earlylick(this_cage) = uicontrol('Parent',handles.hpanel_cage(this_cage),...
            'style','text',...
            'String', 'EL: ??%',        'Units', 'normalized',...
            'Position',[0.65, 0.25, 0.34, 0.1]);

        % Protocol info: currentProtocol-trialsNumsinThisProtocol-Perf50
        handles.htext_protocol(this_cage) = uicontrol('Parent',handles.hpanel_cage(this_cage),...
            'style','text',...
            'String', 'iProt - Trial# - ??%',        'Units', 'normalized',...
            'Position',[0.05, 0.14, 0.9, 0.1]);

        % Task info: currentTask
        handles.hedit_task(this_cage) = uicontrol('Parent',handles.hpanel_cage(this_cage),...
            'style','text','FontSize' ,7,...
            'String', 'task:??(no_space)',        'Units', 'normalized',...
            'Position',[0.05, 0.00, 0.9, 0.15]);

        set_background_color(this_cage,handles.default_color); % set default background color
    end
end

%% Main GUI: Left Panel
% ip address
handles.htext_ipAddress = uicontrol('Style','text',...
    'String',['IP: ' handles.localIP '; Port: ' num2str(handles.localPort)],...
    'Units', 'normalized',...
    'Position',[0.01,0.97,left_panel_width,0.02]);

% Custom message
handles.hedit_customMsg = uicontrol('Style','edit',...
    'String','Custom Message: ', ...
    'Units', 'normalized', ...
    'HorizontalAlignment', 'left', ...
    'Max',3, 'Min', 1, ... % Max - Min > 1, multiple lines
    'Position',[0.01,0.83,left_panel_width,0.15]);

% control panel
handles.hpanel_control = uipanel('Title','Control Panel',...
    'Position',[0.01 .55 left_panel_width .26]);
% chose a cage and Read
cage_cell = cell(handles.total_cage_num,1);
for this_cage = 1:handles.total_cage_num
    cage_cell{this_cage} = ['Cage ' num2str(this_cage)];
end
handles.hpopup_Cage = uicontrol('parent', handles.hpanel_control,...
    'Style','popupmenu',...
    'String',[{'Choose a Cage'} ; cage_cell],...
    'Units', 'normalized', ...
    'Position',[0.05,0.84,0.5,0.14],...
    'Callback',{@button_controlPanel,'Read'},...
    'TooltipString', 'Which Cage to Control');
handles.hbutton_readParas = uicontrol('parent', handles.hpanel_control,...
    'Style','pushbutton','String','Read',...
    'Units', 'normalized',...
    'Callback',{@button_controlPanel,'Read'},...
    'Position',[0.6,0.86,0.35,0.14]);
% Reward left middle right
handles.hedit_rewardLeft = uicontrol('parent', handles.hpanel_control,...
    'Style','edit','String','30',...
    'Units', 'normalized',...
    'TooltipString', 'Left',...
    'Position',[0.05,0.72,0.2,0.1]);
handles.hedit_rewardMiddle = uicontrol('parent', handles.hpanel_control,...
    'Style','edit','String','30',...
    'Units', 'normalized',...
    'TooltipString', 'Middle',...
    'Position',[0.26,0.72,0.2,0.1]);
handles.hedit_rewardRight = uicontrol('parent', handles.hpanel_control,...
    'Style','edit','String','30',...
    'Units', 'normalized',...
    'TooltipString', 'Right',...
    'Position',[0.47,0.72,0.2,0.1]);
handles.hbutton_reward = uicontrol('parent', handles.hpanel_control,...
    'Style','pushbutton','String','Set&Rew',...
    'Units', 'normalized',...
    'Callback',{@button_controlPanel,'Reward'},...
    'Position',[0.68,0.72,0.25,0.1]);
% light intensity control 
handles.hedit_lowlight = uicontrol('parent', handles.hpanel_control,...
    'Style','edit','String','1',...
    'Units', 'normalized',...
    'TooltipString', 'low light intensity',...
    'Position',[0.05,0.25,0.2,0.1]);
handles.hedit_highlight = uicontrol('parent', handles.hpanel_control,...
    'Style','edit','String','255',...
    'Units', 'normalized',...
    'TooltipString', 'high light intensity',...
    'Position',[0.30,0.25,0.2,0.1]);
handles.hbutton_light_intensity = uicontrol('parent', handles.hpanel_control,...
    'Style','pushbutton','String','set LT',...
    'Units', 'normalized',...
    'Callback',{@button_controlPanel,'LightIntensity'},...
    'Position',[0.60,0.25,0.25,0.1]);
% weight
handles.htext_tare = uicontrol('parent', handles.hpanel_control,...
    'Style','text','String','Weight: ',...
    'Units', 'normalized',...
    'Position',[0.05,0.61,0.2,0.1]);
handles.hedit_weight = uicontrol('parent', handles.hpanel_control,...
    'Style','edit','String','0',...
    'Units', 'normalized',...
    'Position',[0.26,0.61,0.2,0.1]);
handles.hbutton_tare = uicontrol('parent', handles.hpanel_control,...
    'Style','pushbutton','String','Tare',...
    'Units', 'normalized',...
    'Callback',{@button_controlPanel,'Tare'},...
    'Position',[0.47,0.61,0.22,0.1]);
handles.hbutton_cali = uicontrol('parent', handles.hpanel_control,...
    'Style','pushbutton','String','Cali(20g)',...
    'Units', 'normalized',...
    'Tooltip','Put the 20-g weight on the weight stage before you click this button!',...
    'Callback',{@button_controlPanel,'Cali'},...
    'Position',[0.70,0.61,0.22,0.1]);

% reserved lines: 0.5 0.39 0.28 0.17
handles.hedit_time = uicontrol('parent', handles.hpanel_control,...
    'Style','edit','String','current time',...
    'Units', 'normalized',...
    'Position',[0.05,0.5,0.45,0.1]);

handles.hbutton_correctTime = uicontrol('parent', handles.hpanel_control,...
    'Style','pushbutton','String','Correct Time',...
    'Units', 'normalized',...
    'Callback',{@button_controlPanel,'Time'},...
    'Position',[0.55,0.5,0.4,0.1]);

handles.hbutton_pauseHabits = uicontrol('parent', handles.hpanel_control,...
    'Style','pushbutton','String','Pause Habits',...
    'Units', 'normalized',...
    'Callback',{@button_controlPanel,'Pause'},...
    'Position',[0.55,0.39,0.4,0.1]);

handles.hbutton_handshake = uicontrol('parent', handles.hpanel_control,...
    'Style','pushbutton','String','Handshake',...
    'Units', 'normalized',...
    'Callback',{@button_controlPanel,'Handshake'},...
    'Position',[0.05,0.39,0.45,0.1]);
% protocol control
handles.hedit_protocol = uicontrol('parent', handles.hpanel_control,...
    'Style','edit','String','0',...
    'Units', 'normalized',...
    'TooltipString', 'current protocol',...
    'Position',[0.20,0.15,0.2,0.1]);
handles.hbutton_protocol = uicontrol('parent', handles.hpanel_control,...
    'Style','pushbutton','String','set protocol',...
    'Units', 'normalized',...
    'Callback',{@button_controlPanel,'protocol'},...
    'Position',[0.60,0.15,0.25,0.1]);
% extra TimeOut
handles.hedit_extra_TimeOut = uicontrol('parent', handles.hpanel_control,...
    'Style','edit','String','0',...
    'Units', 'normalized',...
    'TooltipString', 'current extra TO',...
    'Position',[0.05,0.04,0.2,0.1]);
handles.hbutton_extra_TimeOut = uicontrol('parent', handles.hpanel_control,...
    'Style','pushbutton','String','set extra TO',...
    'Units', 'normalized',...
    'Callback',{@button_controlPanel,'extra_TimeOut'},...
    'Position',[0.30,0.04,0.30,0.1]);
% SD card
handles.hbutton_SDcard = uicontrol('parent', handles.hpanel_control,...
    'Style','pushbutton','String','SD Card Files (todo...)',...
    'Units', 'normalized',...
    'Callback',@button_SDcard,...
    'Position',[0.60,0.02,0.4,0.13]);

% listbox for error message
handles.htext_errorMsg = uicontrol('Style','text','String','Error Message',...
    'Units', 'normalized',        'Tag', 'text_errorMsg',...
    'Position',[(left_panel_width-0.02)/2,0.52,0.06,0.02]);
handles.hlistbox = uicontrol('Style','listbox',...
    'Units', 'normalized','Position',[0.01 0.22 left_panel_width 0.3]);
handles.hbutton_deleteSel = uicontrol('Style','pushbutton','String','Delete Sel',...
    'Units', 'normalized',        'Tag', 'button_deleteSel',...
    'Callback',@button_deleteSel_Callback,...
    'Position',[left_panel_width-0.08,0.19,0.04,0.03]);
handles.hbutton_clearAll = uicontrol('Style','pushbutton','String','Clear All',...
    'Units', 'normalized',        'Tag', 'button_clearAll',...
    'Callback',@button_clearAll_Callback,...
    'Position',[left_panel_width-0.04,0.19,0.04,0.03]);

% load and save parameters
handles.hbutton_saveParas = uicontrol('Style','pushbutton','String','Save Paras',...
    'Units', 'normalized',...
    'Callback',@button_saveParas_Callback,...
    'Position',[left_panel_width-0.09,0.14,0.05,0.05]);
handles.hbutton_loadParas = uicontrol('Style','pushbutton','String','Load Paras',...
    'Units', 'normalized',...
    'Callback',@button_loadParas_Callback,...
    'Position',[left_panel_width-0.09,0.09,0.05,0.05]);
handles.hbutton_analyseData = uicontrol('Style','pushbutton','String','Analyse Data',...
    'Units', 'normalized',...
    'Callback',@button_analyseData_Callback,...
    'Position',[left_panel_width-0.09,0.04,0.05,0.05]);

% Show Color bar for the trial number in last 24 hours
ax = axes;
colormap(handles.mymap);
colorbar(ax,'Ticks',0.0625:0.125:1,'TickLabels',{'120','240','360','480','600','720','840 (24hr)','960 Trials'},...
    'Position',[left_panel_width-0.03, 0.02, 0.01, 0.15],'YAxisLocation','right');
ax.Visible = 'off';

%% UDP and GUI Timer

% Create an UDP port for receiving data from cages
handles.udp = udpport("datagram",'LocalPort',handles.localPort);
configureCallback(handles.udp,"datagram",1,@UDP_callback);
% Create another UDP port for receiving files from cages
handles.udp_sd = udpport("datagram",'LocalPort',handles.localPort+1);
configureCallback(handles.udp_sd ,"datagram" ,1 ,@UDP_sd_callback);

% Create Timer to update GUI (background color and weight) periodically
TimerInteval = 600;       % (sec) Default timer interval (10 min)
handles.update_timer = timer('Name','updateGUI_Timer', 'TimerFcn',{@updateGUI},...
    'Period',TimerInteval,'ExecutionMode','fixedRate');
start(handles.update_timer);
% Create Timer to upload the Trial.txt of all cages periodically
%TimerInteval_Trial = 3600 * 12; % every 12 hours upload the trial.txt file
%handles.upload_timer = timer("Name" ,'uploadTrial_Timer' ,'TimerFcn',{@uploadTrial},'Period' ,TimerInteval_Trial ,'ExecutionMode' ,'fixedRate');
%start(handles.upload_timer);
    %% Callback Functions

    % Edit Mouse ID
    function edit_mouseID_Callback(source,~)
        cage_str = get(source,'Tag'); 
        mouse_id = get(source,'String');
        % Create a folder for the mice under the dirctory of that Cage
        if ~exist([handles.folder_path '/Data/Cage',cage_str,'/',mouse_id], 'dir')
            mkdir([handles.folder_path '/Data/Cage',cage_str,'/',mouse_id]);
        end
    end
    
    function edit_ip_Callback(source,~)
        cage_num = str2double(get(source,'Tag'));
        % update ip_address container
        handles.ip_address{cage_num} = get(source,'String');
    end

    % Open UDP port
    function button_open_Callback(source,~)
        cage_str = get(source,'Tag');
        cage_num = str2double(cage_str);
        if strcmp(get(source,'String'), 'Open')
            % open a panel for listening data from cage

            % if ~invalid(ip_address) return; % todo...
            for i_cage = 1:handles.total_cage_num
                if strcmp(get(handles.hbutton_open(i_cage),'String'), 'Close') && strcmp(handles.ip_address{cage_num}, handles.ip_address{i_cage})
                    msgbox(["Cage " num2str(cage_nm) ": " handles.ip_address{cage_num} ' is being used by Cage ' num2str(i_cage) '. Please input another IP address.']);
                    return;
                end
            end
            if strcmp(handles.ip_address{cage_num}, handles.localIP)
                msgbox(["Cage " num2str(cage_nm) ": " handles.ip_address{cage_num} ' is being used by this PC.']);
                return;
            end

            set(source,'String','Close');
            set(handles.hedit_mouseID(cage_num),'enable','OFF');
            write(handles.udp,['T' num2str(floor(posixtime(datetime('now'))))],'char',handles.ip_address{cage_num},handles.remotePort); % Command: correct 'T'ime: 'T1633597282'

            mouse_name = get(handles.hedit_mouseID(cage_num),'String');
            % msg, trial and weight mat file to record
            handles.fileID(cage_num) = fopen([handles.folder_path '/Data/Cage',cage_str,'/',mouse_name,'/msg.txt'],'a');
            if exist([handles.folder_path '/Data/Cage',cage_str,'/',mouse_name,'/allmat.mat'], 'file') == 0
                handles.matObj{cage_num} = matfile([handles.folder_path '/Data/Cage',cage_str,'/',mouse_name,'/allmat.mat'],'Writable',true);
                handles.matObj{cage_num}.trial_info = []; % date trial# trial_type protocol trial_outcome earlylick resevered1 resevered2 resevered3 resevered4
                handles.matObj{cage_num}.weight = []; % date weight
                mkdir([handles.folder_path '/Data/Cage',cage_str,'/',mouse_name,'/SD_card_files']);
            else
                handles.matObj{cage_num} = matfile([handles.folder_path '/Data/Cage',cage_str,'/',mouse_name,'/allmat.mat'],'Writable',true);
            end

            [mRow,~] = size(handles.matObj{cage_num},'trial_info');
            if mRow > 0
                % update trial#/d and background color
                trial_time = handles.matObj{cage_num}.trial_info(1:mRow,1);
                num_trial_24hr = sum(trial_time > now - 1);

                set(handles.htext_trailNum24hr(cage_num),'String',[num2str(num_trial_24hr),'/d']);
                if num_trial_24hr > 960
                    num_trial_24hr = 960;
                end
                if num_trial_24hr == 0
                    num_trial_24hr = 1;
                end
                set_background_color(cage_num,handles.mymap(ceil(num_trial_24hr/120),:));

                % update trial info
                last_trial = handles.matObj{cage_num}.trial_info(mRow,[2 4]); % date trial# trial_type protocol trial_outcome earlylick
                set(handles.htext_trailperf(cage_num), 'String', [num2str(last_trial(1)) ' - ??%']);
                set(handles.htext_protocol(cage_num), 'String', [num2str(last_trial(2)) ' - ?? - ??%']);

            else
                set(handles.htext_trailNum24hr(cage_num),'String',[num2str(0),'/d']);
                set_background_color(cage_num,handles.mymap(1,:));
            end
        else
            % close the panel
            fclose(handles.fileID(cage_num));
            set(source,'String','Open');
            set(handles.hedit_mouseID(cage_num),'enable','ON');
            set_background_color(cage_num,handles.default_color); % set default background color
        end
    end

    % set start Date button
    function button_chooseStartDate_Callback(source,~)
        cage_num = str2double(get(source,'Tag'));
        D = uigetdate();
        if ~isempty(D)
            set(handles.hedit_date(cage_num),'String',datestr(D,'dd-mmm'), 'UserData', D);
        end
        set(handles.htext_days(cage_num),'String',sprintf('%2.1f d',now - get(handles.hedit_date(cage_num), 'Userdata')));
    end

    % Plot buttons
    function button_plot_Callback(source,~)
        cage_num = str2double(get(source,'Tag'));
        % history performance vs. days
        % trials in last 24 hours
        % weight in last 24 hours (w/ first day weight)
        figure,
        current_time = now; % floor(posixtime(datetime('now')));
        [mRow,~] = size(handles.matObj{cage_num},'trial_info');
        if mRow > 0
            % trail_info: date trial# trial_type protocol trial_outcome earlylick
            trial_info = handles.matObj{cage_num}.trial_info(1:mRow,1:6);
            trial_date = trial_info(:,1);
            trial_type = trial_info(:,3);
            trial_outcome = trial_info(:,5);
            start_date = get(handles.hedit_date(cage_num), 'UserData');
            n_days = ceil(now - start_date);
            perf = NaN(n_days,1);
            for i_day = 1:n_days
                outcome_day = trial_outcome(trial_date > start_date+i_day-1 & trial_date < start_date+i_day);
                perf(i_day) = sum(outcome_day == 1)/(numel(outcome_day)-sum(outcome_day==0)); % exclude no-response trials
            end
            subplot(3,1,1), hold on, plot(perf,'-o'); yline(0.75,'g');
            if n_days < 10
                xlim([0.5 10.5]);
            else
                xlim([0.5 n_days+0.5]);
            end
            ylim([0 1]);
            xlabel('Days'); ylabel('Performance'); title([handles.hedit_mouseID(cage_num).String ': History Performance']);

            % %%%%%%%%%% subplot(3,1,2)
            trial_date_24hr_index = trial_date > current_time-1;
            trial_date_24hr = trial_date(trial_date_24hr_index);
            trial_type_24hr = trial_type(trial_date_24hr_index); % 1,2,3: left, right, middle
            trial_outcome_24hr = trial_outcome(trial_date_24hr_index);

            subplot(3,1,2), hold on,
            plot(trial_date_24hr(trial_outcome_24hr==2),trial_type_24hr(trial_outcome_24hr==2),'rx','markersize',15);
            plot(trial_date_24hr(trial_outcome_24hr==0),trial_type_24hr(trial_outcome_24hr==0),'bo');
            plot(trial_date_24hr(trial_outcome_24hr==1),trial_type_24hr(trial_outcome_24hr==1),'g.','markersize',20);
            % todo: dark cycle plot
            trial_outcome_24hr_01 = trial_outcome_24hr;
            trial_outcome_24hr_01(trial_outcome_24hr_01 == 0) = NaN;
            trial_outcome_24hr_01(trial_outcome_24hr_01 == 2) = 0;
            plot(trial_date_24hr,(movmean(trial_outcome_24hr_01,50,'omitnan') + 0.5) * 2,'k'); % rescale to 1~3
            yline(2.5,'g'); % 75% line
            xlim([current_time-1 current_time]); ylim([0.5 3.5]);
            set(gca,'ytick',[1 2 3],'yticklabel',{'Left', 'Right', 'Middle'});
            set(gca,'xtick',[current_time-1 current_time-0.75 current_time-0.5 current_time-0.25 current_time],'xticklabel',{'0','6','12','18','24'});
            title([num2str(numel(trial_date_24hr_index)), ' trials in last 24 hours (perf: ' num2str(100*sum(trial_outcome_24hr==1)/(numel(trial_outcome_24hr)-sum(trial_outcome_24hr==0)),'%.1f') '%)']); % exclude no-response trial
        else
            subplot(3,1,1), title('History Performance: no history');
            subplot(3,1,2), title('0 trials in last 24 hours');
        end

        [mRow,~] = size(handles.matObj{cage_num},'weight');
        if mRow > 0
            weight_info = handles.matObj{cage_num}.weight(1:mRow,1:2); % ??
            weight_date = weight_info(:,1);
            weight_date_index = weight_date>current_time-1;
            weight_data = weight_info(:,2);
            subplot(3,1,3), hold on,
            yline(0);
            plot(weight_date(weight_date_index), weight_data(weight_date_index), '.');
            xlim([current_time-1 current_time]); ylim([-5 35]);
            set(gca,'xtick',[current_time-1 current_time-0.75 current_time-0.5 current_time-0.25 current_time],'xticklabel',{'0','6','12','18','24'});
            xlabel('Time (hour)'); ylabel('Weight (g)');
            weight_data_index = weight_data(weight_date_index);
            range = 15:0.1:35;
            N = histcounts(weight_data_index,range);
            [~, ind] = max(N);
            yline(range(ind),'k');
            title(['Weights in last 24 hours: avg. ' num2str(range(ind)), ' g']);
        else
            subplot(3,1,3), title('Weights in last 24 hours: no weight data');
        end
    end

    % Open Message txt file
    function button_msg_Callback(source,~)
        cage_str = get(source,'Tag');
        cage_num = str2double(cage_str);
        mouse_name = get(handles.hedit_mouseID(cage_num),'String');
        eval(['!notepad ',handles.folder_path '/Data/Cage',cage_str,'/',mouse_name,'/msg.txt &']); % return immediately with '&' (not working in mac)
    end

    function button_SDcard(~,~)
        handles.hfigure_SD = figure('name','SD card files', 'numbertitle','off', 'MenuBar','none');%
        cage_fetched = 0;
        % chose a cage and Read
        cage_cell_sd = cell(handles.total_cage_num,1);
        for i = 1:handles.total_cage_num
            cage_cell_sd{i} = ['Cage ' num2str(i)];
        end
        handles.hpopup_SD_Cage = uicontrol('parent', handles.hfigure_SD,...
            'Style','popupmenu',...
            'String',[{'Choose a Cage'} ; cage_cell_sd],...
            'Units', 'normalized', ...
            'Position',[0.2,0.8,0.2,0.1]);
        handles.hbutton_SD_fetch_list = uicontrol('parent', handles.hfigure_SD,...
            'Style','pushbutton','String','Fetch SD file list',...
            'Units', 'normalized',...
            'Callback',@button_SD_fetch_list,...
            'Position',[0.5,0.85,0.3,0.05]);

        handles.hlistbox_SD_list = uicontrol('Parent',handles.hfigure_SD,...
            'Style','listbox','String',{'please choice a cage first'},...
            'Units', 'normalized',...
            'Position',[0.1, 0.15, 0.35 0.64]); % List

        handles.hbutton_SD_send = uicontrol('Parent',handles.hfigure_SD,...
            'Style','pushbutton','String','Send files to SD...',...
            'Units', 'normalized',...
            'Callback',@button_SD_send,...
            'Position',[0.5, 0.63, 0.4 0.1]);
        handles.hbutton_SD_fetch_file = uicontrol('Parent',handles.hfigure_SD,...
            'Style','pushbutton','String','Fetch selected file from SD',...
            'Units', 'normalized',...
            'Callback',@button_SD_fetch_file,...
            'Position',[0.5, 0.49, 0.4 0.1]);
        handles.hbutton_SD_remove = uicontrol('Parent',handles.hfigure_SD,...
            'Style','pushbutton','String','Remove selected file from SD',...
            'Units', 'normalized',...
            'Callback',@button_SD_remove,...
            'Position',[0.5, 0.35, 0.4 0.1]);
        handles.hbutton_SD_fetch_all = uicontrol('Parent',handles.hfigure_SD,...
            'Style','pushbutton','String','Fetch all files in SD',...
            'Units', 'normalized',...
            'Callback',@button_SD_fetch_all,...
            'Position',[0.5, 0.21, 0.4 0.1]);
        % build-in udp SD callback function
      %  configureCallback(handles.udp_sd,"datagram",1,@UDP_sd_callback);
    
       %messageSD = src.read(src.NumDatagramsAvailable,"string");
       % if isempty(messageSD)
       %     return;
       % end
        %for i_msgSD = 1:numel(messageSD)
        %   if msgSD(1) == 'F' % end of uploading this SD file
         %       if msgSD(2) == 'H'
         %       finish_flag = 1;
         %       break
         %       end
         %   end
         %   fid = fopen(route_file,'a'); % add new data at the tail of this file
         %   fprintf(fid,'%c',msgSD);
         %   fclose(fid);
        %end
        %end
        

        function button_SD_fetch_list(~,~)
            cage_num = get(handles.hpopup_SD_Cage,'Value') - 1;
            if get(handles.hpopup_SD_Cage,'Value') ~= 1 % not 'Choose a port'
                 if strcmp(get(handles.hbutton_open(cage_num),'String'), 'Open')
                    msgbox(['Cage ', num2str(cage_num), ' is CLOSED!']);
                    return;
                 end
                write(handles.udp,'D','char',handles.ip_address{cage_num},handles.remotePort);
                cage_fetched = 1; % TODO
            else
                msgbox('Choose a cage first');
                return;
            end
        end

        function button_SD_send(~,~)
            if cage_fetched 
                % get the file path 
                cage_num = get(handles.hpopup_SD_Cage,'Value') - 1;
                cage_str = num2str(cage_num);
                mouse_name = get(handles.hedit_mouseID(cage_num) ,'string');
                source_file = [handles.folder_path, '\Data\Cage',cage_str,'\',mouse_name]; 
                [file,path ,index] = uigetfile({'*.txt';'*.hex'} ,'Please Select a .hex or a .txt file','Multiselect','off' ,source_file);
                if ~isequal(file,0)
                        % send file name and size
                        filePath = [path file];
                        fileInfo = dir(filePath);
                        write(handles.udp ,char('Y' + string(file) + ',' + num2str(fileInfo.bytes) + ',') ,"char" ,handles.ip_address{cage_num} ,handles.remotePort);
                        % send file content 
                        FileSD = matlab.io.datastore.DsFileReader(filePath);
                        count512 = 511;
                        pause(1);
                        % 512 BYTES AS ONE packet 511 data + 1 bytes
                        % prehead code 'K'
                        while count512 == 511
                            [packet512 ,count512] = read(FileSD ,511 ,'OutputType','char');
                            write(handles.udp ,char('K' + string(packet512))  ,"string" ,handles.ip_address{cage_num} ,handles.remotePort);
                            pause(100/1000);
                        end
                       % receive finish signal 10 times retransmisstion
                       % available
                       for i=1:10
                            write(handles.udp ,char('E') ,"char" ,handles.ip_address{cage_num} ,handles.remotePort);
                            pause(1);
                            if handles.SDFinishSignal == true
                                handles.SDFinishSignal = false;
                                break;
                            end
                       end
                       msgbox("The file is downloaded!");
                        % update list 
                 end
                
            else
                msgbox('Choose a cage first');
            end
        end

        function button_SD_fetch_file(~,~) 
            if cage_fetched
            cage_num = get(handles.hpopup_SD_Cage,'Value') - 1;
            cage_str = num2str(cage_num);
            fetchID = get(handles.hlistbox_SD_list ,'value'); % the fileID to be fetch in the file_name
            file_name = get(handles.hlistbox_SD_list ,'string');
            fetched_file_name = file_name(fetchID);
            % make sure the file existed is the origin file of the consponding
            % mouse; never happen
            mouse_name = get(handles.hedit_mouseID(cage_num) ,'string');
            source_file = [handles.folder_path, '\Data\Cage',cage_str,'\',mouse_name]; 
            handles.route_file = string(source_file) + '/' + string(fetched_file_name);
            cd(source_file);
            if exist(string(fetched_file_name),'file')
                if fetchID ~= 1
                delete(handles.route_file);
                end
            end

            % '/Data/Cage',cage_str,'/',mouse_name,'/SD_card_files'], move
            % to backup folder


            % send command to microController to fetch files
            % send file name
            
            write(handles.udp,char('F' + string(fetched_file_name) + ','),'char',handles.ip_address{cage_num},handles.remotePort);% fetch files in the sd card
            % receive file content
            % save file to disk
         else
                msgbox('Choose a cage first');
            end
       end

        function button_SD_fetch_all(~,~)
            % for items in list % todo...
            % button_SD_fetch_file
            % end
            if cage_fetched
            cage_num = get(handles.hpopup_SD_Cage,'Value') - 1;
            cage_str = num2str(cage_num);
            file_name = get(handles.hlistbox_SD_list ,'string');
            for fetchID = 1:length(file_name)
            fetched_file_name = file_name(fetchID);
            mouse_name = get(handles.hedit_mouseID(cage_num) ,'string');
            source_file = [handles.folder_path, '\Data\Cage',cage_str,'\',mouse_name]; 
            handles.route_file = string(source_file) + '/' + string(fetched_file_name);
            cd(source_file);
            if exist(string(fetched_file_name),'file')
                %delete(route_file);
            end
            write(handles.udp,char('F' + string(fetched_file_name) + ','),'char',handles.ip_address{cage_num},handles.remotePort);% fetch files in the sd card
            while 1 % 10kb/s upload speed UDP
                %pause(1);
            if handles.finish_flag == 1
            break
            end
            end
            handles.finish_flag = 0;
            end
            else
                msgbox('Choose a cage first');
            end
        end

        function button_SD_remove(~,~)
            if cage_fetched
            cage_num = get(handles.hpopup_SD_Cage,'Value') - 1;
            selectedId = get(handles.hlistbox_SD_list, 'Value');        % get id of selectedLabelName
            existingItems = get(handles.hlistbox_SD_list, 'String');    % get current listbox list
            
            % send command to microController to delete file
            write(handles.udp,char('B' + string(existingItems(selectedId)) + ','),'char',handles.ip_address{cage_num},handles.remotePort);
            % send file name todo...
            
            n_items = length(existingItems);
            if(n_items <= 1)
                new_list = '';
                set(handles.hlistbox_SD_list, 'String', new_list)
            else
                % If the returned list is a cell array there are three cases
                n_items = length(existingItems);
                if(selectedId == 1)
                    % The first element has been selected
                    new_list={existingItems(2:end)};
                elseif(selectedId == n_items)
                    % The last element has been selected
                    new_list={existingItems(1:end-1)};
                    % Set the "Value" property to the previous element
                    set(handles.hlistbox_SD_list, 'Value', selectedId-1);
                else
                    % And element in the list has been selected
                    new_list={existingItems{1:selectedId-1} existingItems{selectedId+1:end}};
                end
            end
            % Update the list
            set(handles.hlistbox_SD_list, 'String', new_list);     % restore cropped version of label list

            else
            msgbox('Choose a cage first');
            end
        end
    end


    function button_deleteSel_Callback(~, ~)
        selectedId = get(handles.hlistbox, 'Value');        % get id of selectedLabelName
        existingItems = get(handles.hlistbox, 'String');    % get current listbox list
        n_items = length(existingItems);

        if(n_items <= 1) %class(existingItems) == 'char'
            set(handles.hlistbox, 'String', '');
            return;
        else
            % If the returned list is a cell array there are three cases
            if(selectedId == 1)
                % The first element has been selected
                new_list={existingItems(2:end)};
            elseif(selectedId == n_items)
                % The last element has been selected
                new_list={existingItems(1:end-1)};
                % Set the "Value" property to the previous element
                set(handles.hlistbox, 'Value', selectedId-1);
            else
                % And element in the list has been selected
                new_list={existingItems{1:selectedId-1} existingItems{selectedId+1:end}};
            end
            % Update the list
            set(handles.hlistbox, 'String', new_list');     % restore cropped version of label list
        end
    end


    function button_clearAll_Callback(~, ~)
        set(handles.hlistbox, 'String', '');
    end


    % save and load button callbacks
    function button_saveParas_Callback(~, ~)
        para = struct();
        for i = 1:handles.total_cage_num
            para(i).ip_address = handles.ip_address{i};
            para(i).mouse_name = get(handles.hedit_mouseID(i),'String');
            para(i).start_date = get(handles.hedit_date(i),'UserData');
            para(i).panel_open_state = get(handles.hbutton_open(i),'String');
            para(i).task_name = get(handles.hedit_task(i),'String');
        end
        para(1).custom_msg = strtrim(string(get(handles.hedit_customMsg,'String')));
        set(handles.hedit_customMsg, 'String', para(1).custom_msg);
        uisave('para','./para.mat');
    end

    function button_loadParas_Callback(~, ~)
        [file,path] = uigetfile;
        if ~isequal(file,0)
            file_loaded = load([path file]);
            for i = 1:handles.total_cage_num
                if strcmp(get(handles.hbutton_open(i), 'String'), 'Open')
                    set(handles.hedit_ip_address(i),'String',file_loaded.para(i).ip_address)
                    handles.ip_address{i} = file_loaded.para(i).ip_address;

                    set(handles.hedit_mouseID(i), 'String', file_loaded.para(i).mouse_name);
                    set(handles.hedit_task(i), 'String', file_loaded.para(i).task_name);

                    if ~isnan(file_loaded.para(i).start_date)
                        set(handles.hedit_date(i),'String',datestr(file_loaded.para(i).start_date,'dd-mmm'));
                        set(handles.hedit_date(i),'UserData',file_loaded.para(i).start_date);
                        set(handles.htext_days(i),'String',sprintf('%2.1f d',now - get(handles.hedit_date(i), 'Userdata')));
                    else
                        set(handles.hedit_date(i),'String','dd-mmm');
                        set(handles.hedit_date(i),'UserData',NaN);
                        set(handles.htext_days(i),'String','NaN d');
                    end

                    if strcmp(file_loaded.para(i).panel_open_state, 'Close')
                        button_open_Callback(handles.hbutton_open(i));
                    end
                end
            end
            set(handles.hedit_customMsg, 'String', file_loaded.para(1).custom_msg);
        end
    end
	
	function button_analyseData_Callback(~, ~)
		% get required params in this python script below
		analysis_button = questdlg("Are you sure do the analysis?" ,"check box" ,"Yes","No" ,"No");
		switch analysis_button
			case "Yes"
				python_command = ['activate habits & python E:\HABITS_NEW\Habits_dev\SD_data_analysis\script\plot_mice_weekly.py '];
				file_head = [];
				file_protocol = [];
				current_opening_cage_ana = 0;
				for i_cage = 1:handles.total_cage_num
					if strcmp(get(handles.hbutton_open(i_cage),'String'), 'Close')
						current_opening_cage_ana = current_opening_cage_ana + 1;
						% file source
						mouse_name = get(handles.hedit_mouseID(i_cage),'String');
						cage_str = num2str(i_cage);
						file_head = [file_head ,handles.folder_path,'/Data/Cage',cage_str,'/',mouse_name ,' '];
						file_protocol = [file_protocol ,get(handles.hedit_task(i_cage),'String') , ' '];
					end
				end
				python_command = [python_command ,'--MiceID ', file_head, '--protocol ', file_protocol, '&'];
				% msgbox(python_command);
				[status ,~] = system(python_command);
				if status ~= 0
					msgbox("python script failed!");
				else 
					msgbox("python script is running!");
				end
			case "No"
				return;
		end
	end


    function button_controlPanel(~, ~, arg)
        % current unix time: floor(posixtime(datetime('now','TimeZone','local'))) %
        % (seconds since 1970, GMT+0)
        % Get cage number
        value = get(handles.hpopup_Cage,'Value');
        if value == 1
            msgbox('Please choose a cage number first!');
            return;
        else
            cage_num = value - 1;
        end
        % Make sure the corresponding cage is open
        if strcmp(get(handles.hbutton_open(cage_num),'String'), 'Open')
            msgbox(['Cage ', num2str(cage_num), ' is CLOSED!']);
            return;
        end
        % Command
        switch arg
            case 'Read' % read
                write(handles.udp,'A','char',handles.ip_address{cage_num},handles.remotePort); % Command: read 'A'll info
            case 'Reward'
                write(handles.udp,['R' get(handles.hedit_rewardLeft,'String') ',' get(handles.hedit_rewardRight,'String') ',' get(handles.hedit_rewardMiddle,'String') ','],...
                    'char',handles.ip_address{cage_num},handles.remotePort); % Command: 'R'eward
            case 'Tare'
                selection = questdlg('Make sure the mouse is not on the plateform!',...
                    'Warning',...
                    'Yes','No','No');
                switch selection
                    case 'Yes'
                        write(handles.udp,'0','char',handles.ip_address{cage_num},handles.remotePort); % Command: 'zero' weight
                        % msgbox('Done!');
                    case 'No'
                        return;
                end
            case 'Cali'
                selection = questdlg('Please put the 20-g-weight on the palteform and click OK!',...
                    'Warning',...
                    'OK','Cannel','Cannel'); %
                switch selection
                    case 'OK'
                        write(handles.udp,'C','char',handles.ip_address{cage_num},handles.remotePort); % Command: 'C'alibration
                        % msgbox('Done!');
                    case 'Cannel'
                        return;
                end
            case 'Time'
                write(handles.udp,['T' num2str(floor(posixtime(datetime('now'))))],'char',handles.ip_address{cage_num},handles.remotePort); % Command: correct 'T'ime: 'T1633597282'
            case 'Pause'
                if strcmp(get(handles.hbutton_pauseHabits, 'String'), 'Pause Habits')
                    write(handles.udp,'P','char',handles.ip_address{cage_num},handles.remotePort); % Command: 'P'ause Habits
                    set(handles.hbutton_pauseHabits, 'String', 'Resume Habits');
                else
                    write(handles.udp,'M','char',handles.ip_address{cage_num},handles.remotePort); % Command: resu'm'e Habits
                    set(handles.hbutton_pauseHabits, 'String', 'Pause Habits');
                end
            case 'Handshake'
                write(handles.udp,'H','char',handles.ip_address{cage_num},handles.remotePort); % Command: 'H'andshake
            case 'LightIntensity'
                write(handles.udp,['L' get(handles.hedit_lowlight,'String') ',' get(handles.hedit_highlight,'String') ','],...
                    'char',handles.ip_address{cage_num},handles.remotePort); % Command: 'L'ight
            
            case 'protocol'
                protocol_button = questdlg("Are you sure change the protocol?" ,"check box" ,"Yes","No" ,"No");
                switch protocol_button
                    case "Yes"
                        write(handles.udp,['Z' get(handles.hedit_protocol,'String') ','],...
                    'char',handles.ip_address{cage_num},handles.remotePort);
                    case "No"
                        return;
                end
            case 'extra_TimeOut'
                extra_TimeOut_button = questdlg("Are you sure add the extra TimeOut?" ,"check box" ,"Yes","No" ,"No");
                switch extra_TimeOut_button
                    case "Yes"
                        write(handles.udp,['E' get(handles.hedit_extra_TimeOut,'String') ','],...
                    'char',handles.ip_address{cage_num},handles.remotePort);
                    case "No"
                        return;
                end
            otherwise
                disp('no this command for button_controlPanel');
        end
    end
    %% UDP Callback
    function UDP_callback(src,~)
        message = src.read(src.NumDatagramsAvailable,"string");
        if isempty(message)
            return;
        end
        for i_msg = 1:numel(message)
            ip_address = char(message(i_msg).SenderAddress);
            ind = find(ip_address == '.');
            cage_num = str2double(ip_address(ind(end)+1:end))-handles.ip_base; % determine which cage
            % read data and store in Cage folder
            msg = char(message(i_msg).Data);

            if msg(1) == 'S' % special message will be received and processed even when the cage is closed
    
                switch msg(2)
                    case 'E' % message will show up in the Erros Message panel
                        add_msg_to_errList(cage_num, msg(5:end));
                        ind = find(msg == '.');
                        set(handles.hedit_task(cage_num),'String', msg(ind(end)+1:end));% read task name from SD card
                        % fprintf(handles.fileID(cage_num),[msg(5:end) '\n']);
                        write(handles.udp,['T' num2str(floor(posixtime(datetime('now'))))],'char',handles.ip_address{cage_num},handles.remotePort); % Command: correct 'T'ime: 'T1633597282'
                    case 'H' % Handshake
                        msgbox(["Handshake with cage " num2str(cage_num) " successfully!"]);
                        % fprintf(handles.fileID(cage_num),["Handshake with cage " num2str(cage_num) " successfully! \n"]);
                end
                continue;
            end

            if strcmp(get(handles.hbutton_open(cage_num),'String'), 'Open')
                continue;
            end

            switch msg(1)
                case 'T' % trial# trial_type  protocol trial_outcome earlylick perf100 trial#P perf50
                    fprintf(handles.fileID(cage_num),[msg '\n']);

                    msg = str2num(msg(2:end));
                    [mRow,~] = size(handles.matObj{cage_num},'trial_info');

                    handles.matObj{cage_num}.trial_info(mRow+1,1:6) = [now msg(1:5)]; % floor(posixtime(datetime('now')))

                    set(handles.htext_trailperf(cage_num),'String', [num2str(msg(1)) ' - ' num2str(msg(6)) '%']);
                    set(handles.htext_protocol(cage_num),'String', [num2str(msg(3)) ' - ' num2str(msg(7)) ' - ' num2str(msg(8)) '%']);
                    set(handles.htext_earlylick(cage_num),'String', ['EL:' num2str(msg(5)) '%']);

                    trial_24hr = get(handles.htext_trailNum24hr(cage_num),'String');
                    ind = find(trial_24hr == '/');
                    trial_24hr = str2double(trial_24hr(1:ind(1)-1));
                    set(handles.htext_trailNum24hr(cage_num),'String',[num2str(trial_24hr+1),'/d']);

                case 'W' % weight1; weight2; ... weight30;
                    msg = str2num(msg(2:end));
                    [mRow,~] = size(handles.matObj{cage_num},'weight');
                    handles.matObj{cage_num}.weight(mRow+1,1) = now;
                    weight_curr = mean(msg);
                    handles.matObj{cage_num}.weight(mRow+1,2) = weight_curr;
                    if all(msg > 14) % all are valid weight data
                        set(handles.htext_weight(cage_num), 'String', [num2str(weight_curr,'%.1f') ' g']); % update weight info
                    end

                case 'E' % emergency msg to disp in the list box
                    if msg(2) == 'S'
                        handles.SDFinishSignal = true; % SD send file ack
                    end
                    add_msg_to_errList(cage_num, msg(4:end));
                    fprintf(handles.fileID(cage_num),[msg '\n']);

                case 'A' % control panel info
                    ind = find(msg == ';');
                    extra_setting = length(ind);
                    %msgbox(num2str(size(ind)))
                    set(handles.hedit_rewardLeft, 'String', msg(2:ind(1)-1));
                    set(handles.hedit_rewardRight, 'String', msg(ind(1)+1:ind(2)-1));
                    set(handles.hedit_rewardMiddle, 'String', msg(ind(2)+1:ind(3)-1));
                    set(handles.hedit_lowlight, 'String', msg(ind(3)+1:ind(4)-1));
                    set(handles.hedit_highlight, 'String', msg(ind(4)+1:ind(5)-1));
                    set(handles.hedit_protocol, 'String', msg(ind(5)+1:ind(6)-1));
                    set(handles.hedit_weight, 'String', msg(ind(6)+1:ind(7)-1));
                    set(handles.hedit_time, 'String', string(datetime(str2double(msg(ind(7)+1:ind(8)-1)),'ConvertFrom','posixTime','format','HH:mm:ss yyyy-MM-dd')));
                    if extra_setting == 9
                    %msgbox(num2str(size(ind)));
                        set(handles.hedit_extra_TimeOut ,'String' ,msg(ind(8)+1:end-1));
                    end
                case 'D' % SD file info
                    cage_sd_file = cell(6 ,1);
                    file_name = ["Trial.txt" ,"event.txt" ,"cage_info.txt" ,"paraS.txt","Tevent.txt" ,"weight.txt"];
                    for i = 1:numel(file_name)
                    cage_sd_file{i} = file_name(i);
                    end
                    set(handles.hlistbox_SD_list, 'String', cage_sd_file);
                    add_msg_to_errList(cage_num, 'SD files wil be sent soon, training stopped!!!');
                    % puase the state machine and give a flag to GUI 
                    set(handles.hbutton_pauseHabits, 'String', 'Resume Habits');
                
                otherwise % case 'M'
                    fprintf(handles.fileID(cage_num),[msg(4:end) '\n']);
            end
        end

    end
    
    function UDP_sd_callback(src ,~)
        messageSD = src.read(src.NumDatagramsAvailable ,"string");
        if isempty(messageSD)
            return;
        end
        for i_msgSD = 1:numel(messageSD)
            msgSD = char(messageSD(i_msgSD).Data);
            if msgSD(1) == 'F'
                if msgSD(2) == 'H'
                    handles.finish_flag = 1;
                    break
                elseif msgSD(2) == 'A'
                    indSD = find(msgSD == '#');
                    cage_id = str2num(msgSD(3:indSD(1) -  2));
					if msgSD(indSD(1) -  1) == 'T'
						file_Name = 'Trial.txt';
                    elseif msgSD(indSD(1) -  1) == 'E'
						file_Name = 'Tevent.txt';
					end
					
                    
                    cage_str = num2str(cage_id);
                    mouse_name = get(handles.hedit_mouseID(cage_id) ,'string');
                    %msgbox(mouse_name);
                    source_file = [handles.folder_path, '\Data\Cage', cage_str ,'\',mouse_name];
                    handles.route_file = string(source_file) + '/' + file_Name;
                    %msgbox(handles.route_file);
                   
                    fid = fopen(handles.route_file ,'a');
                    fprintf(fid ,'%c' ,msgSD(indSD(1)+1:end));
                    fclose(fid);
                end
            else
                fid = fopen(handles.route_file ,'a');
                fprintf(fid ,'%c' ,msgSD);
                fclose(fid);
            end
        
        end
    end

    %% Timer Callback (Update GUI)
    function updateGUI(~,~)
        current_opening_cage = 0;
        % every 10 mins upload one cage's Trial.txt
        handles.auto_upload_cage = handles.auto_upload_cage + 1;
        if handles.auto_upload_cage > length(handles.auto_upload_cage_list)
            handles.auto_upload_cage = 1;
        end
       % if handles.auto_upload_cage == 40 || handles.auto_upload_cage == 41 || handles.auto_upload_cage == 42 || handles.auto_upload_cage == 43 
        handles.route_file = '';
        handles.finish_flag = 0;
        fetch_file = 'Trial.txt';
        write(handles.udp ,char('F' + string(fetch_file) + ',') ,'char' ,handles.ip_address{handles.auto_upload_cage_list(handles.auto_upload_cage)} ,handles.remotePort);
        %end
        
        for i_cage = 1:handles.total_cage_num
            if strcmp(get(handles.hbutton_open(i_cage),'String'), 'Close')
                current_opening_cage = current_opening_cage + 1;
                try
                    % update trial#/d and background color
                    [mRow,~] = size(handles.matObj{i_cage},'trial_info');
                    if mRow > 0
                        trial_time = handles.matObj{i_cage}.trial_info(1:mRow,1);
                        num_trial_24hr = sum(trial_time > now - 1);
                        set(handles.htext_trailNum24hr(i_cage),'String',[num2str(num_trial_24hr),'/d']);
                        if num_trial_24hr > 960
                            num_trial_24hr = 960;
                        end
                        if num_trial_24hr == 0
                            num_trial_24hr = 1;
                        end
                        set_background_color(i_cage,handles.mymap(ceil(num_trial_24hr/120),:));
                    else
                        set(handles.htext_trailNum24hr(i_cage),'String',[num2str(0),'/d']);
                        set_background_color(i_cage,handles.mymap(1,:)); % red
                    end
                catch e
                    disp(e);
                    disp("updateGUI-update trial#");
                end

                % special cases
                % e.g., if perf100 < 30% red.
                perf100 = get(handles.htext_trailperf(i_cage),'String');
                ind1 = find(perf100 == '-');
                ind2 = find(perf100 == '%');
                if str2double(perf100(ind1+2:ind2-1)) < 30 % performance < 30%
                    set_background_color(i_cage, handles.mymap(1,:)); % red
                    add_msg_to_errList(i_cage, 'Peformance Low!!!');
                end
                % e.g., if weights are negtive
                [mRow,~] = size(handles.matObj{i_cage},'weight');
                if mRow > 10
                    if all(handles.matObj{i_cage}.weight(mRow-9:mRow,2) < -10) % weight scale may be broken.
                        set_background_color(i_cage,handles.mymap(1,:)); % red
                        add_msg_to_errList(i_cage, 'Negtive weight!!!');
                    end
                end

                % correct time and upload Trial.txt file every day and every half of day;
               

                if hour(datetime('now')) == 1 && (minute(datetime('now')) > 0 && minute(datetime('now')) < 10) % 1:00 to 1:10
                    % correct time
                    write(handles.udp,['T' num2str(floor(posixtime(datetime('now'))))],'char',handles.ip_address{i_cage},handles.remotePort); % Command: correct 'T'ime: 'T1633597282'
                end
             

                % update days
                set(handles.htext_days(i_cage),'String',sprintf('%2.1f d',now - get(handles.hedit_date(i_cage), 'Userdata')));
            end
        end
        
        
            
    end
    
   
    %% Close Window Callbacks
    function closeGUI_Callback(hObject, ~)
        % handles = guidata(hObject);
        selection = questdlg('Close HABITS GUI?',...
            'Close Request Function',...
            'Yes','No','No');
        switch selection
            case 'Yes'
                disp('Habits GUI window closing...');
                try
                    stop(handles.update_timer); % stop timer
                    delete(timerfind);
                    delete(handles.udp);
                    delete(handles.udp_sd);
                    for i_cage = 1:handles.total_cage_num
                        if strcmp(get(handles.hbutton_open(i_cage),'String'), 'Close')
                            try
                                fclose(handles.fileID(i_cage)); % close file object
                            catch e
                                disp(e);
                            end
                        end
                    end
                    delete(hObject);
                catch e
                    disp(e);
                    delete(hObject);
                end
            case 'No'
                return;
        end
    end

    %% Other Functions
    function set_background_color(cage_num,color)
        set(handles.hpanel_cage(cage_num),'BackgroundColor',color);
        set(handles.hedit_mouseID(cage_num),'BackgroundColor',color);
        set(handles.htext_trailNum24hr(cage_num),'BackgroundColor',color);
        set(handles.hedit_date(cage_num),'BackgroundColor',color);
        set(handles.hbutton_chooseStartDate(cage_num),'BackgroundColor',color);
        set(handles.htext_days(cage_num),'BackgroundColor',color);
        set(handles.hedit_ip_address(cage_num),'BackgroundColor',color);
        set(handles.hbutton_open(cage_num),'BackgroundColor',color);
        set(handles.hbutton_msg(cage_num),'BackgroundColor',color);
        set(handles.hbutton_plot(cage_num),'BackgroundColor',color);
        set(handles.htext_trailperf(cage_num),'BackgroundColor',color);
        set(handles.htext_protocol(cage_num),'BackgroundColor',color);
        set(handles.htext_weight(cage_num),'BackgroundColor',color);
        set(handles.htext_earlylick(cage_num),'BackgroundColor',color);
        set(handles.hedit_task(cage_num),'BackgroundColor' ,color);
    end

    function add_msg_to_errList(cage_num, msg)
        existingItems = get(handles.hlistbox, 'String');    % get current listbox list
        n_items = length(existingItems);
        existingItems{n_items+1} = ['Cage',num2str(cage_num),': ',msg,' (',datestr(now,'mm/dd HH:MM'),')'];
        set(handles.hlistbox, 'String', existingItems);
        set(handles.hlistbox, 'Value', n_items+1);
    end
end