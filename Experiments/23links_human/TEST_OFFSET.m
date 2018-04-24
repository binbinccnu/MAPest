% TEST FLOATING BASE
% comparing taus and fext

clear all;
close all;
clc;

%% General setting
torque_plot = false;
force_plot  = true;

bucket = struct;
subjectID = 10;
trialID = 5;
bucket.pathToSubject = sprintf(fullfile(pwd,'/dataUW/Subj_%02d'),subjectID);
bucket.pathToTrial   = sprintf(fullfile(bucket.pathToSubject,'/trial_0%02d'),trialID);
bucket.pathToProcessedData   = fullfile(bucket.pathToTrial,'/processed');

var_fromFloating = load(fullfile(bucket.pathToProcessedData,'/floating/estimated_variables.mat'),'estimated_variables');
var_fromFixed_struct = load(fullfile(bucket.pathToProcessedData,'/shoes/extractedVariables.mat'),'extractedVariables');
var_fromFixed = var_fromFixed_struct.extractedVariables.values;

measFext = load(fullfile(bucket.pathToProcessedData,'/measuredForces.mat'),'measuredForces'); 

range_cut_plot = (1:2000); %2000 is manually added in MAP computation

%% Defining the torques vector for the comparison
tau_fromFloating_berdy = load(fullfile(bucket.pathToProcessedData, '/floating/computedTauFromBerdy.mat'),'computedTauFromBerdy');
% cut data
for i = 1 : size(tau_fromFloating_berdy.computedTauFromBerdy,2)
    tau_fromFloating_berdy.computedTauFromBerdy(i).values = ...
        tau_fromFloating_berdy.computedTauFromBerdy(i).values(:,range_cut_plot);
end

% At this stage, tau_fromFloating_berdy is not ordered as tau coming from
% the fixed version.  The following for ordering tau_fromFloating_berdy
% as the one from the fixed, making 2 bvariables consistent!
% ---rot_x
tau_fromFloating_berdy.computedTauFromBerdy_ordered.rotx = cell(length(var_fromFloating.estimated_variables.intForce.rotx),1);
for i = 1 : length(var_fromFloating.estimated_variables.intForce.rotx)
    for j = 1 : size(tau_fromFloating_berdy.computedTauFromBerdy,2)
        if strcmp(var_fromFixed_struct.extractedVariables.jointList.list_rotx{i,1}, cell2mat(tau_fromFloating_berdy.computedTauFromBerdy(j).label))
           tau_fromFloating_berdy.computedTauFromBerdy_ordered.rotx{i,1}.label = var_fromFixed_struct.extractedVariables.jointList.list_rotx{i,1};
           tau_fromFloating_berdy.computedTauFromBerdy_ordered.rotx{i,1}.tau   = tau_fromFloating_berdy.computedTauFromBerdy(j).values;
        end
    end
end
% ---rot_y
tau_fromFloating_berdy.computedTauFromBerdy_ordered.roty = cell(length(var_fromFloating.estimated_variables.intForce.roty),1);
for i = 1 : length(var_fromFloating.estimated_variables.intForce.roty)
    for j = 1 : size(tau_fromFloating_berdy.computedTauFromBerdy,2)
        if strcmp(var_fromFixed_struct.extractedVariables.jointList.list_roty{i,1}, cell2mat(tau_fromFloating_berdy.computedTauFromBerdy(j).label))
           tau_fromFloating_berdy.computedTauFromBerdy_ordered.roty{i,1}.label = var_fromFixed_struct.extractedVariables.jointList.list_roty{i,1};
           tau_fromFloating_berdy.computedTauFromBerdy_ordered.roty{i,1}.tau   = tau_fromFloating_berdy.computedTauFromBerdy(j).values;
        end
    end
end
% ---rot_z
tau_fromFloating_berdy.computedTauFromBerdy_ordered.rotz = cell(length(var_fromFloating.estimated_variables.intForce.rotz),1);
for i = 1 : length(var_fromFloating.estimated_variables.intForce.rotz)
    for j = 1 : size(tau_fromFloating_berdy.computedTauFromBerdy,2)
        if strcmp(var_fromFixed_struct.extractedVariables.jointList.list_rotz{i,1}, cell2mat(tau_fromFloating_berdy.computedTauFromBerdy(j).label))
           tau_fromFloating_berdy.computedTauFromBerdy_ordered.rotz{i,1}.label = var_fromFixed_struct.extractedVariables.jointList.list_rotz{i,1};
           tau_fromFloating_berdy.computedTauFromBerdy_ordered.rotz{i,1}.tau   = tau_fromFloating_berdy.computedTauFromBerdy(j).values;
        end
    end
end

%% =========================== TAU COMPARISON =============================
if torque_plot
    % cutting rotx data
    for i = 1 : length(var_fromFloating.estimated_variables.intForce.rotx)
        var_fromFloating.estimated_variables.intForce.rotx{i, 1}.projected_tau =  ...
            var_fromFloating.estimated_variables.intForce.rotx{i, 1}.projected_tau(:,range_cut_plot);
        var_fromFixed.tau.rotx{i, 1}.tau =  ...
            var_fromFixed.tau.rotx{i, 1}.tau(:,range_cut_plot);
    end

    % rotx plots             
    for i = 1 : length(var_fromFloating.estimated_variables.intForce.rotx)
        fig = figure();
        axes1 = axes('Parent',fig,'FontSize',16);
                  box(axes1,'on');
                  hold(axes1,'on');
                  grid on;
        plot1 = plot(var_fromFloating.estimated_variables.intForce.rotx{i, 1}.projected_tau,'b','lineWidth',1.5);
        hold on;
        plot2 = plot(var_fromFixed.tau.rotx{i, 1}.tau,'r','lineWidth',1.5);
        hold on;
        plot3 = plot(tau_fromFloating_berdy.computedTauFromBerdy_ordered.rotx{i,1}.tau,'g','lineWidth',1.5);
        ylabel('Torque [Nm]','HorizontalAlignment','center',...
           'FontWeight','bold',...
           'FontSize',18,...
           'Interpreter','latex');
       xlabel('N samples');
       title(sprintf('%s', var_fromFloating.estimated_variables.intForce.rotx{i, 1}.label), 'Interpreter', 'none');

       leg = legend([plot1,plot3,plot2],{'floating (manual)','floating (berdy)','fixed'});
       set(leg,'FontSize',13);
    end

    % -----------------------
    % cutting roty data
    for i = 1 : length(var_fromFloating.estimated_variables.intForce.roty)
        var_fromFloating.estimated_variables.intForce.roty{i, 1}.projected_tau =  ...
            var_fromFloating.estimated_variables.intForce.roty{i, 1}.projected_tau(:,range_cut_plot);
        var_fromFixed.tau.roty{i, 1}.tau =  ...
            var_fromFixed.tau.roty{i, 1}.tau(:,range_cut_plot);
    end
    % roty plots             
    for i = 1 : length(var_fromFloating.estimated_variables.intForce.roty)
        fig = figure();
        axes1 = axes('Parent',fig,'FontSize',16);
                  box(axes1,'on');
                  hold(axes1,'on');
                  grid on;
        plot1 = plot(var_fromFloating.estimated_variables.intForce.roty{i, 1}.projected_tau,'b','lineWidth',1.5);
        hold on;
        plot2 = plot(var_fromFixed.tau.roty{i, 1}.tau,'r','lineWidth',1.5);
        hold on;
        plot3 = plot(tau_fromFloating_berdy.computedTauFromBerdy_ordered.roty{i,1}.tau,'g','lineWidth',1.5);
        ylabel('Torque [Nm]','HorizontalAlignment','center',...
           'FontWeight','bold',...
           'FontSize',18,...
           'Interpreter','latex');
       xlabel('N samples');
       title(sprintf('%s', var_fromFloating.estimated_variables.intForce.roty{i, 1}.label), 'Interpreter', 'none');

       leg = legend([plot1,plot3,plot2],{'floating (manual)','floating (berdy)','fixed'});
       set(leg,'FontSize',13);
    end

    % -----------------------
    % cutting rotz data
    for i = 1 : length(var_fromFloating.estimated_variables.intForce.rotz)
        var_fromFloating.estimated_variables.intForce.rotz{i, 1}.projected_tau =  ...
            var_fromFloating.estimated_variables.intForce.rotz{i, 1}.projected_tau(:,range_cut_plot);
        var_fromFixed.tau.rotz{i, 1}.tau =  ...
            var_fromFixed.tau.rotz{i, 1}.tau(:,range_cut_plot);
    end
    % rotz plots
    for i = 1 : length(var_fromFloating.estimated_variables.intForce.rotz)
        fig = figure();
        axes1 = axes('Parent',fig,'FontSize',16);
                  box(axes1,'on');
                  hold(axes1,'on');
                  grid on;
        plot1 = plot(var_fromFloating.estimated_variables.intForce.rotz{i, 1}.projected_tau,'b','lineWidth',1.5);
        hold on;
        plot2 = plot(var_fromFixed.tau.rotz{i, 1}.tau,'r','lineWidth',1.5);
        hold on;
        plot3 = plot(tau_fromFloating_berdy.computedTauFromBerdy_ordered.rotz{i,1}.tau,'g','lineWidth',1.5);
        ylabel('Torque [Nm]','HorizontalAlignment','center',...
           'FontWeight','bold',...
           'FontSize',18,...
           'Interpreter','latex');
       xlabel('N samples');
       title(sprintf('%s', var_fromFloating.estimated_variables.intForce.rotz{i, 1}.label), 'Interpreter', 'none');

       leg = legend([plot1,plot3,plot2],{'floating (manual)','floating (berdy)','fixed'});
       set(leg,'FontSize',13);
    end
end
%% ======================= fext COMPARISON ===========================
% cutting fext measured
measFext.measuredForces.righFoot = measFext.measuredForces.righFoot(:,range_cut_plot);
measFext.measuredForces.leftFoot = measFext.measuredForces.leftFoot(:,range_cut_plot);

if force_plot
    baseLeftFoot = true;

    if baseLeftFoot
        % cutting data
        for i = 1 : length(var_fromFixed.extForce)
            var_fromFixed.extForce{i, 1}.extForce = ...
                var_fromFixed.extForce{i, 1}.extForce(1,range_cut_plot);
            var_fromFloating.estimated_variables.extForce{i+1, 1}.extForce = ...
                var_fromFloating.estimated_variables.extForce{i+1, 1}.extForce(1,range_cut_plot);
        end
        % text plots
        for i = 1 : length(var_fromFixed.extForce)
            fig = figure();
            axes1 = axes('Parent',fig,'FontSize',16);
                      box(axes1,'on');
                      hold(axes1,'on');
                      grid on;
            plot1 = plot(var_fromFloating.estimated_variables.extForce{i+1, 1}.extForce,'b','lineWidth',1.5);
            hold on;
            plot2 = plot(var_fromFixed.extForce{i, 1}.extForce,'r','lineWidth',1.5);
            if i == 18
            hold on;
                plot3 = plot(measFext.measuredForces.righFoot(1,:),'m','lineWidth',1.5);
            else
                plot3 = plot(zeros(size(measFext.measuredForces.righFoot(1,:))),'m','lineWidth',1.5);
            end
          
            ylabel('Force x [N]','HorizontalAlignment','center',...
               'FontWeight','bold',...
               'FontSize',18,...
               'Interpreter','latex');
           xlabel('N samples');
           title(sprintf('%s', var_fromFixed.extForce{i, 1}.label), 'Interpreter', 'none');

           leg = legend([plot1,plot2,plot3],{'floating','fixed','meas'});
           set(leg,'FontSize',13);
        end

        % if the base is LeftFoot
        var_fromFloating.estimated_variables.extForce{1, 1}.extForce = ...
                var_fromFloating.estimated_variables.extForce{1, 1}.extForce(3,range_cut_plot);
        fig = figure();
            axes1 = axes('Parent',fig,'FontSize',16);
                      box(axes1,'on');
                      hold(axes1,'on');
                      grid on;
        plot1 = plot(var_fromFloating.estimated_variables.extForce{1, 1}.extForce,'b','lineWidth',1.5);
        ylabel('Force z [N]','HorizontalAlignment','center',...
               'FontWeight','bold',...
               'FontSize',18,...
               'Interpreter','latex');
        xlabel('N samples');
        title('LeftFoot'); 
        leg = legend([plot1],{'floating'});
        set(leg,'FontSize',13);

    else % i.e., RighFoot is the base --> tappullo!
        % I know that the RightFoot is the 19th in the list, so I manually
        % remove it!
        rightFoot_base = var_fromFloating.estimated_variables.extForce{19, 1};
        var_fromFloating.estimated_variables.extForce{19, 1} = [];
        var_fromFloating.estimated_variables.extForce = ...
            var_fromFloating.estimated_variables.extForce(~cellfun('isempty',var_fromFloating.estimated_variables.extForce));

        % cutting data
        rightFoot_base = rightFoot_base.extForce(3,range_cut_plot);
        for i = 1 : length(var_fromFixed.extForce)
            var_fromFixed.extForce{i, 1}.extForce = ...
                var_fromFixed.extForce{i, 1}.extForce(3,range_cut_plot);
            var_fromFloating.estimated_variables.extForce{i, 1}.extForce = ...
                var_fromFloating.estimated_variables.extForce{i, 1}.extForce(3,range_cut_plot);
        end

       % text plots
        for i = 1 : length(var_fromFixed.extForce)
            fig = figure();
            axes1 = axes('Parent',fig,'FontSize',16);
                      box(axes1,'on');
                      hold(axes1,'on');
                      grid on;
            plot1 = plot(var_fromFloating.estimated_variables.extForce{i, 1}.extForce,'b','lineWidth',1.5);
            hold on;
            plot2 = plot(var_fromFixed.extForce{i, 1}.extForce,'r','lineWidth',1.5);
            if i == 1
            hold on;
                plot3 = plot(measFext.measuredForces.leftFoot(3,:),'m','lineWidth',1.5);
            else
                plot3 = plot(zeros(size(measFext.measuredForces.leftFoot(3,:))),'m','lineWidth',1.5);
            end
            
            ylabel('Force z [N]','HorizontalAlignment','center',...
               'FontWeight','bold',...
               'FontSize',18,...
               'Interpreter','latex');
           xlabel('N samples');
           title(sprintf('%s', var_fromFixed.extForce{i, 1}.label), 'Interpreter', 'none');

           leg = legend([plot1,plot2,plot3],{'floating','fixed','meas'});
           set(leg,'FontSize',13);
        end

        % if the base is RightFoot
        fig = figure();
            axes1 = axes('Parent',fig,'FontSize',16);
                      box(axes1,'on');
                      hold(axes1,'on');
                      grid on;
        plot1 = plot(rightFoot_base,'b','lineWidth',1.5);
        ylabel('Force y [N]','HorizontalAlignment','center',...
               'FontWeight','bold',...
               'FontSize',18,...
               'Interpreter','latex');
        xlabel('N samples');
        title('RightFoot'); 
        leg = legend([plot1],{'floating'});
        set(leg,'FontSize',13);
    end
end
