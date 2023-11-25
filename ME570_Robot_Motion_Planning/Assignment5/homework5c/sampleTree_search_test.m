%function sampleTree_search_test()
%enumerate  the variables @boxIvory2 world, @boxIvory2 xStart and @boxIvory2 xEnd
%from the file @boxIvory2!70!DarkSeaGreen2 sphereWorld.mat.  each starting
%location (column) in @boxIvory2 xStart, and each goal location (column) in
%@boxIvory2 xGoal, calls sampleTree_search to find a path.  sphereworld_draw and
%then overimposes all the trajectories from the start locations to the goal
%location. enumerate
function sampleTree_search_test()
load('sphereworld')
if exist(fullfile([pwd '/previous']),'dir')
    addpath(fullfile([pwd '/previous']))
end
goalDistThreshold=4;
dataSet=struct();
colorList={'r','m','c','g','b'};
for iGoal=1:size(xGoal,2)
    figure(iGoal)
    xGoalTemp=xGoal(:,iGoal);
    for iStart=1:size(xStart,2)
        subplot(2,3,iStart)
        sphereworld_plot(world,xGoalTemp);
        hold on
        xStartTemp=xStart(:,iStart);
        [xPath,graphVector]=...
            sampleTree_search(world,xStartTemp,xGoalTemp,goalDistThreshold);
        dataSet(iStart,iGoal).xPath=xPath;
        dataSet(iStart,iGoal).graphVector=graphVector;
        dataSet(iStart,iGoal).goal=num2str(iGoal);
        dataSet(iStart,iGoal).start=num2str(iStart);
        graph_plot(graphVector,...
        'nodeLabels',false,...
        'backpointerCosts',false,...
        'backpointers',true,...
        'start',1,'goal',length(dataSet(iStart,iGoal).graphVector))
        hold on
        if size(xPath,2)>2
            plot(xPath(1,:),xPath(2,:),'-o','color',colorList{iStart},...
                    'LineWidth',1.5,...
                    'MarkerSize',2)%,...'MarkerFaceColor',[0.5,0.5,0.5]
        else
            plot([xStartTemp(1) xGoalTemp(1)],[xStartTemp(2) xGoalTemp(2)],...
                '--o','color',colorList{iStart})
        end
        title(['Start ' num2str(iStart)])
        hold off   
    end
    sgtitle(['Goal ' num2str(iGoal)])
    hold off  
end
figure(3)
for iGoal=1:size(xGoal,2)
    subplot(1,2,iGoal)
    xGoalTemp=xGoal(:,iGoal);
    sphereworld_plot(world,xGoalTemp);
	Legend{1}='';
    for iStart=1:size(xStart,2)
        xPath=dataSet(iStart,iGoal).xPath;
        if size(xPath,2)>2
            plot(xPath(1,:),xPath(2,:),'-o','color',colorList{iStart},...
                    'LineWidth',1.5,...
                    'MarkerSize',2)%,...'MarkerFaceColor',[0.5,0.5,0.5]
        else
            plot([xStartTemp(1) xGoalTemp(1)],[xStartTemp(2) xGoalTemp(2)],...
                '--o','color',colorList{iStart})
        end
        hold on
        Legend{iStart+1}=['Goal ' dataSet(iStart,iGoal).goal ...
            ' Start ' dataSet(iStart,iGoal).start];
    end
    legend(Legend);
    title(['Goal ' num2str(iGoal)])
    hold off
end

