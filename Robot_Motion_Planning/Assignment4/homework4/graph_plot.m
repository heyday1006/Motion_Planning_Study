%function graph_plot(graphVector)
%Visualizes the contents of graphVector. For each element:
%   Number in a box: index of the vertex in graphVector
%   Black arrows: point toward neighbors
%   Numbers near black arrows: cost associated with that edge
%   Blue numbers: value of the estimated cost in the field g (if present
%       and non-empty)
%   Red arrows: point to neighbor in the backpointer field (if present and
%       non-empty)
function graph_plot(graphVector,varargin)

flagEdgeWeights=false;
flagNodeLabels=false;
flagBackpointers=true;
flagBackpointersCost=false;
idxStyles=struct('start','rx','goal','rd',...
    'closed','bs','neighbors','gx','best','gd','open','ro');
idxFlag=struct();
idxValues=struct();

%optional parameters
ivarargin=1;
while ivarargin<=length(varargin)
    switch lower(varargin{ivarargin})
        case 'nodelabels'
            ivarargin=ivarargin+1;
            flagNodeLabels=varargin{ivarargin};
        case 'edgeweights'
            ivarargin=ivarargin+1;
            flagEdgeWeights=varargin{ivarargin};
        case 'backpointers'
            ivarargin=ivarargin+1;
            flagBackpointers=varargin{ivarargin};
        case 'backpointercosts'
            ivarargin=ivarargin+1;
            flagBackpointersCost=varargin{ivarargin};
        case {'start','goal','best','neighbors','closed'}
            name=lower(varargin{ivarargin});
            idxFlag.(name)=true;
            ivarargin=ivarargin+1;
            idxValues.(name)=varargin{ivarargin};
        case 'pqopen'
            ivarargin=ivarargin+1;
            idxValues.open=[varargin{ivarargin}.key];
            idxFlag.open=true;
        otherwise
            disp(varargin{ivarargin})
            error('Argument not valid!')
    end
    ivarargin=ivarargin+1;
end
%plot backpointers and backpointer costs only if requested and available
flagBackpointers=flagBackpointers & isfield(graphVector,'backpointer');
flagBackpointersCost=flagBackpointersCost & flagBackpointers...
    & isfield(graphVector,'g');

NNodes=length(graphVector);

%coordinates of all nodes
xAll=[graphVector.x];

%index pairs of all edges
neighborsAll=cell2mat(... %concatenate cells contents
    cellfun(@(x,y) [repmat(y,size(x)) x],... %to each cell, stack current node index with each neighbor
    {graphVector.neighbors}',num2cell(1:NNodes)',... %feed neighbors and index for each node
    'UniformOutput',false));

%coordinates and displacements for all edges
xEdgeStart=xAll(:,neighborsAll(:,1));
xEdgeDiff=xAll(:,neighborsAll(:,2))-xEdgeStart;

%edges
if flagNodeLabels
    edgeShortenFactor=0.9;
else
    edgeShortenFactor=1;
end
quiver(xEdgeStart(1,:),xEdgeStart(2,:),...
    edgeShortenFactor*xEdgeDiff(1,:),edgeShortenFactor*xEdgeDiff(2,:),...
    'k','AutoScale',false)
hold on

%node labels
if flagNodeLabels
    text(xAll(1,:),xAll(2,:),num2str((1:NNodes)'),...
        'FontSize',15, 'FontWeight','bold','horizontalAlign','center','BackgroundColor',[1 1 1],'EdgeColor','k')
end

%edge weights
if flagEdgeWeights
    xWeights=xEdgeStart+0.5*xEdgeDiff;
    text(xWeights(1,:),xWeights(2,:),num2str(cat(1,graphVector.neighborsCost),'%.2f'))
end

%backpointers
if flagBackpointers
    flagBackpointerIdx=cellfun(@(x) ~isempty(x),{graphVector.backpointer});
    xBackpointerStart=xAll(:,flagBackpointerIdx);
    idxBackpointer=[graphVector(flagBackpointerIdx).backpointer];
    xBackpointerDiff=0.2*(xAll(:,idxBackpointer)-xBackpointerStart);
    quiver(xBackpointerStart(1,:),xBackpointerStart(2,:),...
        xBackpointerDiff(1,:),xBackpointerDiff(2,:),...
        'b','LineWidth',1,'AutoScale',false)
end

%backpointer costs
if flagBackpointersCost
    flagBackpointerCostIdx=cellfun(@(x) ~isempty(x),{graphVector.g});
    xBackpointerCost=xAll(:,flagBackpointerCostIdx);
    gBackpointerCost=[graphVector(flagBackpointerCostIdx).g];
    text(xBackpointerCost(1,:)+0.1, xBackpointerCost(2,:),...
        num2str(gBackpointerCost','g=%.2f'),...
        'Color','b')
end

%various types of indeces
for nameCell=fieldnames(idxFlag)'
    name=nameCell{1};
    if idxFlag.(name)
        xValues=xAll(:,idxValues.(name));
        plot(xValues(1,:),xValues(2,:),idxStyles.(name),'LineWidth',2,'MarkerSize',8)
    end
end

hold off
axis equal
