%A custom Matlab programming style hinter
%function varargout=matlabStyleHinter(fileName,varargin)
%The goal of the hinter is to provide suggestions to make Matlab programs
%more legible and robust (the latter by avoiding common pitfalls).
%Most of the guidelines are derived from the "MATLAB Style Guidelines 2.0",
%by Richard Johnson [1], plus others identified by the author (Roberto
%Tron). The hinter can be somewhat pedantic. Moreover, it uses a simple
%regex-based rules, so false positives are possible. In addition, if
%available, the program also runs the official Matlab hinter.
%
%Inputs
%   fileName    Name of the file to analyze
%Optional inputs
%   'diary'     If the OUTPUT argument is not assigned, tee the terminal
%               output to the file diary.txt
%Outputs
%   nbItems     Number of issues found
%   output      A cell array of strings containing messages from the
%               hinter. If this output is requested, it disables output to
%               the terminal.
%
%References
%   [1] Richard Johnson (2014). MATLAB Style Guidelines 2.0
%       https://www.mathworks.com/matlabcentral/fileexchange/46056-matlab-style-guidelines-2-0,
%       MATLAB Central File Exchange. Retrieved August 14, 2020.

%TODO: find function "string"
%TODO: string comparison with == instead of strcmp

function varargout=matlabStyleHinter(fileName,varargin)
flagDiary=false;
flagCountItems=nargout>0;
flagNoTerminal=nargout>1;

%autopopulate file names from current directory if not specified
if ~exist('fileName','var') || isempty(fileName)
    dirMFiles=dir('*.m');
    fileName={dirMFiles.name};
end

%optional parameters
ivarargin=1;
while ivarargin<=numel(varargin)
    switch lower(varargin{ivarargin})
        case 'diary'
            flagDiary=true;
        otherwise
            error(['Argument ' varargin{ivarargin} ' not valid!'])
    end
    ivarargin=ivarargin+1;
end

%handle single/multiple files
if iscell(fileName)
    nbItems=0;
    output={};
    %multiple files, recursive call
    for iFile=1:numel(fileName)
        thisFile=fileName{iFile};
        output=log_add(output,['** File: ' thisFile]);
        [nbItemsFile,outputFile]=processSingleFile(thisFile,varargin{:});
        nbItems=nbItems+nbItemsFile;
        output=log_add(output, outputFile);
    end
else
    [nbItems,output]=processSingleFile(fileName,varargin{:});
end

if ~flagNoTerminal
    if flagDiary
        diary diary.txt
        diary on
    end

    log_display(output)
    fprintf('Total items found: %d\n',nbItems)

    if flagDiary
        diary off
    end
else
    varargout{2}=output;
end
if flagCountItems
    varargout{1}=nbItems;
end

function [nbItems,output]=processSingleFile(fileName,varargin)
output={};

%automatically add .m extension if necessary
[~,~,ext]=fileparts(fileName);
if isempty(ext)
    fileName=[fileName '.m'];
end

%list of "actions" that trigger a message.
%Each "action" is a [1x2] or [1x3] cell array.
%action{1} is a regexp. If a match if found, a warning is triggered.
%action{2} is either a message to display or a function to call when a
%   warning is triggered
%action{3}, if present, is a struct of flags for controlling the
%   normalization of lines prior to the regexp match. For instance, to
%   check for particular string literals, we should not remove string
%   literals, while in general they should be ignored.
actionList={
    {'''(true|false)''','Use the logical type variables true and false, not strings (type `doc logical` and links therein for more information).',struct('removeString',false)}
    {';.*;','Do not insert multiple commands on the same line.',struct('removeArrays',true)}
    {'hold (on|off);','hold on and hold off commands do not need the trailing semicolon.'}
    {'\<(input|keyboard)\s*\(','All functions and tests should not require direct input from the user. If it is a function, use only its arguments. If it is a test, the data should be automatically generated (preferably in a random way).'}
    {'\<eval\s*\(','Avoid the use of eval.'}
    {'if\s*\([^&|]*\)', 'In Matlab, the if statement syntax does not require parentheses (i.e., use `if iCell==1` instead of `if (iCell==1)`). If instead you need to combine multiple boolean expressions (e.g., `if (a==1) && (b==2)`), it is more clear to use an intermediate flag (e.g., `flagIsABOrdered=(a==1) && (b==2); if flagIsABOrdered ...'}
    {'\<[a-zA-Z]\w?\s*[\(|=]', @isIdentifierTooShort} 
    {'for\s+[^ijk]\w*\s*=', 'Prefix iterator variable names with i, j, k etc. (e.g., instead of `for cell=1:5` use `for iCell=1:nbCells`).'}
    {'\<(quiver|figure|plot|alpha|angle|axes|axis|balance|beta|contrast|gamma|image|info|input|length|line|mode|power|rank|run|start|text|type)\>[\s\w,\]]*=','Avoid variable names that shadow functions'}
    {'\<global\>','Do not use global variables.'}
    {'~\(','This type of logical negation can be usually avoided by reversing the condition (e.g., `if ~(i==1)` should be changed to `if i~=1`)'}
    {'\<(?:sym|syms|solve)\>',@(output,context,idx) aFileNameContains(output,'sym',['Do not use symbolic variables for standard computations. ' ...
        'You can use the symbolic toolbox to derive expressions, but then write those expressions as standard Matlab functions. '  ...
        'For scripts/functions that perform the derivation, include the word `Sym` in the file name.'],context,idx)}
    {'\<(?:figure|plot?|quiver?)\>',@(output,context,idx) aFileNameContains(output,'(?:test|plot)','Display figures only in test or plot functions (i.e.,the file name should contain `test` or `plot`).',context,idx)}
    {'((&&|\|\|)\S|\S(&&|\|\|))','Surround && and || by spaces.'}
    {'\<(for|while|function|global|switch|try|if|elseif)[{\(]','Follow MATLAB keywords by spaces.',struct('caseInsensitive',false)}
    {'pinv\s*\([\w_\:\(\)]*\)\s*\*','Use the backslash operator instead of multiplying pinv() with another vector or matrix.'}
    {'\*\s*pinv\s*\(','Use the slash operator instead of multiplying by pinv().'}
    {'\<length\s*\(','Warning: the function length() when called on a 2-D array behaves differently depending on whether the array contains a single column or not. Consider using size() or numel() instead.'}
    {'%#ok','Do not suppress warnings from the MATLAB''s linter',struct('keepComments',false)}
    };

fid=fopen(fileName,'rt');
if fid<0
    error('File %s not found', fileName);
end

%add file name and identifier to context struct
context.fileName=fileName;
context.fid=fid;

%run our custom regexp-based tests
output=log_add(output,'* Programming style report');
[nbItems,output]=actionCheck(context,actionList,output);

%check if Matlab's code analyzer is available
if ~isempty(which('checkcode'))
    %run Matlab's standard checks
    output=log_add(output,'* Matlab Code Analyzer report');
    [nbItemsMatlab,output]=matlabCheck(context,output);
    nbItems=nbItems+nbItemsMatlab;
end



function [nbItems,output]=matlabCheck(context,output)
%Show report from Matlab Code Analyzer
report=checkcode(context.fileName);
nbItems=numel(report);
if isempty(report)
    output=[output; '    No problems found'];
else
    for iEntry=1:nbItems
        entry=report(iEntry);
        output=log_printf(output,'Line %d, column %d: %s',...
            entry.line, entry.column(1),entry.message);
    end
end

function [nbItems,output]=actionCheck(context,actionList,output)
%Show report on custom regexp style specifications
nbActions=numel(actionList);

nbItems=0;
flagContinueReading=true;
context.flagAnyErrorFound=false;
context.cnt=1;
while flagContinueReading
    lineStr=fgetl(context.fid);
    context.lineStr=lineStr;
    if isnumeric(lineStr) && lineStr==-1
        flagContinueReading=false;
    else
        for iAction=1:nbActions
            action=actionList{iAction};
            %set default normalization option if necessary
            if numel(action)<3
                action{3}=[];
            end
            %normalize line and add it to context
            lineStrNorm=lineNormalization(lineStr,action{3});
            context.lineStrNorm=lineStrNorm;
            %look for regexp pattern
            idx=regexp(lineStrNorm,action{1},'once');
            flag=false;
            if ~isempty(idx)
                if actionHasFunction(action)
                    %Action has a custom function, call it
                    [flag,output]=action{2}(output,context,idx);
                    context.flagAnyErrorFound=context.flagAnyErrorFound|flag;
                else
                    %By default, just show message
                    flag=true;
                    output=log_lineWithMarker(output,context,action{2},idx);
                    context.flagAnyErrorFound=true;
                end
            end
            if flag
                nbItems=nbItems+1;
            end
        end
    end
    context.cnt=context.cnt+1;
end

if ~context.flagAnyErrorFound
    output=log_add(output,'    No problems found');
end


function lineStr=lineNormalization(lineStr,flags)
%remove comments
if ~isfield(flags,'keepComments') || flags.keepComments
    lineStr=regexprep(lineStr,'%.*','');
end
%normalize case
if ~isfield(flags,'caseInsensitive') || flags.caseInsensitive
    lineStr=lower(lineStr);
end
if ~isfield(flags,'removeString') || flags.removeString
    %remove tick (') characters from strings, and empty strings (i.e., double
    %ticks)
    lineStr=regexprep(lineStr,'''{2}','');
    %remove string literals
    lineStr=regexprep(lineStr,'''[^'']*''','');
end
if isfield(flags,'removeArrays') && flags.removeArrays
    lineStr=regexprep(lineStr,'\[[^\]]*(\]|$)','');
end

function flag=actionHasFunction(action)
flag=isa(action{2}, 'function_handle');

function [flag,output]=isIdentifierTooShort(output,context,idx)
msg='Avoid the use of very short identifiers (variable or function names). E.g., instead of `for i=1:l`, use `for iCell=1:nbCells`.';
%check that the regexp did not find "if" as an identifier
flagIsIf=strcmpi(context.lineStrNorm(idx:idx+1),'if');
flagIsField= idx~=1 && context.lineStrNorm(idx-1)=='.';
flag=~flagIsIf && ~flagIsField;
if flag
    output=log_lineWithMarker(output,context,msg,idx);
end

function [flagNameMissingStr,output]=aFileNameContains(output,strName,msg,context,idx)
flagNameMissingStr=isempty(regexp(lower(context.fileName),strName,'once'));
if flagNameMissingStr
    %By default, just show message
    indent='    ';
    output=log_printf(output,'Line %d: %s\n',context.cnt,msg);
    output=log_regexpDisplay(output,context.lineStr,idx,indent);
end

%Logging functions
%Display a string together with regular expression matches
function output=log_regexpDisplay(output,str,expression,indent)
if ~exist('indent','var')
    indent='';
end

if isnumeric(expression)
    idx=expression;
else
    idx=regexp(str,expression);
end

if isempty(idx)
    output=log_add(output,'No matches');
else
    indicators=repmat(' ',1,numel(str));
    indicators(idx)='^';
    output=log_add(output,[indent str]);
    output=log_printf(output,'%s%s',indent,indicators);
end

function output=log_lineWithMarker(output,context,msg,idx)
%Log error message and also echoes the line with a marker at the found
%position
indent='    ';
output=log_printf(output, 'Line %d: %s',context.cnt,msg);
output=log_regexpDisplay(output,context.lineStr,idx,indent);

function output=log_printf(output,varargin)
output=log_add(output,sprintf(varargin{:}));

function output=log_add(output,s)
output=[output;s];

function log_display(output)
for iLine=1:numel(output)
    if is_matlab()
        fprintfWrap('%s\n',output{iLine})
    else
        fprintf('%s\n',output{iLine})
    end
end

function fprintfWrap(varargin)
%Get string equivalent to fprintf output
%TODO: the regex for wrapping does not work on Octave
str=sprintf(varargin{:});
%Get command window size
cmsz = get(0,'CommandWindowSize'); 
width = cmsz(1);
if width<=0
    %This happens for Octave
    %Set to a arbitrary number
    width=160;
end
%Wrap string
strWrap=regexprep(str,['.{1,' num2str(width) '}\s'],'$0\n');
fprintf(strWrap(1:end-1))
