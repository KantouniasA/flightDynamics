function fixpath(topLevelFolder)

if nargin == 0
    topLevelFolder = pwd; % or whatever, such as 'C:\Users\John\Documents\MATLAB\work'
end
% Get a list of all files and folders in this folder.
files = dir(topLevelFolder);

% Get a logical vector that tells which is a directory.
dirFlags = [files.isdir];

% Extract only those that are directories.
subFolders = files(dirFlags); % A structure with extra info.

% Keep only the real folders
subfolderNamesInitial   = {subFolders.name};
subfolderValidFlag      = [];
isubfolderValidFlag  	= 0;
for iSubFolder = 1:length(subfolderNamesInitial)
    if ~any(strcmp(subfolderNamesInitial{iSubFolder},{'.git','..','.'}))
        isubfolderValidFlag = isubfolderValidFlag + 1;
        subfolderValidFlag(isubfolderValidFlag) = iSubFolder;
    end
end
subfolderNames =subfolderNamesInitial(subfolderValidFlag);

% Get only the folder names into a cell array.
if ~isempty(subfolderNames) 
    for iFolder = 1:length(subfolderNames)
        fixpath([topLevelFolder,'\',subfolderNames{iFolder}])
    end
    addpath(topLevelFolder)
    fprintf(['Added: ',replace(topLevelFolder,'\','\\'), '\n\n']);
else
    addpath(topLevelFolder);
    fprintf(['Added: ',replace(topLevelFolder,'\','\\'), '\n\n']);
end

end