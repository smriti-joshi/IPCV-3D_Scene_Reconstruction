[F,inliersIndex] = estimateFundamentalMatrix(matchedPoints{1},matchedPoints{2}, 'Method','RANSAC',...
    'NumTrials',2000,'DistanceThreshold',1);

addpath(genpath('./vgg_ui/'));
addpath(genpath('./vgg_multiview/'));
addpath(genpath('./vgg_numerics/'));
addpath(genpath('./vgg_general/'));

vgg_gui_F(ima{1}, ima{2}, F')
fprintf('Number of inliers: %d\n', sum(inliersIndex))