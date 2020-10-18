clc;
clear all;

addpath(genpath('D:\Work\IPCV\Madrid\Moving cameras\Lab\Lab Evaluation\Section 2'));
ACT_path = 'D:/Work/IPCV/Madrid/Moving cameras/Lab/LabAssignment6/ACT_lite';
addpath(genpath(ACT_path));
% include extra funs
extra_funs_path = 'D:/Work/IPCV/Madrid/Moving cameras/Lab/LabAssignment6/extra_funs';
addpath(genpath(extra_funs_path));

%load variables of section 2
load('Section2.mat');
clear('disp')

%point matches
q_data = n_view_matching(points, features, ima, 0.4, params.Metric, params.MatchThreshold);
q_data_new = homogenize_coords(q_data);

% Fundamental Matrix Computation

ncam = size(q_data, 3);                 %Number of camera
q2_cams = zeros(3, size(q_data_new,2),2);
q2_cams(:,:,1) = q_data_new(:,:,1);     %Points from the first camera
q2_cams(:,:,2) = q_data_new(:,:,ncam);  %Points from the last camera

        
[F, P_2cam,Q_2cam,q_2cam_est] = MatFunProjectiveCalib(q2_cams);%Fundamental matri, Projection matrix, 
                                                                %3D points, reprojected points points
% disp(['Projective Reconstruction from 2 cameras; reprojection error  = ' num2str( ErrorRetroproy(q2_cams,P_2cam,Q_2cam)/2 )]);
%draw_reproj_error(q2_cams,P_2cam,Q_2cam);

%Resectioning Step
P_rep = zeros(3,4,ncam);
P_rep(:,:,[1 ncam]) = P_2cam;
for i = 2:ncam-1
    P_rep(:,:,i) =  PDLT_NA(q_data_new(:,:,i), Q_2cam);
end

disp(['Reprojection error after resectioning  = ' num2str( ErrorRetroproy(q_data_new,P_rep,Q_2cam)/2)]);
draw_reproj_error(q2_cams,P_2cam,Q_2cam);

%Bundle Adjustment
npoints = size(q_data_new, 2);
vp = ones(npoints,ncam);
[P_ba,Q_ba] = BAProjectiveCalib(q_data_new,P_rep,Q_2cam,vp);

disp(['Reprojection error for improve projective reconstruction = ' num2str( ErrorRetroproy(q_data_new,P_ba,Q_ba)/2 )]);
draw_reproj_error(q_data_new,P_ba,Q_ba);

%Recomputation of fundamental matrix
F = vgg_F_from_P(P_ba(:,:,1), P_ba(:,:,2));

%Reconstruction euclidean
K = zeros(3,3,2);
% K_new = 0.5 * [3024.07972050313,-19.1707323282337,2060.43740902577;0,3037.06741274178,1538.06134998808;0,0,1/0.5];
K_new = [ 2997.22154756109,-33.2089218773851,2031.13399279995;0,2997.50444028413,1503.37439437853;0,0,1];
K(:,:,1) = K_new;
K(:,:,2) = K_new;
E = K(:,:,1)'*F*K(:,:,2);

% ------------------------------------------------------------------------
% 4. Factorize the essential matrix with the 2 possible solutions for
% R. Use the function factorize_E to obtain R_est(:,:,1) and R_est(:,:,2) and T_est.
% ------------------------------------------------------------------------
[R_est,T_est] = factorize_E(E);

% ------------------------------------------------------------------------
% Save the 4 solutions (R,t) in the structures Rcam(3,3,cam,sol), T(3,cam,sol),
% where cam indicates the camera number and sol indicates the solution number (1, 2, 3 or 4).
% ------------------------------------------------------------------------
Rcam = zeros(3,3,2,4);
Tcam = zeros(3,2,4);
% ...
%camera 1
Rcam(:,:,1,1) = eye(3,3);
Rcam(:,:,1,2) = eye(3,3);
Rcam(:,:,1,3) = eye(3,3);
Rcam(:,:,1,4) = eye(3,3);

null_translation = [0;0;0];
Tcam(:,1,1) = null_translation;
Tcam(:,1,2) = -null_translation;
Tcam(:,1,3) = null_translation;
Tcam(:,1,4) = -null_translation;

%camera 2

Rcam(:,:,2,1) = R_est(:,:,1);
Rcam(:,:,2,2) = R_est(:,:,1);
Rcam(:,:,2,3) = R_est(:,:,2);
Rcam(:,:,2,4) = R_est(:,:,2);

Tcam(:,2,1) = T_est;
Tcam(:,2,2) = -T_est;
Tcam(:,2,3) = T_est;
Tcam(:,2,4) = -T_est;


% ------------------------------------------------------------------------
% 5. For each solution we obtain an Euclidean solution and we visualize it.
% ------------------------------------------------------------------------
npoints = size(q_data_new,2);
Q_euc = zeros(4,npoints,2); % Variable for recontructed points
P_euc = zeros(3,4,2);       % Variable for projection matrices
figNo=5;

q_new  = project_points(P_ba, Q_ba);
for sol=1:4
    % Euclidean triangulation to obtain the 3D points (use TriangEuc)

    Q_euc = TriangEuc(Rcam(:,:,2,sol),Tcam(:,2,sol),K,q_new); 
%     Q_euc = un_homogenize_coords(Q_euc);
    % visualize 3D reconstruction
    %figure(4);
    %subplot(2,2,sol),
    figure();
    draw_scene(Q_euc, K, Rcam(:,:,:,sol), Tcam(:,:,sol));
    title(sprintf('Solution %d', sol));
    
    % Compute the projection matrices from K, Rcam, Tcam
    for k=1:2
        P_euc(:,:,k) =K(:,:,k) * [Rcam(:,:,k,sol), -Rcam(:,:,k,sol) * Tcam(:,k,sol)];
   end
    
    % Obtain the re-projected points q_rep
    q_rep = zeros(3,npoints,2) ;
    for k=1:2
        q_rep(:,:,k) =  P_euc(:,:,k)*Q_euc;
    end
    
    % Visualize reprojectd points to check that all solutions correspond to
    % the projected images
    q_rep = un_homogenize_coords(q_rep);
    for k=1:2
      figure(figNo); subplot(4,2,2*(sol-1)+k); scatter(q_rep(1,:,k),q_rep(2,:,k),30,[1,0,0]);
      title(sprintf('Reprojection %d, image %d', sol, k));
      daspect([1, 1, 1]);
      pbaspect([1, 1, 1]);
      axis([-1000, 1000, -1000, 1000]);
    end
end

disp('***************** ******************** END')
