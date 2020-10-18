clc;
clear all;

%Own camera calibration
[coords ima_pattern]= get_real_points_checkerboard_vmmc(9,128,1);

im1 = imread('D:/Work/IPCV/Madrid/Moving cameras/Lab/Lab Evaluation/720/im1.jpg');
im2 = imread('D:/Work/IPCV/Madrid/Moving cameras/Lab/Lab Evaluation/720/im2.jpg');
im3 = imread('D:/Work/IPCV/Madrid/Moving cameras/Lab/Lab Evaluation/720/im3.jpg');
im4 = imread('D:/Work/IPCV/Madrid/Moving cameras/Lab/Lab Evaluation/720/im4.jpg');
im5 = imread('D:/Work/IPCV/Madrid/Moving cameras/Lab/Lab Evaluation/720/im5.jpg');
im6 = imread('D:/Work/IPCV/Madrid/Moving cameras/Lab/Lab Evaluation/720/im6.jpg');
im7 = imread('D:/Work/IPCV/Madrid/Moving cameras/Lab/Lab Evaluation/720/im7.jpg');
im8 = imread('D:/Work/IPCV/Madrid/Moving cameras/Lab/Lab Evaluation/720/im8.jpg');
im9 = imread('D:/Work/IPCV/Madrid/Moving cameras/Lab/Lab Evaluation/720/im9.jpg');


%point1
% points1 = get_user_points_vmmc(im1);
homo_1 = homography_solve_vmmc(coords',points1(:,1:9));
[homo_1_ref, r1] = homography_refine_vmmc(coords',points1(:,1:9), homo_1);
t1form = maketform('projective', homo_1_ref');
J_homo1 = imtransform(ima_pattern,t1form,'XData',[1 size(im1,2)], 'YData',[1 size(im1,1)]);

%point2
% points2 = get_user_points_vmmc(im2);
homo_2 = homography_solve_vmmc(coords',points2(:,1:9));
[homo_2_ref, r2] = homography_refine_vmmc(coords',points2(:,1:9), homo_2);
t2form = maketform('projective', homo_2_ref');
J_homo2 = imtransform(ima_pattern,t2form,'XData',[1 size(im2,2)], 'YData',[1 size(im2,2)]);

%point3
% points3 = get_user_points_vmmc(im3);
homo_3 = homography_solve_vmmc(coords',points3(:,1:9));
[homo_3_ref, r3] = homography_refine_vmmc(coords',points3(:,1:9), homo_3);
t3form = maketform('projective', homo_3_ref');
J_homo3 = imtransform(ima_pattern,t3form,'XData',[1 size(im3,2)], 'YData',[1 size(im3,2)]);


%point4
% points4 = get_user_points_vmmc(im4);
homo_4 = homography_solve_vmmc(coords',points4(:,1:9));
[homo_4_ref, r4] = homography_refine_vmmc(coords',points4(:,1:9), homo_4);
t4form = maketform('projective', homo_4_ref');
J_homo4 = imtransform(ima_pattern,t4form,'XData',[1 size(im4,2)], 'YData',[1 size(im4,2)]);


%point5
% points5 = get_user_points_vmmc(im5);
homo_5 = homography_solve_vmmc(coords',points5(:,1:9));
[homo_5_ref, r5] = homography_refine_vmmc(coords',points5(:,1:9), homo_5);
t5form = maketform('projective', homo_5_ref');
J_homo5 = imtransform(ima_pattern,t5form,'XData',[1 size(im5,2)], 'YData',[1 size(im5,2)]);


%points6
% points6 = get_user_points_vmmc(im6);
homo_6 = homography_solve_vmmc(coords',points6(:,1:9));
[homo_6_ref, r6] = homography_refine_vmmc(coords',points6(:,1:9), homo_6);
t6form = maketform('projective', homo_6_ref');
J_homo6 = imtransform(ima_pattern,t6form,'XData',[1 size(im6,2)], 'YData',[1 size(im6,2)]);

%points7
% points7 = get_user_points_vmmc(im7);
homo_7 = homography_solve_vmmc(coords',points7(:,1:9));
[homo_7_ref, r7] = homography_refine_vmmc(coords',points7(:,1:9), homo_7);
t7form = maketform('projective', homo_7_ref');
J_homo7 = imtransform(ima_pattern,t7form,'XData',[1 size(im7,2)], 'YData',[1 size(im7,2)]);


%points8
% points8 = get_user_points_vmmc(im8);
homo_8 = homography_solve_vmmc(coords',points8(:,1:9));
[homo_8_ref, r8] = homography_refine_vmmc(coords',points8(:,1:9), homo_8);
t8form = maketform('projective', homo_8_ref');
J_homo8 = imtransform(ima_pattern,t8form,'XData',[1 size(im8,2)], 'YData',[1 size(im8,2)]);


%points9
% points9 = get_user_points_vmmc(im9);
homo_9 = homography_solve_vmmc(coords',points9(:,1:9));
[homo_9_ref, r9] = homography_refine_vmmc(coords',points9(:,1:9), homo_9);
t9form = maketform('projective', homo_9_ref');
J_homo9 = imtransform(ima_pattern,t9form,'XData',[1 size(im9,2)], 'YData',[1 size(im9,2)]);


%Plotting 
sgtitle('Homography with Zhangs Method'),subplot(9,2,1), imshow(im1);subplot(9,2,2), imshow(J_homo1)
subplot(9,2,3), imshow(im2);subplot(9,2,4), imshow(J_homo2);
subplot(9,2,5), imshow(im3); subplot(9,2,6), imshow(J_homo3);
subplot(9,2,7), imshow(im4);subplot(9,2,8), imshow(J_homo4);
subplot(9,2,9), imshow(im5);subplot(9,2,10), imshow(J_homo5);
subplot(9,2,11), imshow(im6);subplot(9,2,12), imshow(J_homo6);
subplot(9,2,13), imshow(im7);subplot(9,2,14), imshow(J_homo7);
subplot(9,2,15), imshow(im8);subplot(9,2,16), imshow(J_homo8);
subplot(9,2,17), imshow(im9);subplot(9,2,18), imshow(J_homo9);


%Calculation of internal parameters
H_ref = {homo_1_ref,homo_2_ref,homo_3_ref,homo_4_ref,homo_5_ref,homo_6_ref,homo_7_ref,homo_8_ref,homo_9_ref};
A_ref = internal_parameters_solve_vmmc(H_ref);