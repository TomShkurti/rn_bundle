%%
clc
close all
clear all

%%
load('sample_pts.mat');
load('sample_boundary_pts.mat');

sample_pts_centre = [-0.0836266 -0.00382424   -0.134643]

coeff = pca(sample_pts)




%%
figure('Name','points');

scatter3(sample_pts(:,1),sample_pts(:,2),sample_pts(:,3),'filled','red');
hold on;
scatter3(sample_pts_centre(:,1), sample_pts_centre(:,2), sample_pts_centre(:,3),'filled','green');
scatter3(sample_boundary_pts(:,1), sample_boundary_pts(:,2), sample_boundary_pts(:,3), 'blue', '.');
quiver3(sample_pts_centre(:,1), sample_pts_centre(:,2), sample_pts_centre(:,3),...
    coeff(1,1),coeff(2,1),coeff(3,1),0.2);

quiver3(sample_pts_centre(:,1), sample_pts_centre(:,2), sample_pts_centre(:,3),...
    coeff(1,2),coeff(2,2),coeff(3,2),0.2)

quiver3(sample_pts_centre(:,1), sample_pts_centre(:,2), sample_pts_centre(:,3),...
    coeff(1,3),coeff(2,3),coeff(3,3),0.2)

hold off;
axis equal;