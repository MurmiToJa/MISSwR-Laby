load('offlineSlamData.mat');
maxLidarRange = 8;
mapResolution = 20;
slamAlg = lidarSLAM(mapResolution, maxLidarRange);
slamAlg.LoopClosureThreshold = 210;  
slamAlg.LoopClosureSearchRadius = 8;
for i=1:10
    [isScanAccepted, loopClosureInfo, optimizationInfo] = addScan(slamAlg, scans{i});
    if isScanAccepted
        fprintf('Added scan %d \n', i);
    end
end

figure;
show(slamAlg);
title({'Map of the Environment','Pose Graph for Initial 10 Scans'});

figure;
show(slamAlg);
title({'Map of the Environment','Pose Graph for Initial 10 Scans'});


igure
show(slamAlg);
title({'Final Built Map of the Environment', 'Trajectory of the Robot'});

[scans, optimizedPoses]  = scansAndPoses(slamAlg);
map = buildMap(scans, optimizedPoses, mapResolution, maxLidarRange);

igure; 
show(map);
hold on
show(slamAlg.PoseGraph, 'IDs', 'off');
hold off
title('Occupancy Grid Map Built Using Lidar SLAM');