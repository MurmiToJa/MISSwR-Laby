
% 
% %% SLAM
% %% SLAM Implementation
% %% SLAM Implementation
% % This code assumes you've already run the simulation and have:
% % - scanBuffer: Cell array of point clouds
% % - robotPoses: Array of robot poses [x, y, orientation]
% % - timestamps: Array of timestamps for each scan
% 
% disp("Starting SLAM processing...");
% 
% % Check if we have data to process
% if isempty(scanBuffer) || isempty(robotPoses) || isempty(timestamps)
%     error("No data available for SLAM. Make sure simulation collected data properly.");
% end
% 
% % Create a lidarSLAM object
% maxLidarRange = 15;  % Maximum range in meters
% mapResolution = 20;  % Cells per meter
% slamAlg = lidarSLAM(mapResolution, maxLidarRange);
% 
% % Set SLAM algorithm properties
% slamAlg.LoopClosureThreshold = 210;  
% slamAlg.LoopClosureSearchRadius = 8;
% 
% % Process scans for SLAM
% numScans = length(scanBuffer);
% disp("Processing " + numScans + " scans for SLAM...");
% 
% % Create an empty occupancy map with the specified resolution
% % The correct way to specify just resolution:
% omap = occupancyMap(10, 10, mapResolution);  % Create 10m x 10m map with resolution of 20 cells/meter
% 
% % Process each scan
% for i = 1:numScans
%     % Get the current point cloud
%     currentPC = scanBuffer{i};
% 
%     % Skip if point cloud is empty or contains only NaNs
%     if isempty(currentPC) || all(isnan(currentPC.Location(:)))
%         disp("Skipping empty or invalid point cloud at index: " + i);
%         continue;
%     end
% 
%     % Project 3D point cloud to 2D for SLAM
%     pcXYZ = currentPC.Location;
%     validPointsIdx = ~isnan(pcXYZ(:,1)) & ~isnan(pcXYZ(:,2)) & ~isnan(pcXYZ(:,3));
%     pcXYZ = pcXYZ(validPointsIdx, :);
% 
%     % Skip if no valid points after removing NaNs
%     if isempty(pcXYZ)
%         disp("No valid points in scan " + i + ". Skipping.");
%         continue;
%     end
% 
%     % Convert to 2D lidar scan format (polar coordinates)
%     angles = atan2(pcXYZ(:,2), pcXYZ(:,1));
%     ranges = sqrt(pcXYZ(:,1).^2 + pcXYZ(:,2).^2);
% 
%     % Sort by angle to ensure the scan is in order
%     [angles, sortIdx] = sort(angles);
%     ranges = ranges(sortIdx);
% 
%     % Create a lidarScan object
%     scan = lidarScan(ranges, angles);
% 
%     % Add scan to SLAM algorithm with the corresponding robot pose
%     addScan(slamAlg, scan, robotPoses(i,:));
% 
%     % Update progress
%     if mod(i, 10) == 0
%         disp("Processed " + i + " out of " + numScans + " scans");
%     end
% end
% 
% %% Print
% 
% % After processing all scans, show the built map
% disp("SLAM processing completed. Displaying results...");
% 
% % Get the occupancy map from the SLAM algorithm
% [scans, optimizedPoses] = scansAndPoses(slamAlg);
% occMap = buildMap(scans, optimizedPoses, mapResolution, maxLidarRange);
% 
% % Display the built occupancy map
% figure('Name', 'SLAM Map');
% show(occMap);
% title('SLAM Generated Occupancy Map');
% hold on;
% 
% % Plot the optimized robot trajectory
% plot(optimizedPoses(:,1), optimizedPoses(:,2), 'r-', 'LineWidth', 2);
% plot(optimizedPoses(1,1), optimizedPoses(1,2), 'go', 'MarkerSize', 10, 'LineWidth', 2);  % Start position
% plot(optimizedPoses(end,1), optimizedPoses(end,2), 'mo', 'MarkerSize', 10, 'LineWidth', 2);  % End position
% legend('Robot Trajectory', 'Start Position', 'End Position');
% 
% hold off;