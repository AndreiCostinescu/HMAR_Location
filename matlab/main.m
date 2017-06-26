clean;

res_dir_name = ...
    {'../Result_Staticface'
     '../Result_Moveface'
     '../Result_ObjectState'};

fig_file_name = ...
    {'ConfusionMatrix/Result_Staticface'
     'ConfusionMatrix/Result_Moveface'
     'ConfusionMatrix/Result_ObjectState'};
 
% for i=1:3
%     ConfusionMatrix(res_dir_name{i},fig_file_name{i});
% end

out_file_name = ...
    {'TrajectoryPrediction/Result_Staticface/GEdgeProb'
     'TrajectoryPrediction/Result_Moveface/GEdgeProb'
     'TrajectoryPrediction/Result_ObjectState/GEdgeProb'};
 
for i=1:3
    PredictTrajectory(res_dir_name{i},out_file_name{i});
end

