function dx = discretely_PD_controlled_model_with_obs(state_collection, disturbances)
    dx = discretely_controlled_model_with_observer(state_collection, disturbances, @Yaw_Rate_Controller);
end
