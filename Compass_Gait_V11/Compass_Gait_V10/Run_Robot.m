%MEAM517
%Implement 
%By Greg Campbell

function Run_Robot(t)
    %takes t - how long it runs
    [~,x0] = find_limit_cycle;
    [t,x] = simulate_compassgait(x0(:,1),t);
    plot_compassgait_trajectory(t,x);
end