%% gcalculate target accelerate



function acc = GetTargetAcc(delta_s, delta_v)
    %delta_s  si - s_target 都是相对于前车的距离
    %delta_v vi - v_target
    sv_rear1_alpha = 0.05;
    sv_rear1_kesai = -0.12;

    sv_rear2_alpha = 0.01;
    sv_rear2_kesai = -0.06;

    sv_rear3_alpha = -0.05;
    sv_rear3_kesai = 0.1;

    if delta_s > 0 && delta_v > 0
        acc = max(-delta_v^2/(2 * delta_s)*0.4, -1.5);
    elseif delta_s <= 0 && delta_v >= 0
        acc = max(delta_s * sv_rear1_alpha + delta_v * sv_rear1_kesai, -1.5);
    elseif delta_s >= 0 && delta_v <= 0
        acc = delta_s * sv_rear2_alpha + delta_v * sv_rear2_kesai;
    else
        acc = max(min(delta_s * sv_rear3_alpha + delta_v * sv_rear3_kesai, 1.0), -1.5);
    end
end

