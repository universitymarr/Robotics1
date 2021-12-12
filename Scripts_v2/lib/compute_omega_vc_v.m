function [omegai_i, vi_ci, vi_i] = compute_omega_vc_v(ri_ci, rh_i, Rh_i, qi_dot, omegah_h, vh_h, is_prismatic)
    if is_prismatic
        sigma = 1;
    else
        sigma = 0;
    end
    
    Ri_h = Rh_i';
    zh_h = [0 0 1]';
    omegai_i = simplify(Ri_h*(omegah_h + (1-sigma)*qi_dot*zh_h));
    
    
    vi_i = simplify(Ri_h*(vh_h + sigma*qi_dot*zh_h) + cross(omegai_i, Ri_h*rh_i));
    
    vi_ci = simplify(vi_i + cross(omegai_i, ri_ci)); 
end

