function [T, array_Ti] = compute_kinetic_energy_T_Ti(string_joints_types, dhtable, array_q, array_q_dot, cell_ri_ci, array_m_i, array_I_ci, v0_0, omega0_0, print_info)
    T = 0;
    n = length(array_q);
    types = lower(string_joints_types);
    types = char(types);
    
    omegah_h = omega0_0;
    vh_h = v0_0;
    
    array_Ti = sym([]);
    
    [T_mat, A] = DHMatrix(dhtable);
    for i=1:n
        if types(i) == 'r'
            is_prismatic = false;
        elseif types(i) == 'p'
            is_prismatic = true;
        else
            fprintf("\nERROR\n");
        end
        
        ri_ci = cell_ri_ci{i};
        
        Ah_i = A{i};
        rh_i = affine_get_translation(Ah_i);
        Rh_i = affine_get_R(Ah_i);
        
        dq = array_q_dot(i);
        Ici_zz = array_I_ci(i);
        m_i = array_m_i(i);
        
        %link i
        [omegai_i, vi_ci, vi_i] = compute_omega_vc_v(ri_ci, rh_i, Rh_i, dq, omegah_h, vh_h, is_prismatic);
        omegai_i = simplify(omegai_i);
        vi_ci = simplify(vi_ci);
        vi_i = simplify(vi_i);
        
        Ti = simplify(konig_theorem(m_i, vi_ci, omegai_i, Ici_zz));
        array_Ti = [array_Ti, Ti];
        T = T + Ti;

        if print_info
            fprintf("Link %d------------------------------------\n", i);
            display(omegai_i);
            display(vi_ci);
            display(vi_i);
            display(Ti);
            fprintf("\n---------------------------------------------\n\n");
        end
        
        omegah_h = omegai_i;
        vh_h = vi_i;
    end
    
end

