
function [Tyre] = f_corneringStiffness(V,Vehicle,Tyre,choice_model)


switch choice_model
    case {1,2}
        OUTf_1_1 = mfeval(Tyre.Params_f , [Vehicle.Fzf/2 0 -0.01 0 0 V] , 111);
        OUTf_1_2 = mfeval(Tyre.Params_f , [Vehicle.Fzf/2 0 +0.01 0 0 V] , 111);
        OUTr_1_1 = mfeval(Tyre.Params_r , [Vehicle.Fzr/2 0 -0.01 0 0 V] , 111);
        OUTr_1_2 = mfeval(Tyre.Params_r , [Vehicle.Fzr/2 0 +0.01 0 0 V] , 111);

        Tyre.CSf = ((-2*OUTf_1_1(1,2)) -(-2*OUTf_1_2(1,2)))/(-0.01-0.01);
        Tyre.CSr = ((-2*OUTr_1_1(1,2)) -(-2*OUTr_1_2(1,2)))/(-0.01-0.01);
        Tyre.mu = (OUTf_1_2(1,18) + OUTr_1_2(1,18)) / 2;
    case {3,4}

        OUTfl_1_1 = mfeval(Tyre.Params_f , [Vehicle.Fzf/2 0 -0.01 0 0 V] , 111);
        OUTfl_1_2 = mfeval(Tyre.Params_f , [Vehicle.Fzf/2 0 +0.01 0 0 V] , 111);
        OUTfr_1_1 = mfeval(Tyre.Params_f , [Vehicle.Fzf/2 0 -0.01 0 0 V] , 111);
        OUTfr_1_2 = mfeval(Tyre.Params_f , [Vehicle.Fzf/2 0 +0.01 0 0 V] , 111);
        OUTrl_1_1 = mfeval(Tyre.Params_r , [Vehicle.Fzr/2 0 -0.01 0 0 V] , 111);
        OUTrl_1_2 = mfeval(Tyre.Params_r , [Vehicle.Fzr/2 0 +0.01 0 0 V] , 111);
        OUTrr_1_1 = mfeval(Tyre.Params_r , [Vehicle.Fzr/2 0 -0.01 0 0 V] , 111);
        OUTrr_1_2 = mfeval(Tyre.Params_r , [Vehicle.Fzr/2 0 +0.01 0 0 V] , 111);

        CSfl = ((-OUTfl_1_1(1,2)) -(-OUTfl_1_2(1,2)))/(-0.01-0.01);
        CSfr = ((-OUTfr_1_1(1,2)) -(-OUTfr_1_2(1,2)))/(-0.01-0.01);
        CSrl = ((-OUTrl_1_1(1,2)) -(-OUTrl_1_2(1,2)))/(-0.01-0.01);
        CSrr = ((-OUTrr_1_1(1,2)) -(-OUTrr_1_2(1,2)))/(-0.01-0.01);

        Tyre.CSf = CSfl + CSfr;
        Tyre.CSr = CSrl + CSrr;
        Tyre.mu = (OUTfl_1_2(1,18) + OUTfr_1_2(1,18) + OUTrl_1_2(1,18) + OUTrr_1_2(1,18)) / 4;
end
end