%
% Copyright (c) 2014, ADRL/ETHZ. Jemin Hwangbo
% All rights reserved.
%
% Redistribution and use in source and binary forms, with or without
% modification, are permitted provided that the following conditions are met:
%     * Redistributions of source code must retain the above copyright
%       notice, this list of conditions and the following disclaimer.
%     * Redistributions in binary form must reproduce the above copyright
%       notice, this list of conditions and the following disclaimer in the
%       documentation and/or other materials provided with the distribution.
%     * Neither the name of the Autonomous Systems Lab, ETH Zurich nor the
%       names of its contributors may be used to endorse or promote products
%       derived from this software without specific prior written permission.
%
% THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
% ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
% WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
% DISCLAIMED. IN NO EVENT SHALL Jemin Hwangbo BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
% OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
% GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
% HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
% OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
% SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
%

function [cur_theta_new,E_temp_prev] = gradient_descent(Near_policies, Near_costs, cov_inv, initial)

%  Input:   Near_policies, a Matrix where each column is a experienced
%  parameter vector. Dimension=Number of variables to learn X Number of
%  experiences obtained (or depends on how many you pick for simplification).
%           
%           Near_costs, a row vector where each element corresponds to cost
%           of the Near_policies. Dimension=Number of experiences obtained
%           
%           cov, noise covariance. Dimension=number of variables^2
%
%           cov_inv= inverse of cov (it is calculated prior to shorten
%           calculation time)
%
%           initial= initial point of gradient descent
%
%           cost2policy_cov_factor= currently not used
%
%  Output: cur_theta_new, minimum vector found by gradient descent,
%           Dimension= same as policy vector
%          E_temp_prev= expected cost at cur_theta_new
%
%          Note that multiple initialization is needed in more complex cost
%          functions
%



    mean_cost=mean(Near_costs);
    
    Pprior=1;
    Prev_policy=initial;
    alpha=inv(cov_inv)/(mean_cost-min(Near_costs));


    for counter=1:80
       
        % Calculate the gradient. It is an implementation of quotient rule
        % of derivative which is (adotb-bdota)/b2
        
        initial_repmat=repmat(Prev_policy,length(Near_costs),1);
    	diffOftheta=Near_policies-initial_repmat;
        MD2=dot((-0.25*diffOftheta*cov_inv)',diffOftheta');
        residual_term=diffOftheta*cov_inv.*0.5;
        expMD2=exp(MD2);
        expMD2Residual=expMD2*(residual_term);
        adotb=expMD2.*Near_costs'*(residual_term).*(sum(expMD2)+Pprior);
        bdota=expMD2Residual.*(sum(expMD2.*Near_costs')+Pprior*mean_cost);
        b2=(sum(expMD2)+Pprior)^2;
        Jaco=(adotb-bdota)./b2;
        progressV=Jaco*alpha;
        New_Policy=Prev_policy-progressV;
        Prev_policy=New_Policy;
%     progressV*cov_inv*progressV'
        % Compare and update
        if progressV*cov_inv*progressV'<0.0001||(New_Policy-initial)*cov_inv*(New_Policy-initial)'>2
           break
        end
        
        
    end
    %counter
    cur_theta_new=Prev_policy;
    initial_repmat=repmat(Prev_policy,length(Near_costs),1);
    diffOftheta=Near_policies-initial_repmat;
    expMD2=exp(dot((-0.25*diffOftheta*cov_inv)',diffOftheta'));
    E_temp_prev=(dot(expMD2,Near_costs')+mean_cost*Pprior)/(sum(expMD2)+Pprior);
end