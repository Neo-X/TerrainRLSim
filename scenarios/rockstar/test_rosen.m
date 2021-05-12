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




%%  Instruction
%                              How to use
% 1. Set initial policy (mean and unit exploration st/andard deviation)
% 2. Fill in the name of the objective function in "functionName".
% (save your matlab function in the same folder)
% 3. Set "n_total_rollouts", the number of rollouts you want to evaluate. You
% can define different termination criteria in the while loop as well
% 4. Run!
% 5. The variable "data" contains evaluated policies and corresponding
% costs after "how_often_evaluate*(row_number-1)" rollouts
% 6. The best Policy is the best policy ever evaluated.

function best_cost=test_rosen()

%Initial policy
Initial_StandardDeviation= ones(1,20)*0.05 ; % initial exploration standard deviation: it is a row vector. e.g. =ones(1,10)
Initial_theta= zeros(1,20) ; %initial Parameter: theta is a row vector. e.g. = zeros(1,10)
n_parameter=length(Initial_theta);

%Objective Function
functionName='rosen_20';
feval=str2func(functionName);

%Misc.
how_often_evaluate=2000; %%Change if necessary

%Strategic Parameters
lambda=0.3;
n_total_rollouts=2000; %%Change if necessary
lambdaMD=10;
initial_exp=2;
expansion_factor_sigma=1.3^(1/log(n_parameter+2.5))-1;
imp_factor=1.3;

% set up the task (initial and goal state, cost function for a rollout)
covar_init=eye(n_parameter,n_parameter);
cost2policy_cov_factor=chi2inv(0.95,n_parameter)*-0.5/log(lambda);
covar = covar_init;

% setting global step size. This is to improve numerical stability.
% It is a good practice to use linear scaling for all parameters since
% it will remove numerical instability in calculating the determinant and the eigen vectors/values.
sigma=1.0;

C=covar;
determinant=det(C);
cc = 3/(n_parameter+6)/log(n_parameter+6);
ccov = 6/(n_parameter+7)/log(n_parameter+7);
pc = zeros(n_parameter,1);
chiN=n_parameter^0.5*(1-1/(4*n_parameter)+1/(21*n_parameter^2));

%% initialization
policy_history=zeros(n_total_rollouts,n_parameter);
cost_history=zeros(n_total_rollouts,1);
theta_history=zeros(n_total_rollouts,n_parameter);
theta=zeros(1,n_parameter);
theta_history(1,:)=theta.*Initial_StandardDeviation+Initial_theta;
counter_eval=1;
best_policy=theta.*Initial_StandardDeviation+Initial_theta;
best_cost=inf;

data.evaluation_cost=zeros(ceil(n_total_rollouts/how_often_evaluate),2);
data.evaluation_policy=zeros(ceil(n_total_rollouts/how_often_evaluate),n_parameter);
range=chi2inv(0.95,length(theta))*lambdaMD;

% disp('----------------------------------')
% disp('               Rock*')
% disp('----------------------------------')
% disp('rollout      noiseless policy cost    global step size')


%% Rock* algorithm

for iteration_n=1:n_total_rollouts

if mod(iteration_n-1,how_often_evaluate)==0
   % This is not a part of algorithm but gives learning feedback to the
   % user. Remove it if you want to speed up
   cost_eval = feval(theta.*Initial_StandardDeviation+Initial_theta);
   data.evaluation_cost(counter_eval,1)=iteration_n-1;
   data.evaluation_cost(counter_eval,2)=cost_eval;
   data.evaluation_policy(counter_eval,:)=theta.*Initial_StandardDeviation+Initial_theta;
   counter_eval=counter_eval+1;
%    disp(sprintf('%3.0f             %.4f                  %.5f',iteration_n-1,cost_eval,sigma));
end

  %------------------------------------------------------------------
  % search

  theta_eps_cur = mvnrnd(theta,covar);
  policy_history(iteration_n,:)=theta_eps_cur;
  costs_rollouts_cur = feval(theta_eps_cur.*Initial_StandardDeviation+Initial_theta);
  cost_history(iteration_n)=costs_rollouts_cur(1);

  if costs_rollouts_cur(1)<best_cost
      best_cost=costs_rollouts_cur(1);
      best_policy=theta_eps_cur;
  end
  
  if iteration_n>initial_exp-1

     covar_inv=inv(covar);
     
     Near_policies=zeros(n_total_rollouts,length(theta_eps_cur));
     Near_policy_costs=zeros(n_total_rollouts,1);
     
     counter=1;
     temp_coef=1;

    while(1)
      for smaple_n=max(iteration_n-n_parameter*10,1):iteration_n
        if temp_coef*range > (policy_history(smaple_n,:)-theta)*(covar_inv)*(policy_history(smaple_n,:)-theta)'
          Near_policies(counter,:)=policy_history(smaple_n,:);
          Near_policy_costs(counter,:)=cost_history(smaple_n);
          counter=counter+1;
        end
      end
      if counter>min(iteration_n,2)
        break;
      end
      temp_coef=temp_coef*3;
      counter=1;
    end
    
    Near_policies(counter:end,:)=[];
    Near_policy_costs(counter:end) = [];
    [cur_theta_new,E_cost_theta]=gradient_descent(Near_policies, Near_policy_costs,covar_inv./cost2policy_cov_factor, theta);
    [temp, IDX]=sort(Near_policy_costs);

   for i=1:min(length(Near_policy_costs),1)
        [cur_theta_new2,E_cost_theta2]=gradient_descent(Near_policies, Near_policy_costs, covar_inv./cost2policy_cov_factor, Near_policies(IDX(i),:));
        if E_cost_theta2<E_cost_theta
            cur_theta_new=cur_theta_new2;
            E_cost_theta=E_cost_theta2;
        end
   end
    
    if costs_rollouts_cur<mean(Near_policy_costs)
        [cur_theta_new2,E_cost_theta2]=gradient_descent(Near_policies, Near_policy_costs, covar_inv./cost2policy_cov_factor, theta_eps_cur);
        if E_cost_theta2<E_cost_theta
            cur_theta_new=cur_theta_new2;
            E_cost_theta=E_cost_theta2;
        end
    end
    
    % Initial from the best position
    
    if min(Near_policy_costs)~=best_cost
        [cur_theta_new2,E_cost_theta2]=gradient_descent(policy_history(1:iteration_n,:), cost_history(1:iteration_n), covar_inv./cost2policy_cov_factor, best_policy);
        if E_cost_theta2<E_cost_theta
            cur_theta_new=cur_theta_new2;
        end    
    end
    
    theta_history(iteration_n,:) = cur_theta_new;
        
    %% Covariance Matrix Adaptation
     if cost_history(iteration_n-1)>cost_history(iteration_n)
         sigma=sigma*(1+expansion_factor_sigma);
     else
         sigma=sigma/(1+expansion_factor_sigma)^(imp_factor);
     end
     
     if sqrt((cur_theta_new-theta)*covar_inv*(cur_theta_new-theta)')<chiN*1.5
         pc=(1-cc)*pc+cc * (cur_theta_new-theta)' / sigma;
         C=(1-ccov)*C+pc*pc'*ccov;
         C=C.*(determinant/det(C)).^(1/n_parameter);
     end
     
     C = triu(C) + triu(C,1)';
     covar=C.*sigma^2;
     theta=cur_theta_new;
  end
  
  if sigma<1e-8
      break;
  end
  
end
save data;

best_policy= best_policy.*Initial_StandardDeviation+Initial_theta;

end