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

function [ task ] = task_pointmassmotion(n_parameter)

task.name = 'pointmassmotion';
task.perform_rollout = @perform_rollout_pointmassmotion;
task.perform_rollout_evaluation = @perform_rollout_pointmassmotion_evaluation;

% Initial and goal state
task.y0 = 0;
task.yd0= 0;
task.g  = 1;
task.gv  = 0;
task.mass= 1;

% parameters
task.time = 1;
task.dt = 1/200;
task.viapoint = [0.5 0.5]';
task.init = ones(1,n_parameter)*0;
task.Noftimestep=ceil(task.time/task.dt);



  function cost = perform_rollout_pointmassmotion(task,theta)
    %theta scaling
%     theta(1)=theta(1)*0.001;
%     theta(3)=theta(3)*0.001;
%     theta(5)=theta(5)*0.001;
    
    trajectory.y=zeros(task.Noftimestep,1);
    trajectory.yd=zeros(task.Noftimestep,1);
    trajectory.ydd=zeros(task.Noftimestep,1);
    trajectory.y(1)=task.y0;
    trajectory.yd(1)=task.yd0;
    trajectory.ydd(1)=parameterization(0,theta,task.time)/task.mass;

    for time_step=2:task.Noftimestep
	  trajectory.y(time_step)=trajectory.y(time_step-1)+trajectory.yd(time_step-1)*task.dt+0.5*trajectory.ydd(time_step-1)*task.dt*task.dt;
      trajectory.yd(time_step)=trajectory.yd(time_step-1)+trajectory.ydd(time_step-1)*task.dt;
      trajectory.ydd(time_step)=parameterization((time_step-1)*task.dt,theta,task.time)/task.mass;
    end

    % Cost due to acceleration and final position and velocity
    torque = trajectory.ydd.*task.mass;
    cost = sum(0.5*torque.^2)*task.dt*0.03+(trajectory.y(task.Noftimestep)-task.g)^2+(trajectory.yd(task.Noftimestep)-task.gv)^2;
    %cost=theta(1)^2+theta(2)^2+theta(3)^2+theta(4)^2+theta(5);
  end



  function [y, yd, ydd, time] = perform_rollout_pointmassmotion_evaluation(task,theta)

%     theta(1)=theta(1)*0.001;
%     theta(3)=theta(3)*0.001;
%     theta(5)=theta(5)*0.001;
    
    trajectory.y=zeros(task.Noftimestep,1);
    trajectory.yd=zeros(task.Noftimestep,1);
    trajectory.ydd=zeros(task.Noftimestep,1);
    trajectory.time=zeros(task.Noftimestep,1);
    trajectory.y(1)=task.y0;
    trajectory.yd(1)=task.yd0;
    trajectory.ydd(1)=parameterization(0,theta,task.time)/task.mass;

    for time_step=2:task.Noftimestep
	  trajectory.y(time_step)=trajectory.y(time_step-1)+trajectory.yd(time_step-1)*task.dt;
      trajectory.yd(time_step)=trajectory.yd(time_step-1)+trajectory.ydd(time_step-1)*task.dt;
      trajectory.ydd(time_step)=parameterization((time_step-1)*task.dt,theta,task.time)/task.mass;
      trajectory.time(time_step)=time_step*task.dt;
    end
    
    y=trajectory.y;
    yd=trajectory.yd;
    ydd=trajectory.ydd.*task.mass;
    time=trajectory.time;

  end



end



function force=parameterization(t,theta,period)
	force=0;
	for i=1:length(theta)
		force=force+theta(i)*exp(-1*(t/period-(i-1)/(length(theta)-1))^2*length(theta)*length(theta));
	end
end
